/** Zsibot 四足机器狗 SDK ↔ ROS2 桥。
 *
 * 背景: docking 走停逻辑依赖 /odom (nav_msgs/Odometry) 闭环与 /cmd_vel (Twist)
 * 控制底盘。ZSL-1 SDK 不直接提供这两者, 但其状态回包含 IMU 位姿估计:
 *   GetPosition()      位置估计 (x, y, z)        ← 运动学+IMU 航位推算
 *   GetRPY()           姿态角 (roll, pitch, yaw)
 *   GetWorldVelocity() 世界系速度 (vx, vy, vz)
 *   GetBodyGyro()      陀螺仪角速度
 * 本桥把它们组装成 nav_msgs/Odometry 发布到 /zsibot/odom (不占用标准 /odom,
 * 避免与机器狗官方驱动或其它里程计来源冲突), 把 /cmd_vel 映射为 SetRemote
 * 摇杆量心跳下发 —— docking_node / test_* 脚本经 odom_topic 参数指向本话题即可。
 *
 * 安全设计:
 *  - cmd_vel 断流超过 cmd_timeout_sec 自动下发零速 (docking 崩溃时狗会停);
 *  - ~/emergency_stop 服务触发 SDK 软急停 (CMD_EMERGENCY_STOP);
 *  - ROLE_SDK 启动即请求控制权, 周期检查 FunctionMode 失权则重请求。
 *
 * 字段语义(抓包验证): 狗以 ~455Hz 向主机 UDP 8080 推 JSON 状态流, 其中
 *   odom_info (position 单位米, v_world/v_body/omega_body) 与
 *   imu_info  (rpy=[roll,pitch,yaw], quat w-first) 即 SDK
 *   GetPosition/GetRPY/GetWorldVelocity/GetBodyGyro 的数据源。
 * position 原点/漂移特性未知: 首帧把位置与 yaw 清零作为 odom 原点。
 * 方向/量纲用 test_turn_angle / test_jog_distance 实测验证, 不符在此调整。
 */
#include <array>
#include <chrono>
#include <cmath>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "zsibot_api.h"

using namespace std::chrono_literals;

namespace
{
double yaw_from_rpy(const std::array<float, 3> &rpy)
{
  // 已抓包验证: imu_info 的 rpy=[roll, pitch, yaw] (yaw=rpy[2] 与 w-first
  // 四元数自洽), 直接取分量即可。
  return static_cast<double>(rpy[2]);
}

void set_yaw_quaternion(nav_msgs::msg::Odometry &msg, double yaw)
{
  msg.pose.pose.orientation.x = 0.0;
  msg.pose.pose.orientation.y = 0.0;
  msg.pose.pose.orientation.z = std::sin(yaw * 0.5);
  msg.pose.pose.orientation.w = std::cos(yaw * 0.5);
}
}  // namespace

class ZsibotBridge : public rclcpp::Node
{
public:
  ZsibotBridge()
  : Node("zsibot_bridge")
  {
    dog_ip_ = declare_parameter<std::string>("dog_ip", "192.168.168.168");
    send_port_ = static_cast<uint32_t>(declare_parameter<int>("send_port", 8081));
    recv_port_ = static_cast<uint32_t>(declare_parameter<int>("recv_port", 8080));
    // 抓包证实: 狗把状态流(odom_info/imu_info/feedback/dog_state, ~455Hz JSON)
    // 固定推送到主机 8080, 不按指令包源端口回传 —— 绑其它端口收不到任何数据。
    // 因此本桥必须独占绑定 8080, 与官方 zsibot_l1_control 互斥(二选一运行):
    // 8080 被占时启动会报 bind: Address already in use。
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/zsibot/odom");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    cmd_timeout_sec_ = declare_parameter<double>("cmd_timeout_sec", 0.5);
    send_rate_hz_ = declare_parameter<double>("send_rate_hz", 50.0);
    odom_rate_hz_ = declare_parameter<double>("odom_rate_hz", 50.0);
    auto_take_control_ = declare_parameter<bool>("auto_take_control", true);

    RCLCPP_INFO(get_logger(),
                "连接机器狗 %s:%u (回传 %u) ...", dog_ip_.c_str(), send_port_, recv_port_);
    exec_ = std::make_unique<zsibot::ZsibotExecutor>(
      zsibot::Role::ROLE_SDK, dog_ip_, send_port_, recv_port_);
    RCLCPP_INFO(get_logger(), "SDK 初始化完成, 连接状态: %s",
                exec_->IsConnected() ? "已连接" : "未连接 (狗可能未上电, 将继续重试)");

    if (auto_take_control_) {
      request_control();
    }

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        last_cmd_ = *msg;
        last_cmd_ns_ = now_ns();
      });

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);

    // SetRemote 是摇杆量, 需按心跳持续下发 (SDK 示例 20ms 周期)
    auto send_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / send_rate_hz_));
    send_timer_ = create_wall_timer(send_period, std::bind(&ZsibotBridge::on_send_timer, this));

    auto odom_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / odom_rate_hz_));
    odom_timer_ = create_wall_timer(odom_period, std::bind(&ZsibotBridge::on_odom_timer, this));

    // 控制权看门狗: 失权 (如遥控器抢回) 自动重请求
    watchdog_timer_ = create_wall_timer(2s, std::bind(&ZsibotBridge::on_watchdog, this));

    estop_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/emergency_stop",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        exec_->SetCmd(zsibot::CmdCode::CMD_EMERGENCY_STOP);
        last_cmd_ = geometry_msgs::msg::Twist();
        last_cmd_ns_ = 0;   // 立即进入零速保护
        response->success = true;
        response->message = "CMD_EMERGENCY_STOP sent, cmd_vel zeroed";
        RCLCPP_WARN(get_logger(), "软急停已下发, cmd_vel 已清零");
      });

    RCLCPP_INFO(get_logger(),
                "桥就绪: /cmd_vel → SetRemote 心跳, SDK 位姿 → %s (frame: %s → %s)",
                odom_topic_.c_str(), odom_frame_.c_str(), base_frame_.c_str());
  }

private:
  int64_t now_ns()
  {
    return get_clock()->now().nanoseconds();
  }

  void request_control()
  {
    exec_->SetCmd(zsibot::CmdCode::CMD_SDK_CONTROL_RIGHT);
    RCLCPP_INFO(get_logger(), "已请求 SDK 控制权 (CMD_SDK_CONTROL_RIGHT)");
  }

  void on_send_timer()
  {
    const int64_t age_ms = last_cmd_ns_ == 0 ? -1 : (now_ns() - last_cmd_ns_) / 1'000'000;
    geometry_msgs::msg::Twist cmd;
    bool have_cmd = last_cmd_ns_ != 0 &&
      age_ms <= static_cast<int64_t>(cmd_timeout_sec_ * 1000.0);
    if (!have_cmd && !zero_sent_) {
      // cmd_vel 断流: 下发一帧零速并保持, 保证狗停下
      cmd = geometry_msgs::msg::Twist();
      zero_sent_ = true;
      if (age_ms > static_cast<int64_t>(cmd_timeout_sec_ * 1000.0)) {
        RCLCPP_WARN(get_logger(), "cmd_vel 断流 >%.1fs, 已自动下发零速", cmd_timeout_sec_);
      }
    } else if (have_cmd) {
      cmd = last_cmd_;
      zero_sent_ = false;
    } else {
      return;   // 已处于零速保持态, 无需重复发送
    }

    // SetRemote 4 元组 (ROLE_SDK 示例语义): [0]=前后, [1]=左右(左正), [2]=转向(左转/CCW 正)
    // 与 REP-103 cmd_vel 一致, 直接映射; 第 4 分量与 14 键按钮保持 0。
    std::array<float, 4> joy = {
      static_cast<float>(cmd.linear.x),
      static_cast<float>(cmd.linear.y),
      static_cast<float>(cmd.angular.z),
      0.0f};
    exec_->SetRemote(joy, std::array<float, 14>{});
  }

  void on_odom_timer()
  {
    if (!have_origin_) {
      origin_pos_ = exec_->GetPosition();
      origin_yaw_ = yaw_from_rpy(exec_->GetRPY());
      const auto pos = exec_->GetPosition();
      if (pos[0] == 0.0f && pos[1] == 0.0f && pos[2] == 0.0f) {
        return;   // 尚无有效回包, 不发 odom
      }
      have_origin_ = true;
      RCLCPP_INFO(get_logger(),
                  "收到首帧位姿, 设为 odom 原点 | 原始 pos=(%.3f, %.3f, %.3f) rpy=(%.3f, %.3f, %.3f)",
                  pos[0], pos[1], pos[2],
                  exec_->GetRPY()[0], exec_->GetRPY()[1], exec_->GetRPY()[2]);
    }

    const auto pos = exec_->GetPosition();
    const auto rpy = exec_->GetRPY();
    const auto wvel = exec_->GetWorldVelocity();
    const auto gyro = exec_->GetBodyGyro();

    nav_msgs::msg::Odometry msg;
    msg.header.stamp = get_clock()->now();
    msg.header.frame_id = odom_frame_;
    msg.child_frame_id = base_frame_;

    msg.pose.pose.position.x = static_cast<double>(pos[0]) - origin_pos_[0];
    msg.pose.pose.position.y = static_cast<double>(pos[1]) - origin_pos_[1];
    msg.pose.pose.position.z = static_cast<double>(pos[2]);

    const double yaw = normalize_yaw(yaw_from_rpy(rpy) - origin_yaw_);
    set_yaw_quaternion(msg, yaw);

    msg.twist.twist.linear.x = static_cast<double>(wvel[0]);
    msg.twist.twist.linear.y = static_cast<double>(wvel[1]);
    msg.twist.twist.linear.z = static_cast<double>(wvel[2]);
    msg.twist.twist.angular.z = static_cast<double>(gyro[2]);

    odom_pub_->publish(msg);
    log_faults_once();
  }

  void on_watchdog()
  {
    const bool connected = exec_->IsConnected();
    if (connected != was_connected_) {
      was_connected_ = connected;
      RCLCPP_WARN(get_logger(), "机器狗连接状态: %s", connected ? "已连接" : "断开");
    }
    if (auto_take_control_ && connected &&
        exec_->GetFunctionMode() != zsibot::FunctionMode::FM_SDK)
    {
      RCLCPP_WARN(get_logger(), "未处于 SDK 模式 (FunctionMode=%d), 重新请求控制权",
                  static_cast<int>(exec_->GetFunctionMode()));
      request_control();
    }
  }

  void log_faults_once()
  {
    const auto faults = exec_->GetFaultInfo();
    if (faults.empty()) {
      return;
    }
    const int64_t now = now_ns();
    if (last_fault_log_ns_ != 0 && now - last_fault_log_ns_ < 5'000'000'000LL) {
      return;
    }
    last_fault_log_ns_ = now;
    for (const auto &f : faults) {
      RCLCPP_WARN(get_logger(), "狗上报故障: %s/%s 码=%u 级=%s %s",
                  f.module.c_str(), f.submodule.c_str(),
                  f.error_code, f.level.c_str(), f.info.c_str());
    }
  }

  static double normalize_yaw(double yaw)
  {
    return std::atan2(std::sin(yaw), std::cos(yaw));
  }

  std::string dog_ip_;
  uint32_t send_port_, recv_port_;
  std::string cmd_vel_topic_, odom_topic_, odom_frame_, base_frame_;
  double cmd_timeout_sec_, send_rate_hz_, odom_rate_hz_;
  bool auto_take_control_;

  std::unique_ptr<zsibot::ZsibotExecutor> exec_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::TimerBase::SharedPtr send_timer_, odom_timer_, watchdog_timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr estop_srv_;

  geometry_msgs::msg::Twist last_cmd_;
  int64_t last_cmd_ns_ = 0;
  bool zero_sent_ = true;
  bool was_connected_ = true;
  bool have_origin_ = false;
  std::array<float, 3> origin_pos_{{0, 0, 0}};
  double origin_yaw_ = 0.0;
  int64_t last_fault_log_ns_ = 0;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<ZsibotBridge>());
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("zsibot_bridge"), "桥退出: %s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
