"""Main docking node — integrates all subsystems (stop-and-go paradigm).

Architecture:
  Subscriptions:
    /detections      → AprilTag detection array
    /tf              → (via tf2_ros.Buffer) tag pose lookup
    /odom            → odometry feedback

  Publishers:
    /cmd_vel         → (via BaseAdapter) velocity commands
    ~/state          → std_msgs/String — current state name
    ~/error          → geometry_msgs/Vector3 — (error_x, error_y, error_yaw)

  Services:
    ~/start_docking  → std_srvs/Trigger — start docking
    ~/cancel_docking → std_srvs/Trigger — cancel docking
    ~/start_undock   → std_srvs/Trigger — pull out (reverse + 180° turn)

  Action:
    ~/dock           → Dock.action — full docking with feedback

  Control loop: 20 Hz timer

    Stop-and-go paradigm:
      if action active:
          odometry dead-reckoning → check if target reached
      else:
          wait visual settle (after turn)
          read fresh tag pose → geometry planner → start next action

    Every motion is measured by odometry and stops exactly when the
    target distance/angle is reached.  No continuous PID servoing.
"""

import math
import json
import signal
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo
from rclpy.qos import (qos_profile_sensor_data, QoSProfile,
                       ReliabilityPolicy, DurabilityPolicy)
from std_msgs.msg import String
from std_srvs.srv import Trigger
from apriltag_msgs.msg import AprilTagDetectionArray
import tf2_ros

from .utils import TagPose, yaw_from_quat, normalize_angle
from .pose_buffer import PoseBuffer
from .action_executor import ActionExecutor, HeadingHold, ActionPlan
from .state_machine import (DockingStateMachine, DockingState,
                            CODE_TAG_NOT_FOUND, CODE_VISION_NO_PROGRESS,
                            CODE_MOTION_GATED, CODE_MOTION_STALLED)
from .charge_mode import ChargeMode
from .dual_docking import DualTagDocking, DEFAULTS as DUAL_DEFAULTS
from .dual_camera import CameraModel
from .dual_feedback import ActionWatch


class DockingNode(Node):
    """AprilTag auto-docking controller node — stop-and-go paradigm."""

    def __init__(self):
        super().__init__('docking_node')

        # ── Parameters ────────────────────────────────────────────
        self._declare_params()

        # ── TF2 ───────────────────────────────────────────────────
        # TF 接收走专用节点 + 专用执行器线程, 与主执行器隔离。
        # 原因: 机器上 Nav2/odin 栈在 /tf 有 ~800Hz 洪水, 监听器 depth-100
        # 队列 0.12s 灌满。控制循环 (20Hz timer) 里的 lookup_transform 带
        # 0.1s 忙等超时, 查不到时把单线程执行器占满 (20Hz×0.1s=200%), /tf
        # 队列永远不排空 → tag TF 被 DDS 挡在门外 → lookup 永远失败 →
        # 自维持死锁 (同 scripts/test_apriltag 曾出现的故障)。
        self._tf_node = Node('docking_tf_receiver')
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self._tf_node)
        # 执行器在主线程创建而不是线程里: 收栈时 main 才有句柄先 shutdown 再
        # join (见 main() finally 的顺序注释), 否则 TF 线程的 spin 没人能停。
        self._tf_executor = SingleThreadedExecutor()
        self._tf_executor.add_node(self._tf_node)
        self._tf_spin = threading.Thread(target=self._spin_tf_receiver, daemon=True)
        self._tf_spin.start()

        # 双二维码对准管理器: 仅依赖参数, 先于 PoseBuffer 创建 (时效窗取
        # dual.fresh_sec)。
        self._dual = DualTagDocking(self)

        # ── Pose buffer ───────────────────────────────────────────
        self._pose_buffer = PoseBuffer(
            max_size=self._p('pose_buffer.size'),
            max_latency_ns=int(self._dual.p('fresh_sec') * 1_000_000))

        # FIFO, not a latest-only slot: delayed TF must get a chance to arrive.
        self._dual_pending = []
        self._dual_received_ns = 0
        self._dual_window_ns = 0
        self._dual_diag_ns = {}
        # 最后一次拒收的原因与时刻 + 各原因计数: 落点 3 的子因归因 (真丢 /
        # 帧到了被否 / 停稳窗按设计丢) 全靠这两个, 只写不读判据。
        self._dual_reject_last = None
        self._dual_reject_count = {}
        self._dual_info = {}
        self._dual_watch = None
        # 连续"过期"帧的起点与最大观测延迟: 双码用固定 dual.fresh_sec 时效窗,
        # 链路延迟一旦长期超窗, 每帧都被静默丢弃, 外层只看到"从未见过 tag"而
        # 一直转圈搜索。攒够一段时间就明确报错。
        self._dual_expired_since_ns = 0
        self._dual_expired_worst_ms = 0.0
        # 移交双码前的墙码粗对准 (见 _dual_prealign): 双码枚举的步长上限只有几度,
        # 它是精调器不是收敛器; 锁定时残留的十几度方位必须先用墙码大步收掉。
        self._dual_prealigned = False
        self._dual_prealign_steps = 0
        self._dual_prealign_active = False

        # ── Action executor (odometry dead-reckoning) ─────────────
        self._executor = ActionExecutor(
            turn_settle_sec=self._p('stopgo.turn_settle_sec'),
            small_turn_rad=self._p('stopgo.small_turn_rad'),
            turn_lead_per_speed=self._p('stopgo.turn_lead_per_speed'),
            turn_slow_rad=self._p('stopgo.turn_slow_rad'),
            min_angular_rate=self._p('stopgo.min_angular_rate'),
        )

        # ── State machine ─────────────────────────────────────────
        self._sm = DockingStateMachine(self)

        # 充电收尾 (DOCKED 后 静止→阻尼泄力)
        # 需要 self._p 与 self._sm, 必须在定时器启动前创建。
        self._charge = ChargeMode(self)
        # 已规划待发的机动序列: 规划在锁定下完成后暂存, 由 _launch_pending_seq
        # 在恢复运动模式 (motion_enabled=True) 后原样启动, 解锁等待期间
        # 不重测/重规划。
        self._pending_seq: list | None = None

        # ── Base adapter ──────────────────────────────────────────
        self._adapter = self._create_adapter()

        # ── Tag tracking state ────────────────────────────────────
        self._tag_frame = ''
        self._dock_tag_id = 0
        self._camera_frame = ''

        self._last_detection_ns = 0
        self._tf_fail_count = 0
        self._det_msg_count = 0         # /detections 消息总数 (搜索停留日志诊断: 检测流是否活着)
        self._dwell_msg_start = 0       # 本次停留开始时的消息计数 (算"期内消息"差值)

        # Previous control-loop state, for detecting transitions that must
        # cancel any in-progress stop-and-go action (e.g. APPROACH→SEARCH_TAG
        # re-lock: a half-finished turn must not resume against a stale odom
        # reference when we return to APPROACH).
        self._prev_state = None

        # ── Blind turn-drive-turn maneuver ────────────────────────────
        # The planner computes a whole [turn1, drive, turn2] path from one good
        # measurement; the node runs the sub-steps back-to-back by odometry
        # (camera NOT consulted between them — the tag may leave view, that is
        # expected). Only after the full sequence completes do we settle,
        # re-measure, and iterate (iterative refine). This replaces the fragile
        # incremental aim-and-go that lost the tag at the FOV edge.
        self._maneuver_queue: list = []
        self._maneuver_active = False
        # 检测冻结标志：机动（盲转/盲走）期间为 True，此时 _on_detections 直接
        # 丢弃所有帧（运动模糊、视野边缘的坏帧绝不能污染规划用的位姿）。停稳
        # settle 结束后解冻，并清空滤波/缓冲，强制下一次规划只用停稳后的新鲜帧。
        self._frozen = False

        # ── Undock (泊出) sub-phase ──────────────────────────────────
        # 0 = 盲退 undock.backup_distance, 1 = 原地转 180°, 2 = 完成。
        # 由 _run_undock 在 UNDOCKING 态驱动, 纯里程计闭环, 不看 tag。
        self._undock_phase = 0
        # 泊出卡在哪的自述串, 每 tick 更新; 30s 兜底超时由 state_machine 取用
        # (getattr), 用来区分"桥门控没解开"和"指令发了但底盘不动"。
        self._undock_note = ''
        # 泊出超时的码也由本节点给 —— 只有它分得清这次是卡在门控 (查桥)
        # 还是指令发了没走 (查底盘); 状态机只知道"超时了"。
        self._undock_code = ''

        # Recovery-search state: remember which side the tag was last seen on
        # (sign of lat) so the angle-stepped sweep starts toward it. The node
        # rotates a fixed angle (odometry-closed), stops, detects, repeats —
        # sweeping a full 360° in one direction until the tag is found.
        self._last_seen_lat = 0.0
        self._search_step = 0
        self._search_detect_start = 0
        self._search_direction = 1.0

        # Odometry (updated in callback)
        self._odom_x = 0.0
        self._odom_y = 0.0
        self._odom_yaw = 0.0
        self._has_odom = False

        # Control period
        self._dt = 0.05  # 20 Hz

        # Quiescent cmd_vel policy: brake briefly on entering an idle/terminal
        # state, then release /cmd_vel so teleop can drive the robot.
        # Continuously publishing zero at 20 Hz otherwise monopolises the topic
        # and locks out manual control after docking. The chassis watchdog stops
        # the robot if nobody publishes.
        self._quiescent = False
        self._brake_until_ns = 0
        self._BRAKE_WINDOW_NS = int(0.3 * 1e9)   # ~0.3 s firm brake on entry

        # ── ROS interfaces ─────────────────────────────────────────
        self._init_ros_interfaces()

        # ── Timer ──────────────────────────────────────────────────
        self._timer = self.create_timer(self._dt, self._control_loop)

        # ── Shutdown ───────────────────────────────────────────────
        rclpy.get_default_context().on_shutdown(self._safe_stop)
        self._install_signal_handlers()

        self.get_logger().info(
            '停靠节点就绪 | 双码停泊 | 走停模式 | 底盘=omni (/cmd_vel)')

    # ── Parameter helpers ──────────────────────────────────────────

    def _declare_params(self):
        """Declare all ROS2 parameters with defaults."""
        # Tag
        self.declare_parameter('tag.family', '36h11')
        self.declare_parameter('tag.size', 0.16)
        self.declare_parameter('tag.frame', 'tag36h11:0')
        self.declare_parameter('tag.id', 0)
        self.declare_parameter('tag.tag_loss_timeout_sec', 2.5)

        # TF
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('base_frame', 'base_link')

        # Base
        self.declare_parameter('base.cmd_vel_topic', 'cmd_vel')

        self.declare_parameter('timeout_sec', 120.0)

        # Undock (泊出: 盲退一段距离 → 原地转 180° → 完成)
        self.declare_parameter('undock.backup_distance', 0.5)
        self.declare_parameter('undock.linear_rate', 0.08)
        self.declare_parameter('undock.turn_angle_deg', 180.0)   # 正=CCW, 负=CW
        self.declare_parameter('undock.angular_rate', 0.3)
        self.declare_parameter('undock.timeout_sec', 30.0)

        # Search
        self.declare_parameter('search.angular_speed', 0.3)
        self.declare_parameter('search.step_angle_deg', 30.0)
        self.declare_parameter('search.pause_time_sec', 1.5)
        self.declare_parameter('search.initial_look_sec', 4.0)
        self.declare_parameter('search.hold_time_sec', 0.5)
        self.declare_parameter('search.search_direction', 1)
        self.declare_parameter('search.timeout_sec', 60.0)

        # Pose buffer
        self.declare_parameter('pose_buffer.size', 30)

        # ── Dual-tag docking (双二维码对准, 唯一方案) ────────────────
        for name, default in DUAL_DEFAULTS.items():
            self.declare_parameter('dual.' + name, default)
        self.declare_parameter('dual.camera_info_topic', '/camera_sync/camera_info')
        self.declare_parameter('dual.projection_mode', 'raw')
        # 墙码边长 (36h11:0, apriltag 节点按此解 PnP; launch 侧同步透传)
        self.declare_parameter('dual.wall_tag_size', 0.15)
        # 桩码 ID/边长 (ID=51 现场已确认; 换桩改配置或 launch pile_tag_id:=)
        self.declare_parameter('dual.pile_tag_id', 51)
        self.declare_parameter('dual.pile_tag_size', 0.05)
        # 相机系站位距离: 距墙码 1.70m (= observation_distance - tolerance) 处
        # 完成双码对准 → 直行 (站立全程, 桩码 ~0.9m 在站位处可见)。
        # 它只剩 yaw_cap 远/近分档与排序校验两处用法: 桩码垂直离场放行与
        # approach 丢失闭锁改用 straight_envelope (= obs + tol = 1.90) ——
        # 直行提交发生在观察窗内任意处, 用窗下沿当放行门会在 1.7~1.9 造出
        # "要先走一步才准走第一步"的死区 (见 dual_docking.straight_envelope)。
        self.declare_parameter('dual.straight_start_distance', 1.70)
        # 相机系停泊距离: 摄像头距墙码 0.50m = 停泊完成
        self.declare_parameter('dual.dock_distance', 0.50)
        # 对准判据: 两码方位 (base_link 系 bearing) 同时 ≤ ±tol 保持 hold
        self.declare_parameter('dual.align_tolerance_deg', 3.0)
        self.declare_parameter('dual.align_hold_sec', 0.5)

        # Stop-and-go params
        self.declare_parameter('stopgo.jog_linear_rate', 0.08)
        self.declare_parameter('stopgo.jog_angular_rate', 0.3)
        # 转向角速度下限 — 必须 > l1w_control 的 min_angular_z 死区 (0.10)。
        self.declare_parameter('stopgo.min_angular_rate', 0.12)
        self.declare_parameter('stopgo.lateral_rate', 0.08)
        # 狗固件横移通道航位推算严重低估（实测 odom 0.507m / 实际约 2m，
        # 低估 ~4 倍）：判停目标 = 距离/该系数，即真实位移达到目标时停。
        self.declare_parameter('stopgo.lateral_odom_scale', 1.0)
        # 前进/后退通道同样低估（后退实测 odom 0.493m / 实际 1m+，~2 倍；
        # 前进待精标）。只有转向（IMU yaw）可信。
        self.declare_parameter('stopgo.jog_odom_scale', 1.0)
        self.declare_parameter('stopgo.jog_backward_odom_scale', 1.0)
        self.declare_parameter('stopgo.turn_settle_sec', 0.5)
        # 盲转停止滞后补偿 (scripts/test_turn_angle 同款): odom 判停到底盘
        # 真停之间的 控制周期+里程计延迟+惯性 让全速盲转每次多转 ~5-7°,
        # 法线对准在 ±2° 门槛两侧反复翻转 (对接日志: 目标 ±9.7° 实转 15-17°)。
        # lead (rad/(rad/s)): 距目标剩 lead×当前速率 时提前发零速, 滑行正好
        # 补足剩余角; 残差 ≈ (实际滞后−lead/速率)×末速。lead 过大 → 系统性
        # 欠转 (每次欠同一角度), 过小 → 依旧过冲; 若欠/过量恒定可微调本值。
        self.declare_parameter('stopgo.turn_lead_per_speed', 0.30)
        # 距目标此角度内减速到半速, 减小惯性冲量并降低提前量残差。
        self.declare_parameter('stopgo.turn_slow_rad', 0.14)
        self.declare_parameter('stopgo.max_turn_step', 0.3)
        self.declare_parameter('stopgo.small_turn_rad', 0.1)

        # ── 行进中航向保持 (双码纯直行区专用) ─────────────────────────
        # 区内一次走完剩余距离不再走停, 于是行程中新产生的 yaw 漂移没有
        # 任何停看点能纠 —— 只能边走边守。参考量取 odom/IMU yaw 相对起步
        # 的增量 (不取墙码 bearing: 盲动期没有 bearing, 且近场 bearing 的
        # 横向力臂增益正是区内禁转向的理由)。
        # 形态是**带迟滞的继电**而非比例律: l1w_control 有 min_angular_z
        # 死区 0.10, |wz|<0.10 被 clampAxis 截成 0, 比例式命令根本发不出去。
        self.declare_parameter('stopgo.heading_hold_enable', True)
        # 唯一约束: > 死区 0.10。代码另有 max(rate, min_angular_rate) 结构
        # 性抬底, 配进死区也不会静默失效。调大恶化单周期粒度。
        self.declare_parameter('stopgo.heading_hold_rate', 0.12)
        # 接通门槛: 0.47m·sin(2°)=16.4mm < dual.dock_tolerance(20mm), 即
        # "残余航向的终点横向代价刚好小于停泊容差"这一点。
        self.declare_parameter('stopgo.heading_hold_engage_deg', 2.0)
        # 断开门槛**绝不取 0**: 命令→odom 报告有 0.1-0.15s 滞后, 瞄 0 必
        # 过冲换符号 → 抖振。1.3° 迟滞带 > 最短接通粒度 1.03°。
        self.declare_parameter('stopgo.heading_hold_release_deg', 0.7)
        # 单周期 wz 脉冲底盘/步态规划器不一定响应; 3 周期 ≈ 一个步态相位。
        self.declare_parameter('stopgo.heading_hold_min_engage_sec', 0.15)
        # 覆盖命令→odom 滞后, 让下一次接通决策基于已沉降的读数。
        self.declare_parameter('stopgo.heading_hold_cooldown_sec', 0.30)
        # 不是安全边界 (那由"只降 |err|"的符号规则给), 是"底盘不响应 wz /
        # odom yaw 疯了"的诊断闸。正常一趟需 5-10°, 定低会静默关掉功能。
        self.declare_parameter('stopgo.heading_hold_budget_deg', 15.0)

        # ── 停看点量测稳定 ────────────────────────────────────────────
        # 接口前缀 base.l1w_prefix 指向 zsibot_l1_control (l1w_control),
        # charge_mode 收尾 (静止锁定/阻尼) 仍走它。
        self.declare_parameter('base.l1w_prefix', '/l1w_control')

        # 充电收尾 (DOCKED 后): 静止→阻尼泄力。
        self.declare_parameter('charge.enable', True)
        self.declare_parameter('charge.passive', True)  # 阻尼步; false=仅锁定(站立)
        self.declare_parameter('charge.static_stand', True)  # 先锁定再阻尼; false=跳过锁定直接阻尼
        self.declare_parameter('charge.static_ack_timeout_sec', 3.0)
        self.declare_parameter('charge.passive_settle_sec', 2.0)
        self.declare_parameter('charge.retries', 1)
        self.declare_parameter('charge.service_wait_sec', 1.0)

        # Detection topic
        self.declare_parameter('detection_topic', '/detections')
        self.declare_parameter('odom_topic', '/odom_combined')

    def _p(self, name: str):
        return self.get_parameter(name).value

    # ── ROS interfaces ─────────────────────────────────────────────

    def _init_ros_interfaces(self):
        """Create all ROS2 subscriptions, publishers, services, actions."""
        det_topic = self._p('detection_topic')
        self._det_sub = self.create_subscription(
            AprilTagDetectionArray, det_topic, self._on_detections, 10)

        self._dual_info_sub = self.create_subscription(CameraInfo,
            self._p('dual.camera_info_topic'), self._on_dual_camera_info, qos_profile_sensor_data)
        odom_topic = self._p('odom_topic')
        self._odom_sub = self.create_subscription(
            Odometry, odom_topic, self._on_odom, 10)

        self._state_pub = self.create_publisher(String, '~/state', 10)
        self._error_pub = self.create_publisher(Vector3, '~/error', 10)
        # ~/outcome: 每次进终态发一次的"这一轮的结果" (JSON in String)。
        # 锁存 (transient_local, depth 1) —— 上层随时订阅都能拿到最后一次结果,
        # 不用跟 20Hz 的 ~/state 抢时间窗; 按需启停下栈随时会起会收, 这点尤其
        # 要紧。QoS 四字段写法照 charge_mode.py 的桥订阅端, 本包既有先例。
        self._outcome_pub = self.create_publisher(
            String, '~/outcome',
            QoSProfile(depth=1,
                       reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))

        self._srv_start = self.create_service(
            Trigger, '~/start_docking', self._on_start_docking)
        self._srv_cancel = self.create_service(
            Trigger, '~/cancel_docking', self._on_cancel_docking)
        self._srv_undock = self.create_service(
            Trigger, '~/start_undock', self._on_start_undock)

        try:
            from tagdocking.action import Dock
        except ImportError:
            Dock = None
        if Dock is not None:
            self._action_server = ActionServer(
                self, Dock, '~/dock',
                execute_callback=self._execute_dock_cb,
                cancel_callback=self._dock_cancel_cb)
        else:
            self._action_server = None

    # ── Adapter factory ────────────────────────────────────────────

    def _create_adapter(self):
        """Create the BaseAdapter (全向底盘 OmniAdapter, 经 /cmd_vel 下发)."""
        from .base_adapter import OmniAdapter
        return OmniAdapter(self, cmd_vel_topic=self._p('base.cmd_vel_topic'))

    # ── Callbacks ──────────────────────────────────────────────────

    def _on_detections(self, msg: AprilTagDetectionArray):
        """Store detection timestamps; actual TF query happens in control loop."""
        # 计数无条件递增 (冻结早退之前): 停留日志用它区分"检测流断了"
        # 和"tag 不在视野" —— 前者计数不涨, 后者只有有效检测归零。
        self._det_msg_count += 1
        # 机动期间冻结检测：盲转/盲走过程中相机帧运动模糊、二维码常在视野边缘，
        # 这些坏帧一律丢弃，绝不更新 _pose_buffer 或 _last_detection_ns。
        # 规划器因此只会读到小车停稳后新采的帧。
        self._on_dual_detections(msg)

    def _on_odom(self, msg: Odometry):
        self._odom_x = msg.pose.pose.position.x
        self._odom_y = msg.pose.pose.position.y
        self._odom_yaw = yaw_from_quat(msg.pose.pose.orientation)
        self._has_odom = True
        self._odom_stamp_ns = msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec

    def _spin_tf_receiver(self):
        """TF 接收线程: 退出/关闭时的异常就地吞掉。

        必须用专用执行器: Humble 的 rclpy.spin() 不传 executor 时用的是
        全局单例, 与主线程共用会报 "generator already executing"。
        执行器本体在 __init__ 里创建 (句柄要交给 main 的收尾), 这里只转。
        """
        try:
            self._tf_executor.spin()
        except Exception:
            pass

    def _lookup_camera_offset(self):
        """Require full calibrated optical-to-configured-base transform."""
        if self._dual.cam_offset_known:
            return
        try:
            tf = self._tf_buffer.lookup_transform(
                self._p('base_frame'), self._p('camera_frame'),
                rclpy.time.Time(seconds=0), timeout=rclpy.duration.Duration(seconds=0))
            t, q = tf.transform.translation, tf.transform.rotation
            self._dual.set_extrinsics((t.x, t.y, t.z), (q.x, q.y, q.z, q.w))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException, ValueError) as exc:
            self._dual.failure = 'camera calibration unavailable / invalid: ' + str(exc)

    def _on_dual_camera_info(self, msg):
        try:
            model = CameraModel.from_info(msg, self._p('dual.projection_mode'), self._p('camera_frame'))
        except ValueError as exc:
            self._dual.camera = None
            self._dual.failure = 'dual CameraInfo invalid: ' + str(exc)
            self.get_logger().error(self._dual.failure, throttle_duration_sec=2.0)
            return
        stamp = msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec
        self._dual_info[stamp] = model
        while len(self._dual_info) > 128:
            del self._dual_info[next(iter(self._dual_info))]

    def _dual_log(self, key, text, now):
        # Fixed key, independent of frame values; stage changes log immediately.
        key = 'log:' + key
        previous = self._dual_diag_ns.get(key)
        stage = self._dual.stage
        if previous is None or previous[2] != stage or now < previous[0] or now-previous[0] >= self._dual.p('log_period_sec')*1e9:
            suppressed = previous[1] if previous else 0
            self.get_logger().info(text + f' suppressed={suppressed}')
            self._dual_diag_ns[key] = (now, 0, stage)
        else:
            self._dual_diag_ns[key] = (previous[0], previous[1]+1, stage)

    def _detector_saw(self, ids):
        """检测器这一帧到底看见了什么 —— 区分两个完全不同的根因。

        墙码单独没检出 (桩码还在) = 墙码自身问题 (光照/反光/运动模糊/污损);
        整帧一个码都没有 = 检测链路或图像流的问题, 与墙码无关。不分开报,
        操作员只能两边都猜。
        """
        wall_id, pile_id = int(self._p('tag.id')), self._dual.pile_tag_id
        seen = '{' + ','.join(str(i) for i in sorted(ids)) + '}' if ids else '{}(空)'
        if not ids:
            hint = '整帧一个码都没有 → 查检测链路/图像流, 不是墙码本身的问题'
        elif pile_id in ids:
            hint = f'桩码 {pile_id} 检出了、墙码 {wall_id} 单独没检出 → 查墙码本身: 光照/反光/运动模糊/污损'
        else:
            hint = f'墙码 {wall_id} 未检出 → 查墙码本身: 光照/反光/运动模糊/污损'
        return f'检测器可见={seen} ({hint})'

    def _motion_phase(self):
        """丢失发生在盲走途中 / 冻结未收口 / 已停稳 —— 三者排查方向完全不同。

        必须在 _executor.cancel() 之前调用: cancel() 把 _action 置 idle,
        is_active 随即转 False, 之后取相位只会得到"已停稳" —— 现场失败 A
        (盲走 1.00s 时被杀) 与 C (机动结束后 1.1ms 被杀) 就此再也分不开,
        而这正是本批要给出的那个区分。
        """
        confirm = self._dual.p('missing_confirm_sec')
        blind = ('locked 阶段故意绕过冻结门, 运动期的帧也能进来; 墙码无确认窗, '
                 f'一帧即杀 —— 桩码有 dual.missing_confirm_sec={confirm:.1f}s')
        if self._executor.is_active:
            return f'丢失时=盲走执行中 ({blind}) → 首先怀疑运动模糊'
        if self._frozen:
            # 帧能走到这里说明 settle 窗还没武装 (stamp < settle_until_ns 会被
            # :872 拦掉), 所以这是"机动已停、_mark_stopped 还没跑"的那一个 tick。
            return (f'丢失时=冻结未收口 (机动已结束但 _mark_stopped 尚未执行, '
                    f'停稳窗还没武装; {blind}) → 仍属运动尾段, 先查运动模糊')
        return '丢失时=已停稳 → 静止下都检不出, 查检测器/曝光/相机对焦'

    def _wall_lost_reason(self, ids, evidence, phase):
        """落点 1: locked 末段检测器确证缺墙码。ids/evidence/phase 均为抹除前快照。"""
        return ('墙码丢失 — locked 末段检测器确证这一帧里没有墙码, 立即中止; '
                + self._detector_saw(ids) + '; ' + phase
                + ('; ' + evidence if evidence else '')
                + '; 排查: 四边余量若远大于 required 就不是几何/出画问题, 是检测掉帧')

    def _wall_stale_reason(self, now_ns, phase):
        """落点 2: locked 行进中墙码流断供 (dual.fresh_sec 内无可用帧)。"""
        window = self._dual.p('fresh_sec')
        live = getattr(self, '_dual_live_wall_ns', 0) or self._dual.stamp
        age = f'{(now_ns-live)/1e6:.0f}ms' if live else '<本段从未取得墙码>'
        return (f'墙码丢失 — locked 行进中连续 {window:.2f}s (dual.fresh_sec) '
                f'内没有一帧可用墙码, 距上次可用墙码 {age}; ' + phase + '; '
                + self._dual.wall_loss_evidence()
                + '; 排查: 检测流实测有 1.6~3s 空档, 若余量宽裕则是掉帧而非几何; '
                f'必要时上调 dual.fresh_sec (当前 {window:.2f}s)')

    def _wall_loss_outer_evidence(self, now_ns):
        """落点 3 的证据 + 子因归因。

        tag_visible 为假有三个完全不同的原因, 旧串 ('dual wall stream lost')
        只说了第一个, 于是把操作员送去查相机 —— 现场失败 B 就是这么查错方向的:
          (a) 真丢     检测器从未/很久没给出墙码;
          (b) 帧到了被否 墙码一直看得见, 是采纳前的判据拒掉的;
          (c) 停稳窗   settle 窗内每帧按设计丢弃, 计数器却照涨。
        (c) 的重叠时长由状态机侧算 (它才知道丢码窗起点), 这里只给事实。
        """
        seen = getattr(self, '_dual_wall_seen_ns', 0)
        absent = getattr(self, '_dual_wall_absent_ns', 0)
        last = getattr(self, '_dual_reject_last', None)
        parts = []
        if not seen:
            parts.append('检测器本轮从未给出墙码 → (a) 真丢: 查相机/检测节点是否还在出帧')
        else:
            parts.append(f'检测器最近给出墙码={(now_ns - seen) / 1e6:.0f}ms 前')
        if absent:
            parts.append(f'最近一次"帧里确无墙码"={(now_ns - absent) / 1e6:.0f}ms 前')
        if last:
            parts.append(f'末次拒收={last[0]} ({(now_ns - last[1]) / 1e6:.0f}ms 前)')
            if seen and 'wall missing' not in last[0]:
                parts.append('→ (b) 墙码一直看得见, 帧是被采纳前的判据否掉的: '
                             '照这条拒收原因查, 别去查相机')
        parts.append(self._dual.wall_loss_evidence())
        return '; '.join(parts)

    def _dual_diagnostic(self, reason, stamp, now):
        """Per-reason throttle on the docking logger, never the video logger."""
        # 每条拒收路径都会走到这里, 所以顺手记账: 落点 3 的子因归因全靠它,
        # 不需要新增任何判断分支。getattr 兜底是因为单测的假节点是
        # cls.__new__(cls) 造的, 不跑 __init__。
        self._dual_reject_last = (reason, now)
        counts = getattr(self, '_dual_reject_count', None)
        if counts is None:
            counts = self._dual_reject_count = {}
        counts[reason] = counts.get(reason, 0) + 1
        previous = self._dual_diag_ns.get(reason)
        if previous is None or now < previous or now - previous >= 2_000_000_000:
            self._dual_diag_ns[reason] = now
            self.get_logger().info(
                f'dual detection {reason}: stamp={stamp} '
                f'age_ms={(now-stamp)/1e6:.1f} pending={len(self._dual_pending)}')

    def _note_dual_expired(self, stamp, now):
        """双码时效窗全帧拒收: 攒够一段时间就报错, 不再静默转圈搜索。

        双码走固定 dual.fresh_sec 窗 (不像单码那样按实测检测间隔自适应放宽),
        因此链路延迟一旦长期超窗, 每一帧都在 fresh() 门被丢掉: observe() 从不
        被调用 → _last_detection_ns 永不更新 → tag_visible 恒 False → 外层只
        能判定"从未见过 tag", 一路转圈搜索到 search.timeout_sec, 日志里却明明
        每帧都打着"双码发现"。这种"看得见却用不上"必须明确报错, 把实测延迟和
        窗口值一起打出来, 而不是让现场去猜。

        单帧过期是正常抖动, 只有"连续过期跨越 grace"才判为链路问题; 任何一帧
        通过时效窗都会把计时器清零 (见 _on_dual_detections)。
        """
        window = float(self._dual.p('fresh_sec'))
        grace_ns = int(max(3.0, 5.0 * window) * 1e9)
        age_ms = (now - stamp) / 1e6          # 负值 = 未来戳 (时钟不同步)
        if self._dual_expired_since_ns == 0 or now < self._dual_expired_since_ns:
            self._dual_expired_since_ns = now
            self._dual_expired_worst_ms = age_ms
        elif abs(age_ms) > abs(self._dual_expired_worst_ms):
            self._dual_expired_worst_ms = age_ms
        if now - self._dual_expired_since_ns < grace_ns:
            return
        span = (now - self._dual_expired_since_ns) / 1e9
        worst = self._dual_expired_worst_ms
        cause = ('检测时间戳超前于本地时钟 (时钟不同步)' if worst < 0 else
                 f'感知链路延迟 {worst:.0f}ms 超过时效窗 {window*1e3:.0f}ms')
        self._dual_expired_since_ns = 0
        self._dual_expired_worst_ms = 0.0
        self._adapter.publish_stop()
        self._executor.cancel()
        self._sm.abort_motion(
            f'双码时效窗连续 {span:.1f}s 拒收全部检测帧 — {cause}; '
            f'两码一直被检测到但从未进入观测 (外层因此只能转圈搜索)。'
            f'排查: 降低相机/检测链路延迟, 或上调 dual.fresh_sec '
            f'(当前 {window:.2f}s, 须 > 实测 age_ms)',
            # 归 tag_not_found 是粗粒度分族的一处已知代价: 两码其实一直看得见,
            # 真问题是感知链路延迟, 正确处置是查相机而不是重新引导进视野。
            # 上面那串已自带完整排查指引, 故接受。若以后需要独立分支,
            # 再加一族 perception_unhealthy。
            CODE_TAG_NOT_FOUND)

    def _clear_dual_expiry(self):
        """任何"非过期"的帧结局都终止连续计时 —— 只有连续过期才算链路故障。"""
        self._dual_expired_since_ns = 0
        self._dual_expired_worst_ms = 0.0

    def _discard_dual_pending(self):
        """Fence task / freeze / visual windows by ORIGINAL sensor time."""
        self._dual_pending.clear()
        self._dual_window_ns = self.get_clock().now().nanoseconds
        self._dual_received_ns = max(self._dual_received_ns, self._dual_window_ns)
        # 冻结/离开视觉态期间的丢帧与链路延迟无关, 不得计入连续过期,
        # 否则解冻后的第一帧就会拿着跨越冻结期的旧起点直接判死。
        self._clear_dual_expiry()

    def _invalidate_dual_pose(self):
        self._dual.invalidate()
        self._last_detection_ns = 0
        self._pose_buffer.clear()

    def _on_dual_detections(self, msg):
        """Keep bounded, ordered detections until their exact-time TF arrives."""
        now = self.get_clock().now().nanoseconds
        stamp = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec
        ids = {d.id for d in msg.detections}
        wall_id, pile_id = int(self._p('tag.id')), self._dual.pile_tag_id
        # 纯记账, 无判据: 检测器到底有没有出过墙码, 是"真丢"与"帧到了但被采纳
        # 前的判据否掉"的唯一分辨依据。放在状态门之前, 冻结期与终态的帧也如实
        # 计入 —— 问的是检测器出没出, 不是我们用没用。
        if wall_id in ids:
            self._dual_wall_seen_ns = stamp
        else:
            self._dual_wall_absent_ns = stamp
        # "双码发现" 已移除: 它只报告"检测器看见了", 与能否使用无关, 每 2s 一条
        # 却把真正的决策日志冲散。检测是否被采纳由 "双码有效" / dual detection
        # <reason> 两路如实反映, 信息不丢。
        if self._sm.state not in (DockingState.SEARCH_TAG, DockingState.APPROACH):
            self._discard_dual_pending()
            return
        if self._frozen and self._dual.stage != 'locked':
            self._discard_dual_pending()
            self._dual_diagnostic('rejected frozen', stamp, now)
            return
        if (stamp <= self._dual_received_ns or stamp < self._dual_window_ns
                or stamp < self._dual.settle_until_ns):
            self._dual_diagnostic('rejected duplicate/old/window', stamp, now)
            self._clear_dual_expiry()   # 水位/停稳窗拒收, 不是延迟
            return
        # Future stamps must not poison the monotonic watermark.
        if not self._dual.fresh(now, stamp):
            if 0 < stamp <= now:
                self._dual_received_ns = stamp
                self._invalidate_dual_pose()
            self._dual_diagnostic('expired/future', stamp, now)
            self._note_dual_expired(stamp, now)
            return
        # 时效通过 = 链路延迟回到窗内: 这里是"不再全帧过期"的唯一确证点,
        # 比 observe() 成功更早也更准 (observe 还会因稳定性过滤失败, 那与
        # 延迟无关, 不该把延迟计时器留着继续走)。
        self._clear_dual_expiry()
        self._dual_received_ns = stamp
        if wall_id not in ids:
            # 取证必须先于下面的抹除, 有两道抹除, 少躲一道证据就是空的:
            #   _invalidate_dual_pose / observe(..., None) → reset_filter(),
            #       把 wall/stamp/帧数清零 (margin_report 于是只剩 <none>);
            #   _executor.cancel() → _action='idle', is_active 转 False,
            #       "盲走中还是已停稳" 于是恒读成已停稳。
            if self._dual.stage == 'locked':
                evidence, phase = self._dual.wall_loss_evidence(), self._motion_phase()
            else:
                evidence = phase = ''
            # Confirmed absence supersedes older pending observations: none may
            # resurrect wall visibility after this negative observation.
            self._dual_pending.clear()
            self._invalidate_dual_pose()
            self._dual.observe(stamp, now, None)
            self._dual_diagnostic('rejected wall missing', stamp, now)
            if self._dual.stage == 'locked':
                self._dual_live_wall_ns = 0
                self._adapter.publish_stop()
                self._executor.cancel()
                self._sm.abort_motion(
                    self._wall_lost_reason(ids, evidence, phase),
                    CODE_TAG_NOT_FOUND)
            return
        # Drop NEW arrivals when full, never evict a waiting head for new frames.
        if len(self._dual_pending) >= 64:
            self._dual_diagnostic('rejected queue full', stamp, now)
            return
        self._dual_pending.append((stamp, self._dual.pile_tag_id in ids))
        self._retry_dual_detections(now)

    def _retry_dual_detections(self, now):
        """FIFO head retries are nonblocking; each detection succeeds at most once."""
        if self._sm.state not in (DockingState.SEARCH_TAG, DockingState.APPROACH):
            self._discard_dual_pending()
            return
        if self._frozen and self._dual.stage != 'locked':
            self._discard_dual_pending()
            return
        while self._dual_pending:
            stamp, has_pile = self._dual_pending[0]
            if (stamp < self._dual_window_ns or stamp < self._dual.settle_until_ns
                    or not self._dual.fresh(now, stamp)):
                self._dual_pending.pop(0)
                self._dual_diagnostic('expired/window pending', stamp, now)
                if not self._dual.fresh(now):
                    self._invalidate_dual_pose()
                continue
            model = self._dual_info.get(stamp)
            if model is None:
                self._dual_diagnostic('CameraInfo waiting', stamp, now)
                return
            self._dual.camera = model
            try:
                def point(frame):
                    tf = self._tf_buffer.lookup_transform(
                        self._p('camera_frame'), frame,
                        rclpy.time.Time(nanoseconds=stamp),
                        timeout=rclpy.duration.Duration(seconds=0))
                    ts = tf.header.stamp.sec * 1000000000 + tf.header.stamp.nanosec
                    if ts <= 0 or abs(ts-stamp) > self._dual.p('tf_skew_sec')*1e9:
                        raise ValueError('TF stamp inconsistent with detection')
                    t = tf.transform.translation
                    center = (t.x, t.y, t.z)
                    if not all(math.isfinite(v) for v in center) or t.z <= 0:
                        raise ValueError('invalid optical center')
                    return center
                wall = point(self._p('tag.frame'))
                pile = point(self._dual.pile_frame) if has_pile and self._dual.stage != 'locked' else None
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                self._dual_diagnostic('TF waiting', stamp, now)
                return  # Preserve accepted fresh pose and retry this head next tick.
            except ValueError:
                self._dual_pending.pop(0)
                self._invalidate_dual_pose()
                self._dual_diagnostic('rejected TF stamp/geometry', stamp, now)
                continue
            self._dual_pending.pop(0)
            if self._dual.stage == 'locked':
                self._dual_live_wall_ns = stamp
            if self._frozen:
                continue  # Locked travel refreshes only the independent watchdog.
            if self._dual.observe(stamp, now, wall, pile, pile_missing=not has_pile):
                if pile is not None:
                    self._dual_log('valid',
                        f'双码有效: stamp={stamp} age_ms={(now-stamp)/1e6:.1f} '
                        f'wall_z={wall[2]:.3f}m '
                        f'wall_bearing={math.degrees(math.atan2(wall[0], wall[2])):+.2f}deg '
                        f'pile_z={pile[2]:.3f}m '
                        f'pile_bearing={math.degrees(math.atan2(pile[0], pile[2])):+.2f}deg '
                        f'frames={self._dual.frames} stage={self._dual.stage} outer={self._sm.state.name}', now)
                self._last_detection_ns = stamp
                self._pose_buffer.add(TagPose(dist=wall[2], lat=-wall[0],
                    yaw=-math.atan2(wall[0], wall[2]), normal=0.0, stamp_ns=stamp))
            else:
                self._dual_diagnostic('rejected observation', stamp, now)

    def _tag_fresh(self) -> bool:
        """检测是否新鲜 (双码: dual.fresh_sec 时效窗)。"""
        return self._dual.fresh(self.get_clock().now().nanoseconds,
                                self._last_detection_ns)

    def _get_latest_pose(self):
        """Get latest valid pose from buffer (with latency check)."""
        now_ns = self.get_clock().now().nanoseconds
        if not self._dual.fresh(now_ns, self._last_detection_ns):
            return None
        self._pose_buffer.set_max_latency_ns(int(self._dual.p('fresh_sec') * 1e9))
        return self._pose_buffer.get_latest(now_ns)

    # ── Control loop (20 Hz) ───────────────────────────────────────

    def _control_loop(self):
        """Main 20 Hz control loop — stop-and-go paradigm."""
        now_ns = self.get_clock().now().nanoseconds

        self._retry_dual_detections(now_ns)

        # Gather inputs
        tag_visible = self._tag_fresh()
        tag_pose = self._get_latest_pose()

        # Compute errors (for publishing and state machine)
        error_x, error_y, error_yaw = 0.0, 0.0, 0.0
        if tag_pose is not None:
            error_x = tag_pose.dist - self._dual.target
            error_y = tag_pose.lat
            error_yaw = normalize_angle(tag_pose.yaw)

        # Final heading lock never searches, reverses or continues on a silent
        # camera. Frozen observations update only this independent wall watchdog.
        if (self._dual.stage == 'locked'
                and self._executor.is_active
                and not self._dual.fresh(now_ns, getattr(self, '_dual_live_wall_ns', self._dual.stamp))):
            # 同样必须在 cancel() 之前取相位, 否则恒读"已停稳"。
            reason = self._wall_stale_reason(now_ns, self._motion_phase())
            self._adapter.publish_stop()
            self._executor.cancel()
            self._sm.abort_motion(reason, CODE_TAG_NOT_FOUND)
        if self._sm.state == DockingState.SEARCH_TAG:
            self._lookup_camera_offset()
            if self._dual.failure:
                self._adapter.publish_stop()
                self._executor.cancel()
                self._sm.abort_motion(self._dual.failure,
                                      CODE_VISION_NO_PROGRESS)

        # State machine evaluation
        params = self._build_params_dict()
        # 墙码丢失取证经 params 下发: state_machine 对节点内部只用
        # get_logger(), 不许伸手进来, params 是既有且唯一的数据通道。
        # 传的是闭包而不是字符串: 取证要做四角投影与一串格式化, 而它每
        # 2000 个 tick 才用得上一次, 20Hz 白算是纯浪费。
        params['dual_wall_loss'] = lambda: self._wall_loss_outer_evidence(now_ns)
        params['dual_settle_until_ns'] = self._dual.settle_until_ns
        params['dual_settle_sec'] = self._dual.p('settle_sec')
        params['dual_now_ns'] = now_ns
        self._sm.evaluate(
            tag_pose=tag_pose,
            tag_visible=tag_visible,
            motion_stalled=False,  # handled by action_executor
            now_ns=now_ns,
            params=params,
            maneuver_active=self.maneuver_active,
        )

        state = self._sm.state

        # On leaving the stop-and-go states (e.g. APPROACH→SEARCH_TAG re-lock,
        # or any error/terminal transition), abort any half-finished action so
        # it cannot resume later against a stale odometry reference.
        stopgo_states = (DockingState.APPROACH,)
        if (self._prev_state in stopgo_states and state not in stopgo_states):
            self._executor.cancel()
            self._reset_maneuver()
        # 出 UNDOCKING: 终止可能还在跑的盲退/盲转, 清掉 _frozen, 否则下一次
        # 停泊会带着冻结态启动、丢弃所有检测帧。
        if (self._prev_state == DockingState.UNDOCKING
                and state != DockingState.UNDOCKING):
            self._executor.cancel()
            self._reset_maneuver()
        # 出 SEARCH_TAG：终止可能正在转的搜索步 + 解冻
        if (self._prev_state == DockingState.SEARCH_TAG
                and state != DockingState.SEARCH_TAG):
            self._reset_search()
        # 入 SEARCH_TAG (全新开始或丢标重锁)：从头开始角度步进扫描
        if (state == DockingState.SEARCH_TAG
                and self._prev_state != DockingState.SEARCH_TAG):
            self._reset_search()
        # 入 DOCKED → 启动充电收尾 (静止→阻尼泄力)。
        if (self._prev_state != state and state == DockingState.DOCKED):
            self._adapter.publish_stop()
            self._executor.cancel()
            self._charge.begin(now_ns)
        # 进终态那一沿发一次结果 (成功/失败都发)。必须在 _prev_state 被覆盖
        # 之前算, _publish_outcome 要用它判这一轮是停泊还是泊出。
        if self._prev_state != state and self._sm.is_terminal:
            self._publish_outcome(state)
        self._prev_state = state

        # ── Per-state behaviour ───────────────────────────────────
        # Active motion states own /cmd_vel and command motion each tick.
        # Quiescent states (IDLE/DOCKED/errors) must NOT keep publishing zero —
        # that monopolises /cmd_vel and locks out teleop. Brake briefly on
        # entry, then release the topic so other publishers can drive the robot.
        if state == DockingState.SEARCH_TAG:
            self._quiescent = False
            self._run_search(tag_visible, tag_pose, now_ns)

        elif state == DockingState.APPROACH:
            self._quiescent = False
            self._run_stop_and_go(tag_visible, tag_pose, now_ns)

        elif state == DockingState.UNDOCKING:
            self._quiescent = False
            self._run_undock(now_ns)

        elif state == DockingState.DOCKED:
            # 充电收尾推进 (只发模式服务, 不占 /cmd_vel —— 仍走静默策略)。
            self._brake_and_release(now_ns)
            self._charge.tick(now_ns)

        else:  # IDLE, UNDOCKED, error states
            self._brake_and_release(now_ns)

        # Publish state and error
        self._publish_state(state)
        self._publish_error(error_x, error_y, error_yaw)

    # ── Quiescent cmd_vel policy ────────────────────────────────────

    def _brake_and_release(self, now_ns: int):
        """Quiescent-state /cmd_vel policy: brake briefly on entry, then release.

        Continuously publishing Twist() at 20 Hz while idle/docked monopolises
        /cmd_vel — every teleop command is overwritten by zero within 50 ms, so
        the robot appears "locked" until the docking launch is killed. Instead
        publish a short burst of stops on the entry edge (firm brake in case the
        robot still has residual velocity), then publish nothing and let teleop
        own the topic. The chassis watchdog stops the robot if nobody publishes.
        """
        if not self._quiescent:
            self._quiescent = True
            self._brake_until_ns = now_ns + self._BRAKE_WINDOW_NS
        if now_ns < self._brake_until_ns:
            self._adapter.publish_stop()

    # ── Stop-and-go loop ───────────────────────────────────────────

    def _run_stop_and_go(self, tag_visible: bool, tag_pose, now_ns: int):
        """One tick of the turn-drive-turn stop-and-go loop.

        Three nested cases:
          1. An executor action is running  → update its odometry; when it
             finishes, immediately start the NEXT queued sub-step (no settle,
             no re-measure — the maneuver is blind).
          2. The maneuver queue just emptied → settle, then re-measure + re-plan.
          3. Idle & settled                 → measure the tag and plan a fresh
             turn-drive-turn sequence (or declare done).
        """
        # ── Case 1: a sub-step is executing ───────────────────────────
        if self._executor.is_active:
            if not self._check_dual_action(now_ns):
                return
            done = self._executor.update(
                self._odom_x, self._odom_y, self._odom_yaw, now_ns,
            )
            if done:
                # Δyaw / 航向保持已用 是第 0 步就要的现场观测量: 前者是
                # "这一程到底歪了多少"的唯一读数 (定 engage/budget 靠它),
                # 后者区分"没漂移"与"保持压根没接通"。
                hold_note = ''
                if self._executor.jog_hold_spent:
                    hold_note = ' [预算耗尽/yaw 不可信 → 本程后段纯直行]'
                plan = self._dual_watch.plan
                dyaw = math.degrees(self._executor.jog_yaw_error)
                if plan.kind == 'yaw':
                    # 实转 vs 指令: 盲转的欠转/过冲直接可读。
                    yaw_note = (f'Δyaw={dyaw:+.2f}deg'
                                f'(指令{math.degrees(plan.turn_angle):+.2f}deg)')
                else:
                    # 平移步没人命令它转 —— 这里量到的 yaw 全是寄生的。
                    # 1° 起报: 0.5×sin(1°)=8.7mm 已是 dock_tolerance(20mm)
                    # 的一半, 再大就足以单独把一步修正的收益吃光。
                    yaw_note = f'Δyaw={dyaw:+.2f}deg'
                    if abs(dyaw) >= 1.0:
                        yaw_note += '(寄生! 平移步不该转, 下一轮 J 的 theta 项会变差)'
                self.get_logger().info(
                    f'dual action COMPLETE signed_odom={self._dual_watch.signed:+.6f} '
                    f'{yaw_note} '
                    f'航向保持已用={math.degrees(self._executor.jog_hold_used):.2f}deg'
                    f'{hold_note}; awaiting settled visual feedback')

                # 粗对准步不进双码的视觉反馈账: 它没有 active_feedback
                # (未走 action_started), 记进去只会污染 feedback 判据。
                # 清标志统一在 _mark_stopped —— 那是本 tick 之后、且能同时
                # 覆盖"队列排空一步都没起来"的路径。
                if not self._dual_prealign_active:
                    self._dual.action_completed()
                self._dual_watch = None
                if self._maneuver_queue:
                    # Chain straight into the next sub-step by odometry — no
                    # settle, tag not consulted. This is the whole point: the
                    # maneuver runs open-loop on odometry so a narrow FOV losing
                    # the tag mid-turn cannot derail it.
                    self._start_next_maneuver_step()
                else:
                    # Whole sequence finished → settle before re-measuring.
                    self._maneuver_active = False
                    self._mark_stopped(now_ns)
            self._publish_action_cmd()
            return

        if now_ns < self._dual.settle_until_ns:
            self._adapter.publish_stop()
            return

        # ── Case 2: settle after a completed maneuver ─────────────────
        if self._executor.wait_visual_settle(tag_visible, now_ns):
            self._adapter.publish_stop()
            self.get_logger().info(
                '走停：机动结束后等待稳定（已过 '
                f'{(now_ns - (self._executor._last_stop_ns or now_ns))*1e-9:.2f}s）',
                throttle_duration_sec=1.0)
            return

        # ── settle 窗口刚结束：解冻并清空运动期的一切旧数据 ────────────
        # 强制下一次规划只用小车停稳后新采的新鲜帧。清空后本 tick 不规划，
        # 等 _on_detections（已解冻）收到一帧停稳后的检测重新播种滤波。
        if self._frozen:
            self._reset_visual_state()
            self._adapter.publish_stop()
            self.get_logger().info('走停：机动结束，已清空旧位姿，等待停稳后的新鲜帧重新测量')
            return

        # ── Case 2.5: 已规划待发 —— 恢复运动模式后立即原样起步 ─────────
        # 规划在锁定下已完成并暂存 _pending_seq; 等 stand_up 确认期间
        # 不重测/不重规划。
        if not self._launch_pending_seq(now_ns):
            return

        self._adapter.publish_stop()
        if not self._has_odom:
            self._sm.abort_motion('dual docking requires odometry',
                                  CODE_MOTION_STALLED)
            return
        self._lookup_camera_offset()
        # 步骤 1.5: 移交双码前先用墙码把方位粗对准到 ±prealign_tolerance。
        # 双码枚举的单步上限只有几度, 它是精调器不是收敛器 —— 锁定那一刻
        # 残留多少方位误差, 双码就得一步几度地啃回来。现场锁定时方位差
        # 20.4°, 双码要 ~30 步 × 2.4s ≈ 70s, 顶着观测超时走。
        if not self._dual_prealign(tag_visible, tag_pose, now_ns):
            return
        seq = self._dual.plan_dual(tag_pose, now_ns)
        if self._dual.failure:
            self._executor.cancel()
            self._sm.abort_motion(self._dual.failure,
                                  CODE_VISION_NO_PROGRESS)
        elif self._dual.complete:
            self._executor.cancel()
            self._pending_seq = None
            self._adapter.publish_stop()
            self._sm.finish_dual()
        elif seq:
            self._pending_seq = list(seq)
            self._launch_pending_seq(now_ns)
        return

    def _run_undock(self, now_ns: int):
        """泊出的一个 tick: 盲退 → 原地转 180° → UNDOCKED。

        两段纯里程计闭环盲动顺序执行：
          phase 0: 盲退 undock.backup_distance (负 jog)
          phase 1: 原地转 undock.turn_angle_deg (默认 180°)
        两段都到位后调状态机 finish_undock() → UNDOCKED。
        不看 tag；运动期冻结检测 (与停泊盲动一致)。
        """
        # Case 1: 子动作执行中
        if self._executor.is_active:
            self._undock_note = (f'phase{self._undock_phase} '
                                 + self._executor.progress_note(self._odom_x,
                                                                self._odom_y))
            # 指令已经在发了, 超时就意味着里程计没跟上 → 查底盘, 不是查桥。
            self._undock_code = CODE_MOTION_STALLED
            done = self._executor.update(
                self._odom_x, self._odom_y, self._odom_yaw, now_ns,
            )
            if done:
                self._maneuver_active = False
                self._executor.mark_stop_time(now_ns)
                self._undock_phase += 1
                if not self._start_undock_step():
                    # 两段盲动均完成 → 泊出成功
                    self._adapter.publish_stop()
                    self._sm.finish_undock()
                    self._charge.reset()
                    return
            self._publish_action_cmd()
            return

        # Case 2: 首次进入 → 启动第一段(盲退)
        if not self._has_odom:
            self._adapter.publish_stop()
            self._undock_note = '等里程计'
            self._undock_code = CODE_MOTION_STALLED
            self.get_logger().warn('泊出：等待里程计...', throttle_duration_sec=1.0)
            return
        # DOCKED 收尾后狗锁定/阻尼泄力, cmd_vel 被桥拒绝 → 泊出超时。
        # 先由 charge 管理器 stand_up 恢复运动模式, 它不门控时直接放行。
        if not self._charge.motion_ready(now_ns):
            self._adapter.publish_stop()
            self._undock_note = (f'cmd_vel 被桥门控, 等 stand_up '
                                 f'(charge={self._charge.phase_name})')
            # 卡在门控: 盲重试没用, 得去查桥/电机泄力。
            self._undock_code = CODE_MOTION_GATED
            return
        self._undock_phase = 0
        if not self._start_undock_step():
            self._adapter.publish_stop()
            self._sm.finish_undock()
            self._charge.reset()
            return
        self._publish_action_cmd()

    def _start_undock_step(self) -> bool:
        """启动 _undock_phase 指示的泊出子动作。

        phase 0 = 盲退, phase 1 = 原地转 180°。
        返回 True = 已启动一个动作; False = 该相位空跳或已全部完成(调用方据此收尾)。
        空跳(距离/角度过小)时自动推进到下一相位再试, 与 _start_next_maneuver_step
        的"跳过过小子步"行为一致。
        """
        if self._undock_phase == 0:
            dist = abs(float(self._p('undock.backup_distance')))
            rate = float(self._p('undock.linear_rate'))
            if dist < 1e-3:
                self._undock_phase += 1   # 距离为 0, 跳过盲退直接转
            else:
                self._executor.start_jog(
                    -dist, rate,
                    odom_scale=self._p('stopgo.jog_backward_odom_scale'))
                self._executor.set_odom_ref(
                    self._odom_x, self._odom_y, self._odom_yaw)
                self._maneuver_active = True
                self._frozen = True
                self._discard_dual_pending()
                self.get_logger().info(
                    f'泊出：盲退 {-dist:+.3f}m (速率 {rate:.2f}m/s)')
                return True
        if self._undock_phase == 1:
            angle = math.radians(float(self._p('undock.turn_angle_deg')))
            rate = float(self._p('undock.angular_rate'))
            if self._executor.start_turn(angle, rate):
                self._executor.set_odom_ref(
                    self._odom_x, self._odom_y, self._odom_yaw)
                self._maneuver_active = True
                self._frozen = True
                self._discard_dual_pending()
                self.get_logger().info(
                    f'泊出：原地转 {math.degrees(angle):+.1f}°')
                return True
            self._undock_phase += 1   # 角度过小未启动 → 视为完成
        return False

    # ── Angle-stepped search loop ─────────────────────────────────

    def _run_search(self, tag_visible: bool, tag_pose, now_ns: int):
        """角度步进搜索的一个 tick：转固定角度(里程计闭环)→停稳→检测→再转。

        转满 360° 直到找到二维码或状态机超时。结构镜像 _run_stop_and_go 的
        Case 级联（执行中→等待稳定→解冻清旧数据→空闲检测），区别是检测期
        不规划靠近动作，而是累计 pause_time_sec 不可见就再转一个 step_angle。
        冻结机制保证转动期 tag_visible 恒为 False，状态机的 tag-lock 不会误触发。
        """
        # ── Case 1: 搜索步正在执行（里程计闭环盲转）──────────────
        if self._executor.is_active:
            done = self._executor.update(
                self._odom_x, self._odom_y, self._odom_yaw, now_ns,
            )
            if done:
                self._maneuver_active = False
                self._mark_stopped(now_ns)
            self._publish_action_cmd()
            return

        if now_ns < self._dual.settle_until_ns:
            self._adapter.publish_stop()
            return

        # ── Case 2: 转完后等待图像稳定 ────────────────────────────
        if self._executor.wait_visual_settle(tag_visible, now_ns):
            self._adapter.publish_stop()
            return

        # ── Case 2.5: 稳定窗口刚结束 → 解冻，强制下一帧只用停稳后的新鲜帧
        if self._frozen:
            self._reset_visual_state()
            self._search_detect_start = 0
            self._adapter.publish_stop()
            return

        # ── Case 3: 空闲且已稳定 — 检测期 ─────────────────────────
        if not self._has_odom:
            self._adapter.publish_stop()
            self.get_logger().warn('搜索：等待里程计...', throttle_duration_sec=1.0)
            return

        # 二维码可见 → 原地停住，状态机累积 hold 转 APPROACH。
        # 重置检测停留计数，让闪烁的二维码每次消失都重获完整停留窗口。
        if tag_visible and tag_pose is not None:
            self._search_detect_start = 0
            self._adapter.publish_stop()
            return

        # 检测停留：累计 pause_time_sec 的持续不可见，然后转下一步。
        # 第 0 步停留加长 (initial_look_sec)：开局 RTSP/检测流冷启动，
        # tag 可能就在视野里而流还没出帧，别急着转走。
        pause_time = self._p('search.pause_time_sec')
        if self._search_step == 0:
            pause_time = max(pause_time, self._p('search.initial_look_sec'))
        if self._search_detect_start == 0:
            self._search_detect_start = now_ns
            self._dwell_msg_start = self._det_msg_count
            self._adapter.publish_stop()
            self.get_logger().info(
                f'搜索：检测停留 (步数={self._search_step}, '
                f'停留={pause_time:.1f}s)',
                throttle_duration_sec=1.0)
            return

        if (now_ns - self._search_detect_start) * 1e-9 < pause_time:
            self._adapter.publish_stop()
            return

        if (self._search_step * abs(float(
                self._p('search.step_angle_deg'))) >= 360.0):
            self._adapter.publish_stop()
            self._sm.abort_motion(
                'dual bounded wall search exhausted one revolution',
                CODE_TAG_NOT_FOUND)
            return

        # 停留期满仍未见到 → 转下一步。诊断：这段停留里 /detections 到了几条？
        # 0 条 = 检测流断了 (相机/桥/apriltag 问题)，而非 tag 不在视野。
        self.get_logger().info(
            f'搜索：停留 {pause_time:.1f}s 未见 tag '
            f'(步数={self._search_step}, '
            f'期内消息={self._det_msg_count - self._dwell_msg_start}) → 转下一步')
        self._search_detect_start = 0
        self._search_step += 1
        step_angle = math.radians(self._p('search.step_angle_deg'))
        angle = step_angle * self._search_direction
        rate = self._p('search.angular_speed')
        if self._executor.start_turn(angle, rate):
            self._executor.set_odom_ref(
                self._odom_x, self._odom_y, self._odom_yaw)
            self._maneuver_active = True
            self._frozen = True
            self._discard_dual_pending()
            self.get_logger().info(
                f'搜索：第{self._search_step}步 原地转 '
                f'{math.degrees(angle):+.1f}° '
                f'(方向={"CCW" if angle > 0 else "CW"})')
        else:
            self._frozen = False
            self._maneuver_active = False
        self._publish_action_cmd()

    def _start_next_maneuver_step(self):
        """Pop and start the next queued sub-step, re-referencing odometry.

        Skips over sub-steps too small to actually move (a sub-degree turn or a
        sub-millimetre jog): those would leave the executor idle, so without the
        skip the blind chain would stall (Case 1 only advances the queue when an
        action was running). Keeps popping until one step starts or the queue
        empties.
        """
        while self._maneuver_queue:
            step = self._maneuver_queue.pop(0)
            if self._launch_step(step):
                return
        # Queue drained without launching anything → maneuver is over.
        self._maneuver_active = False
        self._mark_stopped(self.get_clock().now().nanoseconds)

    def _dual_prealign(self, tag_visible: bool, tag_pose,
                       now_ns: int) -> bool:
        """移交双码前用墙码把方位粗对准。True = 可以进入双码。

        为什么需要它: 双码 _correction 枚举的单步上限是 yaw_cap (远场 8°、近场
        3°), 每步还要停稳-重测-重规划 ~2.4s。它的定位是"精调器" —— 用两码的
        地平面几何把 theta/e 双自由度收进毫米/度级, 而不是从十几度的初始误差
        开始收敛。现场 `二维码已锁定：距离=1.371m` 之后直接移交双码, 锁定
        时的方位误差 (实测 20.4°) 没有任何粗对准, 双码只能一步几度地啃, ~30 步
        × 2.4s ≈ 70s 顶着 dual.observe_timeout_sec (120s) 走, 必然失败。

        为什么用墙码: 墙码 (0.15m, 挂墙上) 正对视角最清楚, 桩码本来就
        看不见 —— 粗对准只需要墙码。

        为什么用单码而非双码的 geometry(): 粗对准只要收一个自由度 (车头朝向
        墙码), 墙码 bearing = atan2(lat, dist) 是直接量测, 不依赖两码基线、
        不会因为桩码缺失而无解。精度不够正是交给双码的理由。

        预算耗尽不判失败, 只告警后移交: 粗对准是加速器, 收敛与失败判定的责任
        在双码 (它有 observe_timeout / max_actions / 看门狗)。两处都判失败会
        让现场同一个故障出现两种说法。
        """
        if self._dual.stage != 'acquire' or self._dual_prealigned:
            # 粗对准只属于 acquire 相位 (双码还没拿到过一次有效双码观测)。
            # 一旦进了 observe/approach/locked, 朝向由双码几何或锁定直行负责,
            # 单码 bearing 再插手只会和双码抢方向盘。
            return True
        tol = math.radians(self._dual.p('prealign_tolerance_deg'))
        budget = int(self._dual.p('prealign_max_steps'))
        if not tag_visible or tag_pose is None:
            self._adapter.publish_stop()
            self.get_logger().info('单码粗对准: 等待停稳后的墙码新鲜帧',
                                   throttle_duration_sec=1.0)
            return False
        bearing = math.atan2(tag_pose.lat, tag_pose.dist)
        if abs(bearing) <= tol:
            self._dual_prealigned = True
            self.get_logger().info(
                f'单码粗对准完成: 墙码方位={math.degrees(bearing):+.2f}deg '
                f'≤ {math.degrees(tol):.1f}deg (用了 {self._dual_prealign_steps} 步, '
                f'距离={tag_pose.dist:.3f}m) → 交给双码精调')
            return True
        if self._dual_prealign_steps >= budget:
            self._dual_prealigned = True
            self.get_logger().warn(
                f'单码粗对准预算耗尽 ({budget} 步) 仍有方位 '
                f'{math.degrees(bearing):+.2f}deg > {math.degrees(tol):.1f}deg — '
                f'仍交给双码 (由其超时/看门狗判定), 但入桩概率低。'
                f'排查: 转向是否真的执行 (看 signed_odom)、'
                f'dual.prealign_step_deg 是否太小、墙码量测是否抖动')
            return True
        # bearing > 0 = 墙码在车左 → 左转 (CCW, turn_angle > 0)。与双码的
        # 光学 bearing 符号相反 (光学 x 向右), 这里用的是 base 系 lat。
        step = min(math.radians(self._dual.p('prealign_step_deg')),
                   float(self._p('stopgo.max_turn_step')), abs(bearing))
        self._dual_prealign_steps += 1
        self.get_logger().info(
            f'单码粗对准 #{self._dual_prealign_steps}/{budget}: 墙码 '
            f'距离={tag_pose.dist:.3f}m 横向={tag_pose.lat:+.3f}m '
            f'方位={math.degrees(bearing):+.2f}deg (门槛 {math.degrees(tol):.1f}deg) '
            f'→ 原地转 {math.degrees(math.copysign(step, bearing)):+.2f}deg')
        # _dual_prealign_active 让 _launch_pending_seq / _launch_step 把这一步
        # 当成"非双码"处理: 不查 dual.pending_valid (双码这会儿还没规划过,
        # 没有 pending_plan), 也不记进 dual 的动作预算/合格状态。看门狗照装 ——
        # 底盘不动 (死区/门控) 必须现在就炸, 而不是拖到双码去误判几何。
        self._dual_prealign_active = True
        self._pending_seq = [ActionPlan(kind='yaw',
                                        turn_angle=math.copysign(step, bearing))]
        self._launch_pending_seq(now_ns)
        return False

    def _launch_pending_seq(self, now_ns: int) -> bool:
        """处理已规划的待发序列。返回 False = 本 tick 到此为止, 调用方立即 return
        (仍在等 stand_up 解锁, 或已起步 —— 起步后若贯穿落入 Case 3 会在同一
        tick 重测重规划、覆写刚启动的队列); True = 无待发序列, 继续量测规划。

        规划完成后序列暂存 _pending_seq, 由本方法在静止站立解锁确认
        (motion_enabled=True) 后原样启动 —— 解锁等待期间不重测/不重规划,
        规划日志因此每停只打一次。启动后冻结检测 (盲动期丢弃所有帧)。
        """
        if self._pending_seq is None:
            return True
        if (not self._dual_prealign_active
                and not self._dual.pending_valid(now_ns)):
            self._pending_seq = None
            self._adapter.publish_stop()
            self._dual.stopped(now_ns)
            self._reset_visual_state()
            return False
        seq, self._pending_seq = self._pending_seq, None
        self._maneuver_queue = list(seq)
        self._maneuver_active = True
        self._frozen = True          # 开始盲动：冻结检测，运动期丢弃所有帧
        self._discard_dual_pending()
        self._start_next_maneuver_step()
        return False

    def _launch_step(self, plan: ActionPlan) -> bool:
        """Start one executor action from an ActionPlan and re-ref odometry.

        Returns True if an action actually started, False if the step was too
        small to move (caller advances to the next queued step).
        """
        now = self.get_clock().now().nanoseconds
        stamp = getattr(self, '_odom_stamp_ns', 0)
        if self._executor.is_active:
            return False  # No new start, budget or qualification commit.
        if (stamp <= 0 or not 0 <= now-stamp <= self._dual.p('odom_fresh_sec')*1e9
                or not all(math.isfinite(v) for v in (self._odom_x, self._odom_y, self._odom_yaw))):
            self._adapter.publish_stop()
            self._sm.abort_motion(
                'dual requires fresh odometry before action start',
                CODE_MOTION_STALLED)
            return False
        if plan.kind == 'yaw':
            # 全量盲转: 整条机动路径是一次算好的, 钳半截会把后续直行腿带偏
            # 航向; 里程计闭环 + 每停重测兜底。
            if not self._executor.start_turn(
                    plan.turn_angle, self._p('stopgo.jog_angular_rate')):
                return False
            self._executor.set_odom_ref(
                self._odom_x, self._odom_y, self._odom_yaw)
            self.get_logger().info(
                f'  子步：原地转 {math.degrees(plan.turn_angle):+.1f}°（盲转，里程计校准）')
        elif plan.kind == 'forward':
            if abs(plan.lateral_distance) > 1e-4:
                self._executor.start_jog_lateral(
                    plan.lateral_distance, self._p('stopgo.lateral_rate'),
                    odom_scale=self._p('stopgo.lateral_odom_scale'))
                if not self._executor.is_active:
                    return False
                self._executor.set_odom_ref(
                    self._odom_x, self._odom_y, self._odom_yaw)
                self.get_logger().info(
                    f'  子步：横移 {plan.lateral_distance:+.3f}m')
            else:
                # Blind straight leg — odometry only, no visual early-stop.
                # 平移里程计按通道低估，判停目标除以对应通道系数。
                scale = (self._p('stopgo.jog_odom_scale')
                         if plan.jog_distance >= 0
                         else self._p('stopgo.jog_backward_odom_scale'))
                # 航向保持只给双码纯直行区那一步"一次走完"的长直行。复用
                # plan.continuous 而不新增字段: 它的产出点只有一处
                # (dual_docking._advance 区内分支), 语义恰好是"区内禁转向、
                # 一步走完剩余距离", 与要保持航向的区间完全同一。
                # 排除项都是有意的: 回退修剪 (continuous=False, ≤10cm/1.25s,
                # 短到攒不出 engage 门槛的漂移, 且倒走叠 wz 的运动学未验证)、
                # 区外 forward_step 逐步走 (那里墙码 bearing 微调活着, 每停
                # 都在纠方向)、泊出/重试盲腿 (根本不经过这里)。
                hold = self._heading_hold_params() if (
                    plan.continuous
                    and plan.jog_distance > 0) else None
                self._executor.start_jog(
                    plan.jog_distance, self._p('stopgo.jog_linear_rate'),
                    odom_scale=scale, hold=hold)
                if not self._executor.is_active:
                    return False
                self._executor.set_odom_ref(
                    self._odom_x, self._odom_y, self._odom_yaw)
                self.get_logger().info(
                    f'  子步：前进 {plan.jog_distance:+.3f}m（盲走，里程计校准）')
        else:
            return False
        speed = (self._executor.angular_cmd if plan.turn_angle else
                 self._executor.lateral_cmd if plan.lateral_distance else self._executor.linear_cmd)
        self._dual_watch = ActionWatch(plan, now,
            (self._odom_x, self._odom_y, self._odom_yaw),
            self._executor._action_target, speed, self._dual.p)
        if not self._dual_prealign_active:
            self._dual.action_started(plan, now)
        self._publish_action_cmd()
        return True

    def _check_dual_action(self, now):
        watch = self._dual_watch
        reason = ('dual missing action start reference' if watch is None else watch.check(
            now, (self._odom_x,self._odom_y,self._odom_yaw), getattr(self,'_odom_stamp_ns',0)))
        if reason:
            if self._dual_prealign_active:
                # 现场必须能一眼分清"粗对准阶段底盘没动"和"双码精调出问题":
                # 前者是墙码粗对准转向 (死区/门控/服务), 后者是双码几何。
                reason = '单码粗对准阶段 — ' + reason
            self._adapter.publish_stop()
            self._executor.cancel()
            self._pending_seq = None
            self._maneuver_queue = []
            self._maneuver_active = False
            self._dual.failure = reason
            self._dual._travel_qualification = 0
            self._sm.abort_motion(reason, CODE_VISION_NO_PROGRESS)
            return False
        return True

    @property
    def maneuver_active(self) -> bool:
        """True while a blind turn-drive-turn maneuver is executing.

        During this window the tag legitimately leaves view, so the state
        machine must NOT count tag-loss toward the SEARCH_TAG fallback.
        """
        return self._maneuver_active or self._executor.is_active

    def _publish_action_cmd(self):
        """Publish the current action's velocity command via the adapter."""
        kind = self._executor.action_kind
        if kind == 'jogging':
            angular = self._executor.angular_cmd
            # angular == 0 时**必须**走 publish_jog: publish_arc 会把零角速度
            # 退化成纯原地转, 把前进速度整个丢掉。这道分支让"未接通"这条常态
            # 路径与航向保持上线前逐位相同, arc 只出现在真正接通的那零点几秒。
            if angular:
                self._adapter.publish_arc(self._executor.linear_cmd, angular)
            else:
                self._adapter.publish_jog(self._executor.linear_cmd)
        elif kind == 'turning':
            # 纯原地转弯。"转向时叠加前进速度"(arc) 已弃用: 规划器要的是原地
            # 转 θ 度, 叠上 vx 会让车沿弧驶出目标横向范围。转向精度靠里程计
            # 校准 (全量盲转), 转得慢没关系; 差速轮原地转若需克服静摩擦,
            # 宁可加大 jog_angular_rate, 也不叠前向速度。
            # 注意与上面 jogging 分支的 arc 区分, 那是方向相反的另一件事:
            # 这条禁的是"本该只转、却混进了走", 那条是"本该只走、要守住不歪"
            # (行进中航向保持, 见 HeadingHold)。
            self._adapter.publish_turn(self._executor.angular_cmd)
        elif kind == 'lateral':
            self._adapter.publish_lateral(self._executor.lateral_cmd)
        else:
            self._adapter.publish_stop()

    def _mark_stopped(self, now_ns: int):
        """停稳边界: 视觉 settle 时钟起点。

        泊出 phase0→1 的链式段故意不走这里 —— 无缝盲链中间没有停看点,
        settle 时钟只发生在确实要"停下来看"的时刻。
        """
        self._executor.mark_stop_time(now_ns)
        # 粗对准那一步到此结束 —— 标志必须在"停稳"这个唯一收口处清掉, 而不是
        # 只在动作正常完成时清: 队列排空一步都没起来 (步长太小被跳过) 也走这里。
        # 漏清的后果是静默的: 之后真正的双码动作会被当成粗对准步, 既不查
        # pending_valid 也不记 action_started, 双码的预算/合格状态全部作废。
        self._dual_prealign_active = False
        self._dual.stopped(now_ns)

    def _reset_visual_state(self):
        """解冻 + 丢弃运动期全部旧位姿。

        强制下一次规划只用停稳解冻后新采的新鲜帧 (EMA 重新播种)。
        """
        self._frozen = False
        self._last_detection_ns = 0        # _tag_fresh() 归零，等待新帧
        if hasattr(self, '_dual_live_wall_ns'):
            del self._dual_live_wall_ns    # 新锁定回退到本窗口 dual.stamp，绝不沿用旧任务
        self._pose_buffer.clear()          # 丢弃所有历史缓冲位姿
        self._dual.reset_filter()
        self._discard_dual_pending()
        self._dual.settle_until_ns = max(self._dual.settle_until_ns,
            self.get_clock().now().nanoseconds)          # 桩码 EMA 同步丢弃 (hold 跨停保持)

    def _reset_maneuver(self):
        """Clear any queued/active blind maneuver.

        Called on cancel/abort and whenever we leave the stop-and-go states, so
        a fresh docking attempt always re-plans from a new measurement and no
        stale sub-step can resume against an outdated odometry reference.
        """
        self._maneuver_queue = []
        self._maneuver_active = False
        self._pending_seq = None
        self._reset_visual_state()
        self._dual_watch = None
        self._dual_diag_ns.clear()
        self._dual_reject_last = None
        self._dual_reject_count.clear()
        self._clear_dual_expiry()          # 新一轮从零计延迟, 不继承上轮
        # 新一轮 dock 重做粗对准: 上一轮结束时的朝向与本轮无关 (中间可能
        # 搜索转了一圈、也可能倒车重锁), 预算同样从零。
        self._dual_prealigned = False
        self._dual_prealign_steps = 0
        self._dual_prealign_active = False
        self._dual.reset()                 # 复位双码相位闩 (直行/对准保持)

    def _reset_search(self):
        """重置角度步进搜索：计数器、方向、执行器、冻结态。

        进入 SEARCH_TAG 时按最后见到二维码的一侧选初始方向（+lat=左=CCW→+1），
        从未见过则退回 search.search_direction；之后始终同向，12 步转满 360°。
        出 SEARCH_TAG 时也调用，终止可能正在转的搜索步并解冻。
        """
        self._search_step = 0
        self._search_detect_start = 0
        if self._last_seen_lat > 0.0:
            self._search_direction = 1.0
        elif self._last_seen_lat < 0.0:
            self._search_direction = -1.0
        else:
            self._search_direction = 1.0 if self._p('search.search_direction') >= 0 else -1.0
        self._executor.cancel()
        self._reset_visual_state()
        self._maneuver_active = False

    # ── Helpers for the action executor ────────────────────────────

    def _heading_hold_params(self) -> HeadingHold | None:
        """行进中航向保持的继电参数; None = 关闭 (等价于 wz≡0 的老行为)。

        每次起步现读 (_p 是 get_parameter().value), 所以现场
        `ros2 param set /docking_node stopgo.heading_hold_engage_deg 3.0`
        下一趟就生效, 不必重启 —— 这几个值正是最需要在现场边跑边调的。

        release >= engage 是唯一会让状态机行为失去意义的配法 (接通即断开,
        或断开门槛比接通还松), 这里 warn 后关闭保持而不是 raise: 参数配错
        不该让一场本可成功的停泊崩掉 —— 同 _locked_bearing "把锦上添花的
        微调变成整场健康直行的中止是错的交易"。

        budget < 一次最短接通 (rate × min_engage) 是另一种配错, 且是**最坏
        的那种**: 接通门里那条 "used + rate×min_engage < budget" 在 used=0
        时就不成立, 于是一次都接不通, used 恒为 0、jog_hold_spent 也永远
        不会被置上 —— 完成日志打出的 "航向保持已用=0.00deg" 与"漂移没到
        门槛、本就不需要纠"**逐字相同**, 静默关掉整个功能且无法从日志分辨。
        所以这里必须显式拦下并说清楚, 而不是让它假装开着。
        (用抬底后的 rate 比较: start_jog 会把 rate 抬到 min_angular_rate 之上,
        拿配置原值算门槛会算小, 守卫本身就漏。)
        """
        if not bool(self._p('stopgo.heading_hold_enable')):
            return None
        engage = math.radians(self._p('stopgo.heading_hold_engage_deg'))
        release = math.radians(self._p('stopgo.heading_hold_release_deg'))
        if not release < engage:
            self.get_logger().warn(
                f'航向保持参数无效: release({math.degrees(release):.2f}deg) 必须 '
                f'< engage({math.degrees(engage):.2f}deg) —— 本趟关闭航向保持, '
                f'退回纯直行')
            return None
        rate = max(abs(float(self._p('stopgo.heading_hold_rate'))),
                   float(self._p('stopgo.min_angular_rate')))
        min_engage = float(self._p('stopgo.heading_hold_min_engage_sec'))
        budget = math.radians(self._p('stopgo.heading_hold_budget_deg'))
        if budget <= rate*min_engage:
            self.get_logger().warn(
                f'航向保持参数无效: budget({math.degrees(budget):.2f}deg) 不足一次'
                f'最短接通 ({math.degrees(rate*min_engage):.2f}deg = rate {rate:.2f}'
                f'rad/s × min_engage {min_engage:.2f}s) —— 这样配一次都接不通, '
                f'且日志与"没漂够门槛"无法分辨。本趟关闭航向保持, 退回纯直行 '
                f'(要真开就把 budget 调到 {math.degrees(rate*min_engage)*3:.0f}deg 以上)')
            return None
        return HeadingHold(
            rate=float(self._p('stopgo.heading_hold_rate')),
            engage=engage, release=release,
            min_engage_ns=int(min_engage*1e9),
            cooldown_ns=int(self._p('stopgo.heading_hold_cooldown_sec')*1e9),
            budget=budget)

    # ── Params dict ────────────────────────────────────────────────

    def _build_params_dict(self) -> dict:
        """state_machine.evaluate 的参数包: 只含它还消费的键。"""
        return {
            'timeout_sec': self._p('timeout_sec'),
            'tag': {
                'tag_loss_timeout_sec': self._p('tag.tag_loss_timeout_sec'),
            },
            'search': {
                'hold_time_sec': self._p('search.hold_time_sec'),
                'timeout_sec': self._p('search.timeout_sec'),
            },
            'undock': {
                'timeout_sec': self._p('undock.timeout_sec'),
            },
        }

    # ── Publishing ─────────────────────────────────────────────────

    def _publish_state(self, state: DockingState):
        msg = String()
        msg.data = state.name.lower()
        self._state_pub.publish(msg)

    def _publish_outcome(self, state: DockingState):
        """进终态那一沿发一次"这一轮的结果" —— 给上层应用分支用的出口。

        为什么单开一个锁存话题, 而不是塞进 ~/state 或靠 action:
        - ~/state 只发状态名, 且 supervisor/web 都按裸名消费 (还有 startswith
          消费者), 不能改它的载荷。
        - Trigger 服务立即返回, 失败发生在几十秒之后, 结构上带不了原因。
        - Dock action 确实带原因, 但部署链路 (web 控制台 → supervisor 的
          Trigger) 根本不接 action, 一个字都拿不到。

        成功也发: 上层订一个话题就拿到完整"结果"语义, 不必自己再维护一份
        终态名单 (supervisor 和前端现在各硬编码了一份)。
        seq 是轮次号: 锁存值分不出"这一轮的"还是"上一轮残留的", 靠它分。
        """
        ok = self._sm.is_success
        # 整轮耗时: start/start_undock 到终态沿 (重试不重置起点, 见
        # state_machine.start 的注释)。现场"这次停了多久 / 是不是卡了很久"
        # 只能靠它回答, 节点日志与 ~/outcome 各带一份。
        elapsed_sec = round(
            self._sm.elapsed_ns(self.get_clock().now().nanoseconds)*1e-9, 1)
        payload = {
            'seq': self._sm.run_seq,
            # 这一轮是停泊还是泊出 —— 同一个 motion_failed, 上层的处置不同。
            'op': 'undock' if state == DockingState.UNDOCKED
                  or self._prev_state == DockingState.UNDOCKING else 'dock',
            'state': state.name.lower(),
            'ok': ok,
            'code': '' if ok else self._sm.failure_code,
            'reason': '' if ok else self._sm.failure_reason,
            'elapsed_sec': elapsed_sec,
            'stamp': time.time(),
        }
        msg = String()
        # ensure_ascii=False: 理由串是中文, 转义了没人读得懂
        # (与 stack_supervisor._publish_status 同一套写法)。
        msg.data = json.dumps(payload, ensure_ascii=False)
        self._outcome_pub.publish(msg)
        if ok:
            self.get_logger().info(
                f'停泊结果: {payload["state"]} op={payload["op"]} '
                f'seq={payload["seq"]} 耗时={elapsed_sec:.1f}s')
        else:
            self.get_logger().error(
                f'停泊结果: {payload["state"]} code={payload["code"]} '
                f'seq={payload["seq"]} 耗时={elapsed_sec:.1f}s')

    def _publish_error(self, ex: float, ey: float, eyaw: float):
        msg = Vector3()
        msg.x = ex
        msg.y = ey
        msg.z = eyaw
        self._error_pub.publish(msg)

    # ── Service callbacks ──────────────────────────────────────────

    def _on_start_docking(self, request, response):
        ok = self._sm.start()
        response.success = ok
        response.message = f'state={self._sm.state_name}' if ok else 'already active'
        if not ok:
            return response
        self._executor.cancel()
        self._reset_maneuver()
        self._charge.reset()
        return response

    def _on_cancel_docking(self, request, response):
        self._sm.cancel()
        self._executor.cancel()
        self._reset_maneuver()
        self._adapter.publish_stop()
        response.success = True
        response.message = 'cancelled'
        return response

    def _on_start_undock(self, request, response):
        ok = self._sm.start_undock()
        response.success = ok
        response.message = f'state={self._sm.state_name}' if ok else 'docking active'
        if ok:
            self._executor.cancel()
            self._reset_maneuver()
            self._undock_phase = 0
            self._undock_note = ''
            self._undock_code = ''
        return response

    # ── Action callbacks ───────────────────────────────────────────

    def _execute_dock_cb(self, goal_handle):
        """Action execute callback — blocks until docking completes."""
        from tagdocking.action import Dock

        ok = self._sm.start()
        if not ok:
            goal_handle.abort()
            return Dock.Result(success=False, message='already active')

        self._executor.cancel()
        self._reset_maneuver()
        self._charge.reset()

        feedback = Dock.Feedback()

        while rclpy.ok() and self._sm.is_active:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self._sm.cancel()
                self._executor.cancel()
                self._reset_maneuver()
                self._adapter.publish_stop()
                return Dock.Result(success=False, message='cancelled')

            tag_pose = self._get_latest_pose()
            if tag_pose is not None:
                feedback.distance_error = abs(tag_pose.dist - self._dual.target)
                feedback.yaw_error = abs(tag_pose.yaw)
            feedback.state = self._sm.state_name.lower()
            goal_handle.publish_feedback(feedback)

            rclpy.spin_once(self, timeout_sec=0.05)

        if self._sm.is_success:
            goal_handle.succeed()
            return Dock.Result(success=True, message='docked')
        else:
            goal_handle.abort()
            # 保留 state_name 前缀 (可能有 startswith/in 的消费者), 只追加理由:
            # 光一个 'motion_failed' 让调用方对失败原因一无所知。
            # 码夹在中括号里跟在状态名后: 前缀匹配不受影响, 而 action 的调用方
            # 也能像订 ~/outcome 的上层那样按码分支, 不必去解中文串。
            reason = self._sm.failure_reason
            code = self._sm.failure_code
            state = self._sm.state_name.lower()
            tail = ' '.join(x for x in (f'[{code}]' if code else '', reason) if x)
            return Dock.Result(success=False,
                               message=f'{state}: {tail}' if tail else state)

    def _dock_cancel_cb(self, cancel_request):
        self._sm.cancel()
        self._executor.cancel()
        self._reset_maneuver()
        self._adapter.publish_stop()
        return CancelResponse.ACCEPT

    # ── Emergency stop ─────────────────────────────────────────────

    def _install_signal_handlers(self):
        stopping = {'flag': False}

        def _handle_signal(signum, frame):
            if stopping['flag']:
                return
            stopping['flag'] = True
            try:
                self.get_logger().info(f'紧急停止（信号 {signum}）')
            except Exception:
                pass
            for _ in range(100):
                try:
                    self._adapter.publish_stop()
                except Exception:
                    pass
                time.sleep(0.01)
            # 绝不在这里 rclpy.shutdown(): 处理器跑在主线程上, 此刻主 spin 和
            # TF 接收线程都还在转, 上下文在它们脚下被抽掉, rmw (zenoh) 的会话
            # 拆除就会与在途 wait_set/回调竞速 —— 实测 zenoh rx 线程
            # "Received Data for unknown expr_id" 刷屏, 直至 "terminate called
            # without an active exception" (SIGABRT)。这里只负责把零速度刷完;
            # 打断 spin 交给异常, 收尾顺序交给 main() 的 finally:
            # 停线程 → 销毁节点 → 最后才关上下文。
            raise KeyboardInterrupt

        signal.signal(signal.SIGINT, _handle_signal)
        signal.signal(signal.SIGTERM, _handle_signal)

    def _safe_stop(self):
        try:
            self._adapter.publish_stop()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = DockingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        # spin 里的真异常: 上下文已被有意关闭时属正常退出路径, 吞掉以免
        # launch 报 process died; 仅 rclpy 仍 ok 的才重新抛出。
        if rclpy.ok():
            raise
    finally:
        # 收尾顺序是这段代码的正确性所在 (信号处理器只刷零速度并抛
        # KeyboardInterrupt, 见 _handle_signal):
        #   1. 先停 TF 接收线程: shutdown 唤醒它的 spin, join 到真正退出;
        #   2. 再销毁节点: 此时已没有线程握着这些句柄;
        #   3. 最后才 shutdown 上下文: rmw (zenoh) 的会话拆除发生在所有本地
        #      wait_set 都停下之后。上下文在还有线程在 spin 时被关掉, 拆除会
        #      与在途回调竞速 —— zenoh rx "unknown expr_id" 刷屏直至 SIGABRT,
        #      实测即旧版在信号处理器里直接 shutdown 的死法。
        node._tf_executor.shutdown()
        node._tf_spin.join(timeout=2.0)
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            node._tf_node.destroy_node()
        except Exception:
            pass
        # 上下文此刻必然还没人关过 (处理器不再关), 正常路径 ok() 为真;
        # 退出竞态下可能已被关, 静默放行。
        if rclpy.ok():
            rclpy.shutdown()
