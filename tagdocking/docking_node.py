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
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from std_srvs.srv import Trigger
from apriltag_msgs.msg import AprilTagDetectionArray
import tf2_ros

from .utils import TagPose, yaw_from_quat, normalize_angle, tag_normal_angle
from .pose_buffer import PoseBuffer
from .geometry_planner import GeometryPlanner, ActionPlan
from .action_executor import ActionExecutor
from .state_machine import DockingStateMachine, DockingState
from .posture_mode import PostureMode
from .charge_mode import ChargeMode
from .dual_docking import DualTagDocking, DEFAULTS as DUAL_DEFAULTS
from .dual_camera import CameraModel
from .dual_feedback import ActionWatch
from .dual_posture import DualPosture


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
        self._tf_spin = threading.Thread(target=self._spin_tf_receiver, daemon=True)
        self._tf_spin.start()

        # ── Pose buffer ───────────────────────────────────────────
        self._pose_buffer = PoseBuffer(
            max_size=self._p('pose_buffer.size'),
            max_latency_ns=int(self._p('camera.max_latency_ms') * 1_000_000))

        # 双二维码对准管理器: 仅依赖参数, 必须先于规划器创建以提供目标距离。
        # dual.enable=false 时全程旁路, 行为同单码方案。
        self._dual = DualTagDocking(self)
        # 姿态切换 (搜索锁定→趴下匍匐, 桩码锁定→站立): 用户流程步骤 2/5。
        self._dual_posture = DualPosture(self)
        # 切站立宽限窗内豁免 locked 墙码 watchdog (姿态切换瞬间墙码短暂丢失)
        self._dual_stand_grace_ns = 0
        # FIFO, not a latest-only slot: delayed TF must get a chance to arrive.
        self._dual_pending = []
        self._dual_received_ns = 0
        self._dual_window_ns = 0
        self._dual_diag_ns = {}
        self._dual_info = {}
        self._dual_watch = None

        # ── Geometry planner (normal-line alignment) ──────────────
        self._planner = GeometryPlanner(
            target_distance=self._dual.effective_dock_distance(),
            lateral_threshold=self._p('stopgo.lateral_threshold'),
            yaw_threshold=math.radians(self._p('stopgo.yaw_threshold_deg')),
            tune_angle=self._p('stopgo.tune_angle'),
            jog_min=self._p('stopgo.jog_min'),
            jog_max=self._p('stopgo.jog_max'),
            position_tol=self._p('tolerance.position_m'),
            base_type=self._p('base.type'),
        )

        # ── Action executor (odometry dead-reckoning) ─────────────
        self._executor = ActionExecutor(
            turn_settle_sec=self._p('stopgo.turn_settle_sec'),
            turn_undershoot=self._p('stopgo.turn_undershoot'),
            max_turn_step=self._p('stopgo.max_turn_step'),
            small_turn_rad=self._p('stopgo.small_turn_rad'),
            final_approach_distance=self._p('final_servo.distance'),
            yaw_threshold=math.radians(self._p('stopgo.yaw_threshold_deg')),
            turn_lead_per_speed=self._p('stopgo.turn_lead_per_speed'),
            turn_slow_rad=self._p('stopgo.turn_slow_rad'),
        )

        # ── State machine ─────────────────────────────────────────
        self._sm = DockingStateMachine(self)
        self._sm._max_retries = int(self._p('retry.max_retries'))

        # ── 静止站立(锁定)管理器: 每个停看点的"停"升级为 ──────────────
        # 停→static_stand 锁定(不喘)→稳定帧→规划→stand_up 解锁→走。
        # 需要 self._p 与 self._sm, 必须在定时器启动前创建。
        self._posture = PostureMode(self)
        # 充电收尾 (DOCKED 后 静止→趴下→阻尼), 与 posture.enable 无关
        self._charge = ChargeMode(self)
        # 已规划待发的机动序列: 规划在锁定下完成后暂存, 由 _launch_pending_seq
        # 在恢复运动模式 (motion_enabled=True) 后原样启动, 解锁等待期间
        # 不重测/重规划。
        self._pending_seq: list | None = None
        # 停稳解冻后的连续 accepted 帧计数 (稳定帧门), _reset_visual_state 清零
        self._stable_frames = 0
        self._stable_window_start_ns = 0

        # ── Base adapter ──────────────────────────────────────────
        self._adapter = self._create_adapter()

        # ── Tag tracking state ────────────────────────────────────
        self._tag_frame = ''
        self._dock_tag_id = 0
        self._camera_frame = ''

        # Filtered pose
        self._filtered_dist: float | None = None
        self._filtered_lat: float | None = None
        self._filtered_yaw: float | None = None
        self._filtered_normal: float | None = None
        self._normal_sin = 0.0
        self._normal_cos = 0.0
        self._filter_init = False
        self._ema_alpha = 0.5
        self._max_pose_jump_m = 0.3
        self._jump_reject_count = 0
        self._max_jump_rejections = 10

        # Latest raw values
        self._raw_dist: float | None = None
        self._raw_lat: float | None = None
        self._raw_yaw: float | None = None
        self._raw_normal: float | None = None
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
        self._maneuver_iters = 0
        # 直行入口检查锁存：首次跨入直行距离的停看点判一次方位门槛，
        # 直行途中每停复测不再判（bearing=atan2(lat,dist) 随 dist 缩小自然
        # 变大, 复判会把入口合格的进入在中途误杀成后退重试）。dist 退出
        # 直行区（重试倒车/搜索后）重新武装。_reset_maneuver 时清零。
        self._straight_entered = False
        # 检测冻结标志：机动（盲转/盲走）期间为 True，此时 _on_detections 直接
        # 丢弃所有帧（运动模糊、视野边缘的坏帧绝不能污染规划用的位姿）。停稳
        # settle 结束后解冻，并清空滤波/缓冲，强制下一次规划只用停稳后的新鲜帧。
        self._frozen = False
        # Each iteration advances at most jog_max (stopgo.jog_max, runtime-tunable)
        # so covering a metre-plus approach plus refinement turns needs a
        # generous ceiling. This is only a runaway backstop — normal docking
        # converges (drive shrinks, no more clamping) well before it. APPROACH's
        # own timeout bounds wall-clock independently.
        self._max_maneuver_iters = 40

        # ── Undock (泊出) sub-phase ──────────────────────────────────
        # 0 = 盲退 undock.backup_distance, 1 = 原地转 180°, 2 = 完成。
        # 由 _run_undock 在 UNDOCKING 态驱动, 纯里程计闭环, 不看 tag。
        self._undock_phase = 0

        # Recovery-search state: remember which side the tag was last seen on
        # (sign of lat) so the angle-stepped sweep starts toward it. The node
        # rotates a fixed angle (odometry-closed), stops, detects, repeats —
        # sweeping a full 360° in one direction until the tag is found.
        self._last_seen_lat = 0.0
        self._search_step = 0
        self._search_detect_start = 0
        self._search_direction = 1.0

        # Adaptive detection-rate tracking. The pose-buffer staleness window is
        # derived from the measured inter-detection interval, so the controller
        # self-tunes to whatever rate the camera actually delivers (6 Hz or
        # 30 Hz). In stop-and-go 6 Hz is plenty; we just must not discard a
        # pose as "stale" faster than a new one can arrive.
        self._det_interval_ns: float | None = None   # EMA of gaps between detections
        self._latency_floor_ns = int(self._p('camera.max_latency_ms') * 1_000_000)
        self._latency_margin = self._p('camera.latency_interval_margin')

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
            f'停靠节点就绪 | 底盘={self._p("base.type")} | 走停模式')

    # ── Parameter helpers ──────────────────────────────────────────

    def _declare_params(self):
        """Declare all ROS2 parameters with defaults."""
        # Camera / timing
        self.declare_parameter('camera.max_latency_ms', 200)
        self.declare_parameter('camera.expected_fps', 30)
        # Adaptive staleness: window = measured detection interval × this margin,
        # clamped to be at least max_latency_ms. Tolerates a few dropped frames.
        self.declare_parameter('camera.latency_interval_margin', 3.0)
        # 相机安装横向偏移补偿: 相机光学中心装在底盘中心线左 lateral_offset_m 处
        # (+y, + = 相机偏左), base→camera 静态 TF 的 mount.y 未含此偏移 → 量测 lat
        # 系统性偏小该值 → 节点认为"正对"时底盘中心实际在 tag 法线右该值处, 停泊
        # 整体偏右、左腿撞桩。加回后 raw_lat 反映真实横向, 规划器据此左移修正。
        # 必须量测驱动每停重测, 不能一次性盲移(SEARCH 抖动重入会叠加)。
        self.declare_parameter('camera.lateral_offset_m', 0.0)

        # Tag
        self.declare_parameter('tag.family', '36h11')
        self.declare_parameter('tag.size', 0.16)
        self.declare_parameter('tag.frame', 'tag36h11:0')
        self.declare_parameter('tag.id', 0)
        self.declare_parameter('tag.fresh_timeout_sec', 1.0)
        self.declare_parameter('tag.tag_loss_timeout_sec', 2.5)
        self.declare_parameter('tag.ema_alpha', 0.5)
        self.declare_parameter('tag.max_pose_jump_m', 0.3)

        # TF
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('measure_frame', '')

        # Dock target
        self.declare_parameter('dock_target.distance', 0.30)
        self.declare_parameter('dock_target.lateral_offset', 0.0)
        self.declare_parameter('dock_target.yaw_offset_deg', 0.0)

        # Base
        self.declare_parameter('base.type', 'diff_drive')
        self.declare_parameter('base.cmd_vel_topic', 'cmd_vel')

        # Tolerance
        self.declare_parameter('tolerance.position_m', 0.03)
        self.declare_parameter('tolerance.yaw_deg', 3.0)
        self.declare_parameter('tolerance.stable_time_sec', 1.0)

        # Safety
        self.declare_parameter('safety.minimum_distance_m', 0.15)
        self.declare_parameter('timeout_sec', 120.0)

        # Retry (失败后倒车一段距离再重新 dock)
        self.declare_parameter('retry.max_retries', 2)
        self.declare_parameter('retry.backup_distance', 0.5)
        self.declare_parameter('retry.linear_rate', 0.08)
        self.declare_parameter('retry.timeout_sec', 15.0)

        # Undock (泊出: 盲退一段距离 → 原地转 180° → 完成)
        self.declare_parameter('undock.backup_distance', 0.5)
        self.declare_parameter('undock.linear_rate', 0.08)
        self.declare_parameter('undock.turn_angle_deg', 180.0)   # 正=CCW, 负=CW
        self.declare_parameter('undock.angular_rate', 0.3)
        self.declare_parameter('undock.timeout_sec', 30.0)

        # Search
        self.declare_parameter('search.angular_speed', 0.3)
        self.declare_parameter('search.step_angle_deg', 30.0)
        self.declare_parameter('search.rotate_time_sec', 0.8)  # deprecated, unused
        self.declare_parameter('search.pause_time_sec', 1.5)
        self.declare_parameter('search.initial_look_sec', 4.0)
        self.declare_parameter('search.hold_time_sec', 0.5)
        self.declare_parameter('search.search_direction', 1)
        self.declare_parameter('search.timeout_sec', 60.0)

        # Pose buffer
        self.declare_parameter('pose_buffer.size', 30)

        # State timeouts
        self.declare_parameter('align_timeout_sec', 15.0)
        self.declare_parameter('approach_timeout_sec', 60.0)
        self.declare_parameter('final_servo_timeout_sec', 30.0)

        # Final servo
        self.declare_parameter('final_servo.distance', 0.20)
        self.declare_parameter('final_servo.max_linear_speed', 0.05)
        self.declare_parameter('final_servo.max_yaw_speed', 0.2)
        # 到 dock_distance 即判定成功的方位门槛 (deg)。直行阶段不再追角度,
        # 到距离后方位偏 15° 以内都接受 —— 只要求到位就 DOCKED, 不再做
        # 1s 稳定确认/连续微调(呼吸摆动会让稳定凑不齐, 把已停好的泊位误判失败)。
        self.declare_parameter('final_servo.yaw_tol_deg', 15.0)

        # Final straight (两阶段停泊: 85cm 对准 → 55cm 纯直行)
        self.declare_parameter('final_straight.enable', True)
        self.declare_parameter('final_straight.start_distance', 0.85)
        self.declare_parameter('final_straight.yaw_threshold_deg', 3.0)
        # 近场法线(normal)对准门槛 (deg): 收紧到 ~2° 让"先对齐法线再横移"的次序
        # 成立 —— 之前法线对准用 stopgo.yaw_threshold_deg(10°) 太松, 2~5° 航向
        # 误差被当作已对齐, 带着误差横移导致反复/反向横移。
        self.declare_parameter('final_straight.normal_yaw_threshold_deg', 2.0)
        # 法线转向触发的噪声下限 (deg): 实际转向门槛 = max(上面, 本值)。
        # normal 有平面 PnP 二义性 (mirror 解, 近场 ±10° 双峰: 实测解卷绕后在
        # ~170°/~190° 两簇间跳, 车体已转 15° 读数几乎不变), 3帧 EMA 后残差
        # 仍 ±3-5°。门槛低于噪声下限时转向决策本身抖动 → 原地摆头追噪声、
        # 横移永远触发不了 (2026-09 对接日志)。残差航向误差不查 normal ——
        # 由直行入口包络 (bearing+横向) 兜底, 系统容忍 final_servo 15°。
        self.declare_parameter('final_straight.normal_turn_min_deg', 6.0)
        self.declare_parameter('final_straight.entry_lateral_m', 0.03)
        # 近场/远场分界: dist ≤ 此值时阶段1 走「法线对准 + 横移」微调;
        # 远场走「纯方位粗对准 + 前进」(不横移), 避免 1.5m 处 normal 噪声
        # 放大成反复/反向横移。用户直观认知 ~1.3m。
        self.declare_parameter('final_straight.tighten_distance', 1.3)
        # 远场粗对准门槛 (dist > tighten_distance 且 two_phase 启用):
        # 比停走宽松 —— 远场只做大尺度朝向修正, 微调留给近场。方位用 bearing
        # (atan2(lat,dist)), 不受 normal 噪声影响。
        self.declare_parameter('final_straight.far_yaw_threshold_deg', 15.0)
        self.declare_parameter('final_straight.far_lateral_m', 0.20)
        # 近场横移修正 / 捷径的横向门槛 (比入口 entry_lateral_m 更紧): 量测补偿
        # 加回 3cm 偏置后, 近场横移修正需在直行前把真实横向压到此值内, 否则左腿
        # 仍会撞桩。0.02 < 阶段1 stopgo.lateral_threshold(0.05), 保证规划器会滑。
        self.declare_parameter('final_straight.lateral_threshold_m', 0.02)

        # ── Dual-tag docking (双二维码对准, 默认关闭) ────────────────
        # enable=false 时全程旁路: 节点行为与单码方案完全一致。
        for name, default in DUAL_DEFAULTS.items():
            self.declare_parameter('dual.' + name, default)
        self.declare_parameter('dual.enable', False)
        self.declare_parameter('dual.camera_info_topic', '/camera_sync/camera_info')
        self.declare_parameter('dual.projection_mode', 'raw')
        # 墙码边长 (36h11:0, apriltag 节点按此解 PnP; launch 侧同步透传)
        self.declare_parameter('dual.wall_tag_size', 0.15)
        # 桩码 ID/边长 (ID=51 现场已确认; 换桩改配置或 launch pile_tag_id:=)
        self.declare_parameter('dual.pile_tag_id', 51)
        self.declare_parameter('dual.pile_tag_size', 0.05)
        # 相机系目标: 距墙码 1.0m 处完成双码对准 → 纯直行
        self.declare_parameter('dual.straight_start_distance', 1.0)
        # 相机系停泊距离: 摄像头距墙码 0.50m = 停泊完成
        self.declare_parameter('dual.dock_distance', 0.50)
        # 对准判据: 两码方位 (base_link 系 bearing) 同时 ≤ ±tol 保持 hold
        self.declare_parameter('dual.align_tolerance_deg', 3.0)
        self.declare_parameter('dual.align_hold_sec', 0.5)
        # 桩码量测新鲜窗口 (超时视为桩码不可见 → 墙码单码修正)
        self.declare_parameter('dual.pile_fresh_timeout_sec', 2.0)

        # Stop-and-go params
        self.declare_parameter('stopgo.lateral_threshold', 0.04)
        self.declare_parameter('stopgo.yaw_threshold_deg', 3.0)
        self.declare_parameter('stopgo.tune_angle', 0.0)
        self.declare_parameter('stopgo.jog_min', 0.05)
        self.declare_parameter('stopgo.jog_max', 0.50)
        self.declare_parameter('stopgo.jog_linear_rate', 0.08)
        self.declare_parameter('stopgo.jog_angular_rate', 0.3)
        self.declare_parameter('stopgo.turn_creep_linear', 0.0)  # 已弃用，固定纯原地转
        self.declare_parameter('stopgo.lateral_rate', 0.08)
        # 狗固件横移通道航位推算严重低估（实测 odom 0.507m / 实际约 2m，
        # 低估 ~4 倍）：判停目标 = 距离/该系数，即真实位移达到目标时停。
        self.declare_parameter('stopgo.lateral_odom_scale', 1.0)
        # 前进/后退通道同样低估（后退实测 odom 0.493m / 实际 1m+，~2 倍；
        # 前进待精标）。只有转向（IMU yaw）可信。
        self.declare_parameter('stopgo.jog_odom_scale', 1.0)
        self.declare_parameter('stopgo.jog_backward_odom_scale', 1.0)
        self.declare_parameter('stopgo.turn_settle_sec', 0.5)
        self.declare_parameter('stopgo.turn_undershoot', 0.75)
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
        self.declare_parameter('stopgo.theta_shrink_ratio', 2.0)
        self.declare_parameter('stopgo.drift_tol', 0.15)

        # ── 静止站立 (posture) — 走停 × 呼吸抑制 ──────────────────────
        # 每个停看点的"停"升级为: static_stand 锁定(不喘) → 等稳定帧 →
        # 规划 → stand_up 解锁 → 走。接口在 zsibot_l1_control (l1w_control);
        # 无桥 (备用桥/纯台架) 时 service_wait_sec 宽限后自动降级停用。
        self.declare_parameter('base.l1w_prefix', '/l1w_control')
        self.declare_parameter('posture.enable', False)
        # 停→量测最短间隔: 覆盖 RTSP 延迟 (0.1~0.5s) + CMD_LOCK_MODE 过渡 +
        # 呼吸衰减。与 stopgo.turn_settle_sec 同一起点, 实际取两者较大值。
        self.declare_parameter('posture.static_settle_sec', 1.2)
        self.declare_parameter('posture.lock_ack_timeout_sec', 2.0)
        self.declare_parameter('posture.unlock_ack_timeout_sec', 2.0)
        self.declare_parameter('posture.unlock_retries', 2)
        self.declare_parameter('posture.min_stable_frames', 3)
        self.declare_parameter('posture.stable_frame_timeout_sec', 2.5)
        self.declare_parameter('posture.service_wait_sec', 1.0)

        # 充电收尾 (DOCKED 后): 静止→匍匐趴下→阻尼泄力。与 posture.enable
        # 无关 —— 关掉中途锁定/解锁, 停泊完成后仍执行整个序列。
        self.declare_parameter('charge.enable', True)
        self.declare_parameter('charge.passive', True)  # 阻尼步; false=仅锁定(站立)
        self.declare_parameter('charge.static_stand', True)  # 先锁定再阻尼; false=跳过锁定直接阻尼
        self.declare_parameter('charge.static_ack_timeout_sec', 3.0)
        self.declare_parameter('charge.lie_down_settle_sec', 4.0)
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

        if self._dual.enabled:
            self._dual_info_sub = self.create_subscription(CameraInfo,
                self._p('dual.camera_info_topic'), self._on_dual_camera_info, qos_profile_sensor_data)
        odom_topic = self._p('odom_topic')
        self._odom_sub = self.create_subscription(
            Odometry, odom_topic, self._on_odom, 10)

        self._state_pub = self.create_publisher(String, '~/state', 10)
        self._error_pub = self.create_publisher(Vector3, '~/error', 10)

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
        """Create the appropriate BaseAdapter based on base.type parameter."""
        base_type = self._p('base.type')
        cmd_vel_topic = self._p('base.cmd_vel_topic')

        if base_type == 'omni':
            from .base_adapter import OmniAdapter
            return OmniAdapter(self, cmd_vel_topic=cmd_vel_topic)
        elif base_type == 'quadruped':
            from .base_adapter import QuadrupedAdapter
            return QuadrupedAdapter(node=self)
        else:  # diff_drive
            from .base_adapter import DiffDriveAdapter
            return DiffDriveAdapter(self, cmd_vel_topic=cmd_vel_topic)

    # ── Callbacks ──────────────────────────────────────────────────

    def _on_detections(self, msg: AprilTagDetectionArray):
        """Store detection timestamps; actual TF query happens in control loop."""
        # 计数无条件递增 (冻结/终态早退之前): 停留日志用它区分"检测流断了"
        # 和"tag 不在视野" —— 前者计数不涨, 后者只有有效检测归零。
        self._det_msg_count += 1
        # 机动期间冻结检测：盲转/盲走过程中相机帧运动模糊、二维码常在视野边缘，
        # 这些坏帧一律丢弃，绝不更新 _filtered_*、_pose_buffer 或 _last_detection_ns。
        # 规划器因此只会读到小车停稳后新采的帧。
        if self._dual.enabled:
            self._on_dual_detections(msg)
            return
        if self._frozen:
            return

        state = self._sm.state
        if state in (DockingState.DOCKED, DockingState.TAG_LOST,
                     DockingState.TIMEOUT, DockingState.MOTION_FAILED,
                     DockingState.CANCELLED):
            return

        dock_id = int(self._p('tag.id'))
        tag_frame = self._p('tag.frame')

        for det in msg.detections:
            if det.id == dock_id:
                self._tag_frame = tag_frame
                self._dock_tag_id = dock_id
                self._lookup_tag_pose()
                break

    def _on_odom(self, msg: Odometry):
        self._odom_x = msg.pose.pose.position.x
        self._odom_y = msg.pose.pose.position.y
        self._odom_yaw = yaw_from_quat(msg.pose.pose.orientation)
        self._has_odom = True
        self._odom_stamp_ns = msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec

    # ── TF tag pose lookup ─────────────────────────────────────────

    def _lookup_tag_pose(self):
        """Query TF for tag pose in base_link frame.

        Converts from camera optical frame (z-forward, x-right) to
        base_link convention (x-forward, y-left, REP-103).

        Results are EMA-filtered and stored in self._raw_* and self._filtered_*.
        """
        base_frame = self._p('base_frame')
        measure_frame = self._p('measure_frame')

        src_frame = measure_frame if measure_frame else base_frame

        if not self._tag_frame:
            return False

        try:
            t = self._tf_buffer.lookup_transform(
                src_frame, self._tag_frame,
                rclpy.time.Time(seconds=0),
                rclpy.duration.Duration(seconds=0))
            # timeout=0: 非阻塞查最新值。TF 接收在专用线程持续供数, buffer
            # 里已有即秒回; 控制循环回调里绝不能忙等 (会把执行器占死)。
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            self._tf_fail_count += 1
            return False

        self._tf_fail_count = 0

        # TF 查询 src_frame→tag 的结果已经是 src_frame 坐标系的表示。
        # base_link 遵循 REP-103: x=前, y=左, z=上。
        # 直接取 x 为距离, y 为横向，不需要做光学坐标系转换。
        raw_dist = t.transform.translation.x
        raw_lat = t.transform.translation.y
        # 相机安装横向偏移补偿: camera.lateral_offset_m = +0.03 表示相机偏底盘
        # 中心线左 3cm (base_link 的 +y), 静态 TF mount.y 未含此偏移 → raw_lat
        # 系统性偏小 0.03m → 加回后 lat 反映底盘中心到 tag 法线的真实横向, 规划器
        # 据此在直行前触发左移修正 (而非盲移 3cm, 盲移在 SEARCH 抖动重入会叠加)。
        raw_lat += float(self._p('camera.lateral_offset_m'))

        # Tag yaw = 方位角 (bearing): 机器人需要转多少弧度才能正对 Tag。
        # atan2(lat, dist) 是 tag 在机器人坐标系中的方向角，正=左边。
        tag_yaw_raw = math.atan2(raw_lat, raw_dist) if raw_dist > 0.001 else 0.0

        # Tag OUTWARD-NORMAL direction (rad, base_link ground plane). Needed for
        # the turn-drive-turn maneuver: the robot must reach the tag's normal
        # line and face the tag squarely, which requires the tag's orientation,
        # not just the bearing. Self-corrects the solvePnP flip ambiguity.
        tag_normal_raw = tag_normal_angle(t.transform.rotation, raw_dist, raw_lat)

        # Jump rejection
        if self._filter_init:
            jump_d = abs(raw_dist - self._filtered_dist) > self._max_pose_jump_m
            jump_l = abs(raw_lat - self._filtered_lat) > self._max_pose_jump_m
            if jump_d or jump_l:
                self._jump_reject_count += 1
                if self._jump_reject_count >= self._max_jump_rejections:
                    self._filtered_dist = raw_dist
                    self._filtered_lat = raw_lat
                    self._jump_reject_count = 0
                self._stable_frames = 0   # 跳变帧打断"连续 accepted"计数
                return False
            else:
                self._jump_reject_count = 0

        # EMA filtering
        alpha = self._ema_alpha
        if self._filter_init:
            self._filtered_dist = alpha * raw_dist + (1.0 - alpha) * self._filtered_dist
            self._filtered_lat = alpha * raw_lat + (1.0 - alpha) * self._filtered_lat
            self._filtered_yaw = alpha * tag_yaw_raw + (1.0 - alpha) * self._filtered_yaw
            # Circular EMA for the normal (wraps at ±pi): filter the sin/cos.
            self._normal_sin = alpha * math.sin(tag_normal_raw) + (1.0 - alpha) * self._normal_sin
            self._normal_cos = alpha * math.cos(tag_normal_raw) + (1.0 - alpha) * self._normal_cos
            self._filtered_normal = math.atan2(self._normal_sin, self._normal_cos)
        else:
            self._filtered_dist = raw_dist
            self._filtered_lat = raw_lat
            self._filtered_yaw = tag_yaw_raw
            self._normal_sin = math.sin(tag_normal_raw)
            self._normal_cos = math.cos(tag_normal_raw)
            self._filtered_normal = tag_normal_raw
            self._filter_init = True

        self._raw_dist = raw_dist
        self._raw_lat = raw_lat
        self._raw_yaw = tag_yaw_raw
        self._raw_normal = tag_normal_raw

        now_ns = self.get_clock().now().nanoseconds
        # Track the inter-detection interval and adapt the staleness window.
        if self._last_detection_ns != 0:
            gap = now_ns - self._last_detection_ns
            # Ignore huge gaps (tag was out of view): they are not the frame
            # rate, only genuine consecutive detections estimate the cadence.
            if 0 < gap < 2_000_000_000:  # < 2s
                if self._det_interval_ns is None:
                    self._det_interval_ns = float(gap)
                else:
                    self._det_interval_ns = 0.3 * gap + 0.7 * self._det_interval_ns
                # Window = a few detection intervals, never below the floor,
                # so one dropped frame at low rate does not orphan the buffer.
                adaptive = self._det_interval_ns * self._latency_margin
                self._pose_buffer.set_max_latency_ns(
                    max(self._latency_floor_ns, int(adaptive)))
        self._last_detection_ns = now_ns

        stamp = self._last_detection_ns
        pose = TagPose(dist=self._filtered_dist, lat=self._filtered_lat,
                       yaw=self._filtered_yaw, normal=self._filtered_normal,
                       stamp_ns=stamp)
        self._pose_buffer.add(pose)
        # Remember the side the tag was last seen on, to bias recovery search.
        if abs(self._filtered_lat) > 1e-3:
            self._last_seen_lat = self._filtered_lat
        self._stable_frames += 1   # accepted 帧: 稳定帧门计数
        return True

    def _spin_tf_receiver(self):
        """TF 接收线程: 退出/关闭时的异常就地吞掉。

        必须用专用执行器: Humble 的 rclpy.spin() 不传 executor 时用的是
        全局单例, 与主线程共用会报 "generator already executing"。
        """
        try:
            executor = SingleThreadedExecutor()
            executor.add_node(self._tf_node)
            executor.spin()
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

    def _dual_diagnostic(self, reason, stamp, now):
        """Per-reason throttle on the docking logger, never the video logger."""
        previous = self._dual_diag_ns.get(reason)
        if previous is None or now < previous or now - previous >= 2_000_000_000:
            self._dual_diag_ns[reason] = now
            self.get_logger().info(
                f'dual detection {reason}: stamp={stamp} '
                f'age_ms={(now-stamp)/1e6:.1f} pending={len(self._dual_pending)}')

    def _discard_dual_pending(self):
        """Fence task / freeze / visual windows by ORIGINAL sensor time."""
        if self._dual.enabled:
            self._dual_pending.clear()
            self._dual_window_ns = self.get_clock().now().nanoseconds
            self._dual_received_ns = max(self._dual_received_ns, self._dual_window_ns)

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
        if wall_id in ids and pile_id in ids:
            self._dual_log('found',
                f'双码发现: wall_id={wall_id} pile_id={pile_id} stamp={stamp} '
                f'age_ms={(now-stamp)/1e6:.1f} stage={self._dual.stage} '
                f'frozen={self._frozen} outer={self._sm.state.name} (仅检测，尚未通过TF/观测检查)', now)
        if self._sm.state not in (DockingState.SEARCH_TAG, DockingState.ALIGN,
                                  DockingState.APPROACH, DockingState.FINAL_SERVO):
            self._discard_dual_pending()
            return
        if self._frozen and self._dual.stage != 'locked':
            self._discard_dual_pending()
            self._dual_diagnostic('rejected frozen', stamp, now)
            return
        if (stamp <= self._dual_received_ns or stamp < self._dual_window_ns
                or stamp < self._dual.settle_until_ns):
            self._dual_diagnostic('rejected duplicate/old/window', stamp, now)
            return
        # Future stamps must not poison the monotonic watermark.
        if not self._dual.fresh(now, stamp):
            if 0 < stamp <= now:
                self._dual_received_ns = stamp
                self._invalidate_dual_pose()
            self._dual_diagnostic('expired/future', stamp, now)
            return
        self._dual_received_ns = stamp
        if wall_id not in ids:
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
                self._sm.abort_motion('locked final: wall detection lost')
            return
        # Drop NEW arrivals when full, never evict a waiting head for new frames.
        if len(self._dual_pending) >= 64:
            self._dual_diagnostic('rejected queue full', stamp, now)
            return
        self._dual_pending.append((stamp, self._dual.pile_tag_id in ids))
        self._retry_dual_detections(now)

    def _retry_dual_detections(self, now):
        """FIFO head retries are nonblocking; each detection succeeds at most once."""
        if self._sm.state not in (DockingState.SEARCH_TAG, DockingState.ALIGN,
                                  DockingState.APPROACH, DockingState.FINAL_SERVO):
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
                self._raw_dist = wall[2]
                self._raw_lat = -wall[0]
                self._last_detection_ns = stamp
                self._pose_buffer.add(TagPose(dist=wall[2], lat=-wall[0],
                    yaw=-math.atan2(wall[0], wall[2]), normal=0.0, stamp_ns=stamp))
            else:
                self._dual_diagnostic('rejected observation', stamp, now)

    def _tag_fresh(self) -> bool:
        """Check if tag detection is within freshness window."""
        if self._dual.enabled:
            return self._dual.fresh(self.get_clock().now().nanoseconds,
                                    self._last_detection_ns)
        if self._last_detection_ns == 0:
            return False
        timeout_ns = int(self._p('tag.fresh_timeout_sec') * 1e9)
        now_ns = self.get_clock().now().nanoseconds
        return (now_ns - self._last_detection_ns) < timeout_ns

    def _get_latest_pose(self):
        """Get latest valid pose from buffer (with latency check)."""
        now_ns = self.get_clock().now().nanoseconds
        if self._dual.enabled:
            if not self._dual.fresh(now_ns, self._last_detection_ns):
                return None
            self._pose_buffer.set_max_latency_ns(int(self._dual.p('fresh_sec') * 1e9))
        return self._pose_buffer.get_latest(now_ns)

    # ── Control loop (20 Hz) ───────────────────────────────────────

    def _control_loop(self):
        """Main 20 Hz control loop — stop-and-go paradigm."""
        now_ns = self.get_clock().now().nanoseconds

        if self._dual.enabled:
            self._retry_dual_detections(now_ns)

        # Gather inputs
        tag_visible = self._tag_fresh()
        tag_pose = self._get_latest_pose()
        base_type = self._p('base.type')

        # Compute errors (for publishing and state machine)
        error_x, error_y, error_yaw = 0.0, 0.0, 0.0
        if tag_pose is not None:
            error_x = tag_pose.dist - self._dual.effective_dock_distance()
            error_y = tag_pose.lat - self._p('dock_target.lateral_offset')
            error_yaw = normalize_angle(
                tag_pose.yaw - math.radians(self._p('dock_target.yaw_offset_deg')))

        # Final heading lock never searches, reverses or continues on a silent
        # camera. Frozen observations update only this independent wall watchdog.
        if (self._dual.enabled and self._dual.stage == 'locked'
                and self._executor.is_active
                and now_ns > self._dual_stand_grace_ns
                and not self._dual.fresh(now_ns, getattr(self, '_dual_live_wall_ns', self._dual.stamp))):
            self._adapter.publish_stop()
            self._executor.cancel()
            self._sm.abort_motion('locked final: wall stream stale')
        if self._dual.enabled and self._sm.state == DockingState.SEARCH_TAG:
            self._lookup_camera_offset()
            if self._dual.failure:
                self._adapter.publish_stop()
                self._executor.cancel()
                self._sm.abort_motion(self._dual.failure)

        # State machine evaluation
        params = self._build_params_dict()
        params['dual_enable'] = self._dual.enabled
        self._sm.evaluate(
            tag_pose=tag_pose,
            tag_visible=tag_visible,
            odom_x=self._odom_x,
            odom_y=self._odom_y,
            odom_yaw=self._odom_yaw,
            cmd_vx=0.0, cmd_vy=0.0, cmd_wz=0.0,  # not used in stop-and-go
            motion_stalled=False,  # handled by action_executor
            now_ns=now_ns,
            params=params,
            maneuver_active=self.maneuver_active,
        )

        state = self._sm.state

        # 机动中被外部切入静止站立 (如网页台"静止站立"按钮): cmd_vel 已被桥
        # 拒绝、里程计不会再走, 立即中止而不是静默耗完接近超时。
        # (external_lock 要求 motion_enabled==False, 解锁刚完成时 posture_state
        # 的滞后残留不会误判。)
        if self._executor.is_active and self._posture.external_lock:
            self.get_logger().error('机动期间被外部切入静止站立 → MOTION_FAILED')
            self._sm.abort_motion('外部锁定打断机动')
            state = self._sm.state

        # On leaving the stop-and-go states (e.g. APPROACH→SEARCH_TAG re-lock,
        # or any error/terminal transition), abort any half-finished action so
        # it cannot resume later against a stale odometry reference.
        stopgo_states = (DockingState.ALIGN, DockingState.APPROACH,
                         DockingState.FINAL_SERVO)
        if (self._prev_state in stopgo_states and state not in stopgo_states):
            self._executor.cancel()
            self._planner.reset()
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
        # 终态恢复运动模式 (DOCKED 除外): 释放 cmd_vel 给遥控/网页台。
        # DOCKED 按约定保持静止站立 (泊出时由 _run_undock 先解锁);
        # UNDOCKED 时狗本就在运动模式。
        if (self._prev_state != state and state in (
                DockingState.TAG_LOST, DockingState.TIMEOUT,
                DockingState.MOTION_FAILED, DockingState.CANCELLED)):
            self._posture.release(now_ns, reason=f'进入终态 {state.name}')
        # 入 DOCKED → 启动充电收尾 (静止→趴下→阻尼)。与 posture.enable 无关:
        # 呼吸抑制只管停-看循环的量测稳定, 泊完把狗放到桩上必须做。
        if (self._prev_state != state and state == DockingState.DOCKED):
            self._adapter.publish_stop()
            self._executor.cancel()
            self._charge.begin(now_ns)
        self._prev_state = state

        # ── Per-state behaviour ───────────────────────────────────
        # Active motion states own /cmd_vel and command motion each tick.
        # Quiescent states (IDLE/DOCKED/errors) must NOT keep publishing zero —
        # that monopolises /cmd_vel and locks out teleop. Brake briefly on
        # entry, then release the topic so other publishers can drive the robot.
        if state == DockingState.SEARCH_TAG:
            self._quiescent = False
            self._run_search(tag_visible, tag_pose, base_type, now_ns)

        elif state in (DockingState.ALIGN, DockingState.APPROACH,
                       DockingState.FINAL_SERVO):
            self._quiescent = False
            self._run_stop_and_go(tag_visible, tag_pose, base_type, now_ns)

        elif state == DockingState.RETRYING:
            self._quiescent = False
            self._run_retry(base_type, now_ns)

        elif state == DockingState.UNDOCKING:
            self._quiescent = False
            self._run_undock(base_type, now_ns)

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

    def _run_stop_and_go(self, tag_visible: bool, tag_pose, base_type: str,
                         now_ns: int):
        """One tick of the turn-drive-turn stop-and-go loop.

        Three nested cases:
          1. An executor action is running  → update its odometry; when it
             finishes, immediately start the NEXT queued sub-step (no settle,
             no re-measure — the maneuver is blind).
          2. The maneuver queue just emptied → settle, then re-measure + re-plan.
          3. Idle & settled                 → measure the tag and plan a fresh
             turn-drive-turn sequence (or declare done).
        """
        target_distance = self._dual.effective_dock_distance()

        # ── Case 1: a sub-step is executing ───────────────────────────
        if self._executor.is_active:
            if self._dual.enabled and not self._check_dual_action(now_ns):
                return
            done = self._executor.update(
                self._odom_x, self._odom_y, self._odom_yaw,
                tag_visible and not self._dual.enabled, self._raw_dist,
                self._bearing_fn,
                self._theta_bounds_fn,
                target_distance,
                self._p('stopgo.drift_tol'),
                now_ns,
            )
            if done:
                if self._dual.enabled:
                    self.get_logger().info(f'dual action COMPLETE signed_odom={self._dual_watch.signed:+.6f}; awaiting settled visual feedback')
                    self._dual.action_completed()
                    self._dual_watch = None
                if self._maneuver_queue:
                    # Chain straight into the next sub-step by odometry — no
                    # settle, tag not consulted. This is the whole point: the
                    # maneuver runs open-loop on odometry so a narrow FOV losing
                    # the tag mid-turn cannot derail it.
                    self._start_next_maneuver_step(base_type)
                else:
                    # Whole sequence finished → settle before re-measuring.
                    self._maneuver_active = False
                    self._mark_stopped(now_ns)
            self._publish_action_cmd(base_type)
            return

        if self._dual.enabled and now_ns < self._dual.settle_until_ns:
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

        # ── Case 2.4: 静止站立锁定 + 停振窗口 (呼吸抑制) ───────────────
        # 停稳边界 (_mark_stopped) 已请求 static_stand; 这里等 posture_state
        # 变为 static_stand 且距停稳 >= posture.static_settle_sec (覆盖 RTSP
        # 延迟 + CMD_LOCK_MODE 过渡), 之后才解冻量测 —— 量测窗内狗完全静止,
        # tag 位姿不再被步态呼吸晃动。降级/停用时本门直接放行。
        if not self._posture.lock_settled(now_ns):
            self._adapter.publish_stop()
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
        if not self._launch_pending_seq(base_type, now_ns):
            return

        if self._dual.enabled:
            self._adapter.publish_stop()
            if not self._has_odom:
                self._sm.abort_motion('dual docking requires odometry')
                return
            self._lookup_camera_offset()
            # 步骤 2: 搜索锁定墙码后先趴下 —— 桩码贴桩底座更矮, 站立视角看不到,
            # 双码对准/接近全程匍匐。切换期间停车等待 (响应+settle 非阻塞轮询)。
            if not self._dual_posture.ensure_crouch(now_ns):
                if self._dual_posture.failure:
                    self._executor.cancel()
                    self._sm.abort_motion('dual 姿态切换: ' + self._dual_posture.failure)
                return
            seq = self._dual.plan_dual(tag_pose, base_type, self._planner, now_ns)
            if self._dual.failure:
                self._executor.cancel()
                self._sm.abort_motion(self._dual.failure)
            elif self._dual.complete:
                self._executor.cancel()
                self._pending_seq = None
                self._adapter.publish_stop()
                self._sm.finish_dual()
            elif not seq and self._dual.request_stand:
                # 步骤 5: 桩码 z ≤ 锁定距离且对准成立 → 切站立 (匍匐进不了桩底座)。
                # 站立后桩码必然丢失, locked 只按墙码纯直行; watchdog 给宽限窗
                # (姿态切换瞬间墙码也会短暂丢失)。确认后重置观测窗重新采样。
                if self._dual_posture.ensure_stand(now_ns):
                    self._dual.request_stand = False
                    self._dual.stopped(now_ns)
                    self._discard_dual_pending()
                    self._dual_stand_grace_ns = now_ns + int(5.0e9)
                elif self._dual_posture.failure:
                    self._executor.cancel()
                    self._sm.abort_motion('dual 姿态切换: ' + self._dual_posture.failure)
                return
            elif seq:
                self._pending_seq = list(seq)
                self._launch_pending_seq(base_type, now_ns)
            return

        # ── Case 3: idle & settled — measure and plan a fresh sequence ─
        if not tag_visible or tag_pose is None:
            self._adapter.publish_stop()
            self.get_logger().info(
                f'走停：空闲停车（二维码可见={tag_visible}, '
                f'位姿={"无" if tag_pose is None else "有"}）',
                throttle_duration_sec=1.0)
            return

        # ── 稳定帧门: 规划只认停稳解冻后连续 accepted 的新鲜帧 ──────────
        # EMA 已被解冻重新播种, 再要求 N 帧一致只多花 ~0.2-0.5s (6-10fps),
        # 却能把单帧噪声挡在规划之外。等不满时超时放行 (防闪烁卡死),
        # 设 1 即关闭。与 posture.enable 无关 —— 感知侧去噪永远值得。
        min_frames = int(self._p('posture.min_stable_frames'))
        if min_frames > 1 and self._stable_frames < min_frames:
            if self._stable_window_start_ns == 0:
                self._stable_window_start_ns = now_ns
            if (now_ns - self._stable_window_start_ns) * 1e-9 < float(
                    self._p('posture.stable_frame_timeout_sec')):
                self._adapter.publish_stop()
                return
            self.get_logger().warn(
                '稳定帧数量不足, 以当前位姿继续规划', throttle_duration_sec=5.0)

        if self._maneuver_iters >= self._max_maneuver_iters:
            self.get_logger().warn(
                f'走停：已达最大机动迭代次数（{self._max_maneuver_iters}），'
                '停止；最后位姿已在可达范围内')
            self._adapter.publish_stop()
            return

        yaw_tol = math.radians(self._p('tolerance.yaw_deg'))

        # ── 两阶段停泊门控 ──────────────────────────────────────
        # 阶段1(带角度修正)为默认：用 plan_sequence 转向对准+前进, 尽量对准。
        # 阶段2(纯直行, 不修 yaw/横向)：dist ≤ start_distance 即无条件激活——
        # 进入直行距离后像停车入库, 不再打方向(再往前已无空间调位姿)。
        # 入口检查只在首次跨入直行距离的停看点判一次（入口包络）：方位误差超
        # yaw_threshold_deg 或 |横向| 超 entry_lateral_m 都算对准过差、入库会
        # 撞偏 → fail() 重试 —— 直行阶段不修横向, 入口横向误差会一路带到终点。
        # 直行途中每停复测不判 —— bearing=atan2(lat,dist), lat 固定时随
        # dist 缩小自然变大, 复判会把入口合格的进入在中途误杀成后退重试,
        # 与"直行阶段不再调角、不后退重试"的语义矛盾。dist 退出直行区
        # (重试倒车/搜索后) 重新武装, 下一次接近仍有入口门槛。
        # start_distance ≤ target_distance 视为误配置, 静默回退单阶段。
        straight_enabled = self._p('final_straight.enable')
        straight_start = self._p('final_straight.start_distance')
        straight_yaw_tol = math.radians(self._p('final_straight.yaw_threshold_deg'))
        entry_lat_tol = float(self._p('final_straight.entry_lateral_m'))
        # 近场横移修正 / 捷径门槛 (比入口 entry_lat_tol 更紧, 默认 0.02): 量测
        # 补偿加回相机 3cm 偏置后, 入口 0.03 的横向容差会让真实 0.03m 偏移被捷径
        # 放行直行、左腿撞桩 —— 近场修正与捷径改用此更紧值, 入口检查仍用宽值。
        lat_threshold_m = float(self._p('final_straight.lateral_threshold_m'))
        two_phase = straight_enabled and straight_start > target_distance
        # 近场收紧边界: dist ≤ 此值时阶段1 修正门槛收紧到入口包络。误配
        # tighten < start 时钳到 start —— 收紧一直生效到直行区边界, 不留缝。
        tighten_dist = max(float(self._p('final_straight.tighten_distance')),
                           straight_start)

        # 方位误差用车体朝向(bearing=atan2(lat,dist))，不用方阵误差(square_err)。
        # square_err 依赖 tag 法线(normal)，而 normal 是 AprilTag 最不可靠的自由度
        # ——近场时在 ±180° 附近抖动，经 ±π 归一化后误差被放大到 4~5°，即使车已
        # 正对标签(方位角<1°)也会误判超差。bearing 只取决于标签在画面中的位置，稳定可靠。
        bearing = math.atan2(tag_pose.lat, tag_pose.dist)
        bearing_err = abs(normalize_angle(bearing))

        go_straight = False
        if two_phase:
            if tag_pose.dist <= straight_start:
                # 进入直行距离 → 无条件直行（不再调角）。
                # 入口检查只判一次（首次跨入的停看点, _straight_entered 锁存）。
                if not self._straight_entered:
                    self._straight_entered = True
                    if (bearing_err > straight_yaw_tol
                            or abs(tag_pose.lat) > entry_lat_tol):
                        self.get_logger().error(
                            f'直行失败：进入直行距离({tag_pose.dist:.2f}m)时误差 '
                            f'方位={math.degrees(bearing_err):.1f}° '
                            f'(门槛{math.degrees(straight_yaw_tol):.1f}°) '
                            f'横向={tag_pose.lat:+.3f}m '
                            f'(门槛±{entry_lat_tol:.3f}m)，对准过差无法入库')
                        self._sm.fail()
                        self._adapter.publish_stop()
                        return
                go_straight = True
            else:
                # 直行区外（接近初期 / 重试倒车后）→ 重新武装入口检查。
                # 已达入口包络的直行捷径只在近场 (dist ≤ tighten_distance) 启用:
                # 远场交给 plan() 粗对准 (far_* 门槛, normal=None 不做法线对准/
                # 横移) —— 大方向对齐后纯前进逼近, 走进近场再精调, 不带大误差
                # 直冲直行区。
                self._straight_entered = False
                if (tag_pose.dist <= tighten_dist
                        and bearing_err <= straight_yaw_tol
                        and abs(tag_pose.lat) <= lat_threshold_m):
                    # 方位(bearing)+横向都已在收紧门槛内 → 直接直行, 不再摆头。
                    # 不查法线: normal 是 AprilTag 最不可靠的自由度(近场 ±180°
                    # 抖动, 实测已对正标签仍被算成 ~8° 偏角), 入口门控同理只用
                    # bearing+横向。方位只依赖标签在画面中的位置, 稳定可靠。
                    go_straight = True

        # 走停步长运行期可调: ros2 param set <节点> stopgo.jog_max 0.5 即时生效,
        # 无需重启 —— 每次规划前把当前参数同步进规划器 (launch 也可传 jog_max:=)。
        # 修正容差按远/近区分 (two_phase 启用时):
        #   远场 (dist > tighten_distance): 放宽到 far_*, 只做大尺度粗对准+前进,
        #     不做法线对准/横移 (plan() 传 normal=None)。1.5m 处 normal 噪声
        #     ±10° 被 dist 放大成 ±0.3m 横移噪声, 是反复/反向横移的根源。
        #   近场 (dist ≤ tighten_distance): 收紧到入口包络, 做"对齐法线→
        #     垂直偏距横移→直行"。法线转向门槛钳在 normal_turn_min_deg(6°)
        #     之上 (低于噪声下限会摆头); 横移量由规划器改用垂直偏距
        #     dist·sin(n)−lat·cos(n), 航向残余在门槛内也移向正确的线
        #     (按画面 lat 横移在航向未转正时会反向, 见 plan() 内注释)。
        # two_phase 关闭时退回到原单阶段 stopgo 值 (legacy)。
        near_field = two_phase and tag_pose.dist <= tighten_dist
        stopgo_lat = self._p('stopgo.lateral_threshold')
        stopgo_yaw = math.radians(self._p('stopgo.yaw_threshold_deg'))
        if two_phase:
            if near_field:
                # 法线转向门槛钳到噪声下限之上 (见参数声明处): 低于噪声下限的
                # 门槛让转向决策追二义性双峰抖动, 摆头不止、横移触发不了。
                normal_turn_thr = math.radians(max(
                    self._p('final_straight.normal_yaw_threshold_deg'),
                    self._p('final_straight.normal_turn_min_deg')))
                lat_thr, yaw_thr, bear_thr = (
                    lat_threshold_m, normal_turn_thr, straight_yaw_tol)
            else:
                lat_thr = float(self._p('final_straight.far_lateral_m'))
                yaw_thr = bear_thr = math.radians(
                    self._p('final_straight.far_yaw_threshold_deg'))
        else:
            lat_thr, yaw_thr, bear_thr = stopgo_lat, stopgo_yaw, stopgo_yaw
        self._planner.set_tolerances(
            lateral_threshold=lat_thr,
            yaw_threshold=yaw_thr,
            bearing_yaw_threshold=bear_thr)
        # 直行区外的前进步长钳到"恰好停在区界": 跨界盲走越短, 入口检查拿到
        # 的量测越新鲜 (最低保 jog_min, 防 plan_straight 的 drive ≤ jog_min/2
        # 判 done 原地打转)。
        jog_max_eff = self._p('stopgo.jog_max')
        if two_phase and tag_pose.dist > straight_start:
            jog_max_eff = min(jog_max_eff,
                              max(tag_pose.dist - straight_start,
                                  self._p('stopgo.jog_min')))
        self._planner.set_jog_limits(
            jog_min=self._p('stopgo.jog_min'),
            jog_max=jog_max_eff)

        if go_straight:
            seq = self._planner.plan_straight(tag_pose.dist)
            self.get_logger().info(
                f'走停 直线阶段 iter={self._maneuver_iters}: '
                f'距离={tag_pose.dist:.3f}m → 目标={target_distance:.3f}m',
                throttle_duration_sec=2.0)
        elif self._is_omni(base_type):
            # Omni：逐帧小步规划。两阶段模式下:
            #   远场 (dist > tighten_distance): normal=None, plan() 走纯方位
            #     (bearing) 对准+前进(pure pursuit), 不做法线对准/横移 —— 避免
            #     1.5m 处 normal 噪声放大成反复/反向横移。
            #   近场 (dist ≤ tighten_distance) / 单阶段: 传 normal, 走"对齐
            #     法线 → 按垂直偏距横移 → 直行"。
            # 不用 plan_sequence 的法线盲机动：其 standoff 点 A = tag + d_target·n
            # 在 dist ≈ d_target 时贴在机器人脚下, turn1=atan2(A) 对厘米级测量
            # 噪声极敏感（实测 dock_distance=1.2 @ dist=1.28 规划出 ±44° 小挪
            # 动）；且大角度盲转的腿式滑移让真实位移超出里程计判停值，下一轮
            # 测量突变（dist 跌破 d_target → A 翻到身后 → 转 124° 往回走），
            # 正反馈打转。
            # 横移按画面 lat 平移只在车头与法线平行时才等价于"平移到法线上"：
            # lat = 真实垂直偏距 − dist·sin(残余航向误差)，航向没转正时按 lat
            # 横移会把 tag 挪到画面正中、车体却离法线更远 (2026-09 实测反向
            # 横移)。故 omni 顺序：先原地转齐法线 (转角 = normal+π, 仍受
            # max_turn_step 逐步钳制+停稳重测)；横移量由规划器改用垂直偏距
            # dist·sin(n)−lat·cos(n) —— 航向残余 ≤ 门槛也移向正确的线；残余
            # 航向在移正后再由 aim-and-go 的 bearing 转向收掉 (在法线上
            # bearing = −航向误差, 转齐 bearing 即同时转正航向)，最后直线前进。
            align_norm = (not two_phase) or near_field
            step = self._planner.plan(tag_pose.dist, tag_pose.lat, bearing,
                                      normal=(tag_pose.normal if align_norm else None))
            if step.kind == 'yaw':
                # 大角度对准按 max_turn_step 分批：每步停稳重测，避免一次
                # 大盲转的滑移污染下一轮测量。
                step.turn_angle = math.copysign(
                    min(abs(step.turn_angle), self._p('stopgo.max_turn_step')),
                    step.turn_angle)
            seq = [step]
        else:
            seq = self._planner.plan_sequence(
                tag_pose.dist, tag_pose.lat, tag_pose.normal, yaw_tol=yaw_tol)

        # 移动前打印：二维码相对位姿 + 完整规划路径，仅凭日志即可诊断丢标问题。
        # bearing = 指向二维码的方向；normal = 二维码朝外法线方向；
        # 每一步以带符号量显示（转向单位度，前进单位米）。
        bearing_deg = math.degrees(math.atan2(tag_pose.lat, tag_pose.dist))
        steps = []
        for p in seq:
            if p.kind == 'yaw':
                steps.append(f'转 {math.degrees(p.turn_angle):+.1f}°')
            elif p.kind == 'forward':
                if abs(p.lateral_distance) > 1e-4:
                    steps.append(f'横移 {p.lateral_distance:+.3f}m')
                else:
                    steps.append(f'前进 {p.jog_distance:+.3f}m')
            else:
                steps.append(p.kind)
        self.get_logger().info(
            f'走停 规划 iter={self._maneuver_iters}: '
            f'二维码 距离={tag_pose.dist:.3f}m 横向={tag_pose.lat:+.3f}m '
            f'方位={bearing_deg:+.1f}° 法线={math.degrees(tag_pose.normal):+.1f}° '
            f'| 原始 距离={self._raw_dist:.3f} 横向={self._raw_lat:+.3f} '
            f'法线={math.degrees(self._raw_normal):+.1f}° '
            f'| 直行={go_straight} 方位误差={math.degrees(bearing_err):.1f}°'
            f'(失败门槛{math.degrees(straight_yaw_tol):.1f}°) '
            f'| 路径 [{", ".join(steps)}]', throttle_duration_sec=1.0)

        if len(seq) == 1 and seq[0].kind == 'done':
            self._adapter.publish_stop()
            # Leave APPROACH→FINAL_SERVO/DOCKED to the state machine (it checks
            # the same tolerance on tag_pose).
            # 容差内: 保持静止站立 (不发 stand_up), 由 FINAL_SERVO→DOCKED
            # 确认 —— 到位后狗不喘、姿态最稳。
            return

        # 规划完成 → 暂存待发, 由 _launch_pending_seq 在恢复运动模式
        # (motion_enabled=True) 后原样启动 —— 解锁等待期间不再重测/重规划,
        # 上面的规划日志因此每停只打一次。
        self._pending_seq = list(seq)
        if not self._launch_pending_seq(base_type, now_ns):
            return

    def _run_retry(self, base_type: str, now_ns: int):
        """重试倒车的一个 tick: 盲退 retry.backup_distance, 到位后 → SEARCH_TAG。

        镜像 _run_search 的 Case 1/2 结构。倒车纯里程计闭环(blind), 不看 tag;
        退够距离后调状态机 retry_search() 转 SEARCH_TAG 重新锁定靠近。
        进入 RETRYING 时控制循环的 cleanup(离开 stopgo 态)已 cancel 旧 executor
        + _reset_maneuver, 故首 tick executor 空闲 → Case 2 启动盲退。
        """
        # Case 1: 倒车执行中
        if self._executor.is_active:
            done = self._executor.update(
                self._odom_x, self._odom_y, self._odom_yaw,
                False, None,   # blind: 不看 tag
                self._bearing_fn, self._theta_bounds_fn,
                self._p('dock_target.distance'),
                self._p('stopgo.drift_tol'), now_ns,
            )
            if done:
                self._maneuver_active = False
                self._mark_stopped(now_ns)
                self._sm.retry_search()
            self._publish_action_cmd(base_type)
            return

        # Case 2: 倒车未开始 → 启动盲退(负距离 = 后退)
        dist = abs(float(self._p('retry.backup_distance')))
        rate = float(self._p('retry.linear_rate'))
        if dist < 1e-3:
            self._sm.retry_search()
            self._adapter.publish_stop()
            return
        # 上一轮失败/到站时狗可能仍处于静止站立 (DOCKED 约定), cmd_vel 会被
        # 桥拒绝 → 里程计不走 → 15s 重试超时。先恢复运动模式再盲退。
        if not self._posture.motion_ready(now_ns):
            self._adapter.publish_stop()
            return
        self._executor.start_jog(
            -dist, rate, blind=True,
            odom_scale=self._p('stopgo.jog_backward_odom_scale'))
        self._executor.set_odom_ref(self._odom_x, self._odom_y, self._odom_yaw)
        self._maneuver_active = True
        self._frozen = True
        self._discard_dual_pending()
        self.get_logger().info(
            f'重试：盲退 {-dist:+.3f}m (速率 {rate:.2f}m/s) 后重新锁定')
        self._publish_action_cmd(base_type)

    def _run_undock(self, base_type: str, now_ns: int):
        """泊出的一个 tick: 盲退 → 原地转 180° → UNDOCKED。

        两段纯里程计闭环盲动顺序执行 (镜像 _run_retry 的 Case 结构)：
          phase 0: 盲退 undock.backup_distance (负 jog, 同重试倒车)
          phase 1: 原地转 undock.turn_angle_deg (默认 180°)
        两段都到位后调状态机 finish_undock() → UNDOCKED。
        不看 tag；运动期冻结检测 (与停泊盲动一致)。
        """
        # Case 1: 子动作执行中
        if self._executor.is_active:
            done = self._executor.update(
                self._odom_x, self._odom_y, self._odom_yaw,
                False, None,   # blind: 不看 tag
                self._bearing_fn, self._theta_bounds_fn,
                self._p('dock_target.distance'),
                self._p('stopgo.drift_tol'), now_ns,
            )
            if done:
                self._maneuver_active = False
                self._executor.mark_stop_time(now_ns)
                self._undock_phase += 1
                if not self._start_undock_step(base_type):
                    # 两段盲动均完成 → 泊出成功
                    self._adapter.publish_stop()
                    self._sm.finish_undock()
                    self._charge.reset()
                    return
            self._publish_action_cmd(base_type)
            return

        # Case 2: 首次进入 → 启动第一段(盲退)
        if not self._has_odom:
            self._adapter.publish_stop()
            self.get_logger().warn('泊出：等待里程计...', throttle_duration_sec=1.0)
            return
        # DOCKED 按约定保持静止站立; 盲退前必须 stand_up 并等 motion_enabled,
        # 否则 cmd_vel 被桥拒绝 → 泊出超时。phase0→1 链式段已解锁不再处理。
        # 充电收尾后狗趴着/阻尼 (posture.enable=false 时 posture 不管):
        # 先由 charge 管理器 stand_up 恢复运动, 它不门控时直接放行。
        if not self._charge.motion_ready(now_ns):
            self._adapter.publish_stop()
            return
        if not self._posture.motion_ready(now_ns):
            self._adapter.publish_stop()
            return
        self._undock_phase = 0
        if not self._start_undock_step(base_type):
            self._adapter.publish_stop()
            self._sm.finish_undock()
            self._charge.reset()
            return
        self._publish_action_cmd(base_type)

    def _start_undock_step(self, base_type: str) -> bool:
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
                    -dist, rate, blind=True,
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
            if self._executor.start_turn(angle, rate, full=True):
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

    def _run_search(self, tag_visible: bool, tag_pose, base_type: str,
                    now_ns: int):
        """角度步进搜索的一个 tick：转固定角度(里程计闭环)→停稳→检测→再转。

        转满 360° 直到找到二维码或状态机超时。结构镜像 _run_stop_and_go 的
        Case 级联（执行中→等待稳定→解冻清旧数据→空闲检测），区别是检测期
        不规划靠近动作，而是累计 pause_time_sec 不可见就再转一个 step_angle。
        冻结机制保证转动期 tag_visible 恒为 False，状态机的 tag-lock 不会误触发。
        """
        # ── Case 1: 搜索步正在执行（里程计闭环盲转）──────────────
        if self._executor.is_active:
            done = self._executor.update(
                self._odom_x, self._odom_y, self._odom_yaw,
                tag_visible, self._raw_dist,
                self._bearing_fn, self._theta_bounds_fn,
                self._p('dock_target.distance'),
                self._p('stopgo.drift_tol'), now_ns,
            )
            if done:
                self._maneuver_active = False
                self._mark_stopped(now_ns)
            self._publish_action_cmd(base_type)
            return

        if self._dual.enabled and now_ns < self._dual.settle_until_ns:
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

        # ── Case 2.6: 检测期同样静止站立 (首次进入/重进搜索时自武装补锁) ──
        # 锁定下检测, 二维码位姿不被呼吸晃动; 降级/停用时直接放行。
        if not self._posture.lock_settled(now_ns):
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

        if (self._dual.enabled and self._search_step * abs(float(
                self._p('search.step_angle_deg'))) >= 360.0):
            self._adapter.publish_stop()
            self._sm.abort_motion('dual bounded wall search exhausted one revolution')
            return

        # 停留期满仍未见到 → 先恢复运动模式再转下一步。解锁等待期间保持
        # 停留状态 (不重置 _search_detect_start), 否则每步多等一个 pause_time。
        if not self._posture.motion_ready(now_ns):
            self._adapter.publish_stop()
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
        if self._executor.start_turn(angle, rate, full=True):
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
        self._publish_action_cmd(base_type)

    def _start_next_maneuver_step(self, base_type: str):
        """Pop and start the next queued sub-step, re-referencing odometry.

        Skips over sub-steps too small to actually move (a sub-degree turn or a
        sub-millimetre jog): those would leave the executor idle, so without the
        skip the blind chain would stall (Case 1 only advances the queue when an
        action was running). Keeps popping until one step starts or the queue
        empties.
        """
        while self._maneuver_queue:
            step = self._maneuver_queue.pop(0)
            if self._launch_step(step, base_type):
                return
        # Queue drained without launching anything → maneuver is over.
        self._maneuver_active = False
        self._mark_stopped(self.get_clock().now().nanoseconds)

    def _launch_pending_seq(self, base_type: str, now_ns: int) -> bool:
        """处理已规划的待发序列。返回 False = 本 tick 到此为止, 调用方立即 return
        (仍在等 stand_up 解锁, 或已起步 —— 起步后若贯穿落入 Case 3 会在同一
        tick 重测重规划、覆写刚启动的队列); True = 无待发序列, 继续量测规划。

        规划完成后序列暂存 _pending_seq, 由本方法在静止站立解锁确认
        (motion_enabled=True) 后原样启动 —— 解锁等待期间不重测/不重规划,
        规划日志因此每停只打一次。启动后冻结检测 (盲动期丢弃所有帧)。
        """
        if self._pending_seq is None:
            return True
        if self._dual.enabled and not self._dual.pending_valid(now_ns):
            self._pending_seq = None
            self._adapter.publish_stop()
            self._dual.stopped(now_ns)
            self._reset_visual_state()
            return False
        if not self._posture.motion_ready(now_ns):
            self._adapter.publish_stop()
            return False
        seq, self._pending_seq = self._pending_seq, None
        self._maneuver_queue = list(seq)
        self._maneuver_active = True
        self._frozen = True          # 开始盲动：冻结检测，运动期丢弃所有帧
        self._discard_dual_pending()
        self._maneuver_iters += 1
        self._start_next_maneuver_step(base_type)
        return False

    def _launch_step(self, plan: ActionPlan, base_type: str) -> bool:
        """Start one executor action from an ActionPlan and re-ref odometry.

        Returns True if an action actually started, False if the step was too
        small to move (caller advances to the next queued step).
        """
        if self._dual.enabled:
            now = self.get_clock().now().nanoseconds
            stamp = getattr(self, '_odom_stamp_ns', 0)
            if self._executor.is_active:
                return False  # No new start, budget or qualification commit.
            if (stamp <= 0 or not 0 <= now-stamp <= self._dual.p('odom_fresh_sec')*1e9
                    or not all(math.isfinite(v) for v in (self._odom_x, self._odom_y, self._odom_yaw))):
                self._adapter.publish_stop()
                self._sm.abort_motion('dual requires fresh odometry before action start')
                return False
        if plan.kind == 'yaw':
            # Full computed turn — no undershoot, no per-step cap. The whole
            # turn-drive-turn path was computed together; a clamped turn1 would
            # drive the full leg along the wrong heading.
            if not self._executor.start_turn(
                    plan.turn_angle, self._p('stopgo.jog_angular_rate'),
                    full=True):
                return False
            self._executor.set_odom_ref(
                self._odom_x, self._odom_y, self._odom_yaw)
            self.get_logger().info(
                f'  子步：原地转 {math.degrees(plan.turn_angle):+.1f}°（盲转，里程计校准）')
        elif plan.kind == 'forward':
            if abs(plan.lateral_distance) > 1e-4 and self._is_omni(base_type):
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
                self._executor.start_jog(
                    plan.jog_distance, self._p('stopgo.jog_linear_rate'),
                    blind=True, odom_scale=scale)
                if not self._executor.is_active:
                    return False
                self._executor.set_odom_ref(
                    self._odom_x, self._odom_y, self._odom_yaw, bearing=0.0)
                self.get_logger().info(
                    f'  子步：前进 {plan.jog_distance:+.3f}m（盲走，里程计校准）')
        else:
            return False
        if self._dual.enabled:
            speed = (self._executor.angular_cmd if plan.turn_angle else
                     self._executor.lateral_cmd if plan.lateral_distance else self._executor.linear_cmd)
            self._dual_watch = ActionWatch(plan, now,
                (self._odom_x, self._odom_y, self._odom_yaw),
                self._executor._action_target, speed, self._dual.p)
            self._dual.action_started(plan, now)
        self._publish_action_cmd(base_type)
        return True

    def _check_dual_action(self, now):
        watch = self._dual_watch
        reason = ('dual missing action start reference' if watch is None else watch.check(
            now, (self._odom_x,self._odom_y,self._odom_yaw), getattr(self,'_odom_stamp_ns',0)))
        if reason:
            self._adapter.publish_stop()
            self._executor.cancel()
            self._pending_seq = None
            self._maneuver_queue = []
            self._maneuver_active = False
            self._dual.failure = reason
            self._dual._travel_qualification = 0
            self._sm.abort_motion(reason)
            return False
        return True

    @property
    def maneuver_active(self) -> bool:
        """True while a blind turn-drive-turn maneuver is executing.

        During this window the tag legitimately leaves view, so the state
        machine must NOT count tag-loss toward the SEARCH_TAG fallback.
        """
        return self._maneuver_active or self._executor.is_active

    def _publish_action_cmd(self, base_type: str):
        """Publish the current action's velocity command via the adapter."""
        kind = self._executor.action_kind
        if kind == 'jogging':
            self._adapter.publish_jog(self._executor.linear_cmd)
        elif kind == 'turning':
            # 纯原地转弯。之前的"边走边转"(arc)方案会叠加前进速度，可能让小车
            # 驶出目标的横向范围——已弃用。转向精度靠里程计校准（full=True 全量
            # 盲转），转得慢一点没关系；若差速轮原地转需克服静摩擦，宁可加大
            # jog_angular_rate，也不叠加前向速度。
            self._adapter.publish_turn(self._executor.angular_cmd)
        elif kind == 'lateral':
            if hasattr(self._adapter, 'publish_lateral'):
                self._adapter.publish_lateral(self._executor.lateral_cmd)
            else:
                self._adapter.publish_stop()
        else:
            self._adapter.publish_stop()

    def _mark_stopped(self, now_ns: int):
        """停稳边界: 视觉 settle 时钟 + 静止站立锁定请求 (每停只武装一次)。

        泊出 phase0→1 的链式段故意不走这里 —— 无缝盲链中间没有停看点,
        锁定只发生在确实要"停下来看"的时刻。
        """
        self._executor.mark_stop_time(now_ns)
        self._posture.on_stop(now_ns)
        if self._dual.enabled:
            self._dual.stopped(now_ns)

    def _reset_visual_state(self):
        """解冻 + 丢弃运动期全部旧位姿 + 重置稳定帧计数。

        强制下一次规划只用停稳解冻后新采的新鲜帧 (EMA 重新播种);
        稳定帧门 (_stable_frames) 从零重新累计。
        """
        self._frozen = False
        self._filter_init = False          # EMA 重新播种（首帧新鲜帧作种子）
        self._last_detection_ns = 0        # _tag_fresh() 归零，等待新帧
        if hasattr(self, '_dual_live_wall_ns'):
            del self._dual_live_wall_ns    # 新锁定回退到本窗口 dual.stamp，绝不沿用旧任务
        self._pose_buffer.clear()          # 丢弃所有历史缓冲位姿
        self._stable_frames = 0
        self._stable_window_start_ns = 0
        self._dual.reset_filter()
        self._discard_dual_pending()
        if self._dual.enabled:
            self._dual.settle_until_ns = max(self._dual.settle_until_ns,
                self.get_clock().now().nanoseconds)          # 桩码 EMA 同步丢弃 (hold 跨停保持)

    def _reset_maneuver(self):
        """Clear any queued/active blind maneuver and its iteration counter.

        Called on cancel/abort and whenever we leave the stop-and-go states, so
        a fresh docking attempt always re-plans from a new measurement and no
        stale sub-step can resume against an outdated odometry reference.
        """
        self._maneuver_queue = []
        self._maneuver_active = False
        self._maneuver_iters = 0
        self._pending_seq = None
        self._reset_visual_state()
        self._straight_entered = False
        self._dual_watch = None
        self._dual_posture.reset()          # 姿态序列回 IDLE (新轮 dock 重新趴下)
        self._dual_stand_grace_ns = 0
        self._dual_diag_ns.clear()
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

    def _bearing_fn(self) -> float:
        """Current tag bearing from robot (rad)."""
        if self._raw_dist is None or self._raw_lat is None:
            return 0.0
        return math.atan2(self._raw_lat, self._raw_dist)

    def _theta_bounds_fn(self) -> float:
        """Dynamic theta tolerance — tighter when closer."""
        if self._raw_dist is None or self._raw_dist <= 0.0:
            return math.radians(self._p('stopgo.yaw_threshold_deg'))
        ratio = self._p('stopgo.theta_shrink_ratio')
        return max(
            math.radians(self._p('stopgo.yaw_threshold_deg')),
            self._raw_dist / max(ratio, 0.1),
        )

    def _blind_cap_for_turn(self) -> float | None:
        """Maximum turn angle when tag is not visible."""
        return min(
            self._p('stopgo.max_turn_step'),
            2.0 * self._theta_bounds_fn(),
        )

    @staticmethod
    def _is_omni(base_type: str) -> bool:
        return base_type in ('omni', 'quadruped')

    # ── Params dict ────────────────────────────────────────────────

    def _build_params_dict(self) -> dict:
        return {
            'timeout_sec': self._p('timeout_sec'),
            'tag': {
                'tag_loss_timeout_sec': self._p('tag.tag_loss_timeout_sec'),
            },
            'search': {
                'angular_speed': self._p('search.angular_speed'),
                'step_angle_deg': self._p('search.step_angle_deg'),
                'rotate_time_sec': self._p('search.rotate_time_sec'),
                'pause_time_sec': self._p('search.pause_time_sec'),
                'hold_time_sec': self._p('search.hold_time_sec'),
                'search_direction': self._p('search.search_direction'),
                'timeout_sec': self._p('search.timeout_sec'),
            },
            'tolerance': {
                'position_m': self._p('tolerance.position_m'),
                'yaw_deg': self._p('tolerance.yaw_deg'),
                'stable_time_sec': self._p('tolerance.stable_time_sec'),
            },
            'dock_target': {
                'distance': self._dual.effective_dock_distance(),
                'lateral_offset': self._p('dock_target.lateral_offset'),
                'yaw_offset_deg': self._p('dock_target.yaw_offset_deg'),
            },
            'safety': {
                'minimum_distance_m': self._p('safety.minimum_distance_m'),
            },
            'retry': {
                'timeout_sec': self._p('retry.timeout_sec'),
            },
            'undock': {
                'timeout_sec': self._p('undock.timeout_sec'),
            },
            'align_timeout_sec': self._p('align_timeout_sec'),
            'approach_timeout_sec': self._p('approach_timeout_sec'),
            'final_servo_timeout_sec': self._p('final_servo_timeout_sec'),
            'final_servo': {
                'yaw_tol_deg': self._p('final_servo.yaw_tol_deg'),
            },
        }

    # ── Publishing ─────────────────────────────────────────────────

    def _publish_state(self, state: DockingState):
        msg = String()
        msg.data = state.name.lower()
        self._state_pub.publish(msg)

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
        self._planner.reset()
        self._executor.cancel()
        self._reset_maneuver()
        self._charge.reset()
        return response

    def _on_cancel_docking(self, request, response):
        self._sm.cancel()
        self._planner.reset()
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
            self._planner.reset()
            self._executor.cancel()
            self._reset_maneuver()
            self._undock_phase = 0
        return response

    # ── Action callbacks ───────────────────────────────────────────

    def _execute_dock_cb(self, goal_handle):
        """Action execute callback — blocks until docking completes."""
        from tagdocking.action import Dock

        ok = self._sm.start()
        if not ok:
            goal_handle.abort()
            return Dock.Result(success=False, message='already active')

        self._planner.reset()
        self._executor.cancel()
        self._reset_maneuver()
        self._charge.reset()

        feedback = Dock.Feedback()

        while rclpy.ok() and self._sm.is_active:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self._sm.cancel()
                self._planner.reset()
                self._executor.cancel()
                self._reset_maneuver()
                self._adapter.publish_stop()
                return Dock.Result(success=False, message='cancelled')

            tag_pose = self._get_latest_pose()
            if tag_pose is not None:
                target_dist = self._dual.effective_dock_distance()
                feedback.distance_error = abs(tag_pose.dist - target_dist)
                feedback.yaw_error = abs(tag_pose.yaw)
            feedback.state = self._sm.state_name.lower()
            goal_handle.publish_feedback(feedback)

            rclpy.spin_once(self, timeout_sec=0.05)

        if self._sm.is_success:
            goal_handle.succeed()
            return Dock.Result(success=True, message='docked')
        else:
            goal_handle.abort()
            return Dock.Result(success=False, message=self._sm.state_name.lower())

    def _dock_cancel_cb(self, cancel_request):
        self._sm.cancel()
        self._planner.reset()
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
            try:
                rclpy.shutdown()
            except Exception:
                pass

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
        # 信号处理器(_handle_signal)会调用 rclpy.shutdown() 以便打断 spin,
        # 但这会让正在转的 spin 下一轮 wait_set 初始化抛 RCLError
        # ("context is not valid")。上下文已被有意关闭时属正常退出路径,
        # 吞掉以免 launch 报 process died; 仅真异常(rclpy 仍 ok)才重新抛出。
        if rclpy.ok():
            raise
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            node._tf_node.destroy_node()
        except Exception:
            pass
        # The signal handler may have already shut down the context; calling
        # rclpy.shutdown() again raises "Context must be initialized". Guard it
        # so launch gets a clean exit instead of SIGKILL-ing us into zombies.
        if rclpy.ok():
            rclpy.shutdown()
