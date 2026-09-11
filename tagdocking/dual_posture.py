"""双码姿态切换管理器: 搜索锁定墙码 → 趴下(匍匐), 桩码锁定距离 → 站立。

机器狗流程 (用户语义):
    站立搜索到墙码 → 趴下匍匐 (桩码贴桩底座更矮, 站立时摄像头看不到)
    → 匍匐完成两码对准 + 前进 → 桩码 z ≤ 锁定距离 (30cm 再近将出视野)
    → 站立 (匍匐进不了桩底座) → 站立直行 → 距墙码 0.50m → 阻尼坐桩。

桥接口 (l1w_control, std_srvs/Trigger): {prefix}/lie_down 与 {prefix}/stand_up。
motion_enabled (Bool, transient_local) 是桥的 cmd_vel 门控本身:
  - 站立确认以此为准 (motion_enabled==True 才真的能走, 与 posture_mode 同判据);
  - 匍匐行走前提: 桥在 lie_down 后保持 motion_enabled=True (匍匐步态接受
    cmd_vel)。若桥趴下后锁 cmd_vel (False), 这里明确 failure —— 不猜测,
    提示检查桥的 lie_down 门控行为。None (无状态回传) 视为无桥语义放行。

确认方式:
  - 趴下: 服务 success + crouch_settle_sec 定时 —— posture_state 只有
    static_stand/standing/not_standing, 没有趴下专属值, 无法用状态回传确认;
  - 站立: motion_enabled==True。

失败策略: 趴下失败 → failure (桩码矮, 站立看不见, 双码无法继续, 节点
abort_motion); 站立失败 → failure (匍匐进不了桩底座, 必须中止而不是带病
直行撞桩)。

所有调用非阻塞 (call_async + 20Hz 轮询), 绝不 sleep/spin_until_future_complete
—— 与 posture_mode/charge_mode 同一约定。
"""

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


class DualPosture:
    """趴下(匍匐) ↔ 站立 的双态序列, 每个控制 tick 轮询推进。

    状态:
      IDLE       初始/站立未锁 (dock 启动时狗在站立搜索)
      CROUCHING  lie_down 已发, 等响应 + settle
      CROUCHED   匍匐就绪 (双码对准/前进在此姿态)
      STANDING   stand_up 已发, 等 motion_enabled==True
      STANDED    站立就绪 (locked 纯直行)
      FAILED     服务失败/超时/门控冲突 (failure 带原因, 节点 abort)
    """

    IDLE, CROUCHING, CROUCHED, STANDING, STANDED, FAILED = range(6)

    def __init__(self, node):
        self._node = node
        self.failure = ''
        self._phase = self.IDLE

        self._crouch_settle_ns = int(
            float(node._p('dual.crouch_settle_sec')) * 1e9)
        self._ack_ns = int(float(node._p('posture.lock_ack_timeout_sec')) * 1e9)
        self._retries = int(node._p('dual.posture_retries'))
        self._service_wait_ns = int(
            float(node._p('posture.service_wait_sec')) * 1e9)

        prefix = node._p('base.l1w_prefix')
        self._cli_lie = node.create_client(Trigger, f'{prefix}/lie_down')
        self._cli_stand = node.create_client(Trigger, f'{prefix}/stand_up')

        # 与 posture_mode 的订阅并行 (各自回调, 互不干扰); latched 快照对齐
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._motion_sub = node.create_subscription(
            Bool, f'{prefix}/motion_enabled', self._on_motion_enabled, qos)
        self._motion_enabled: bool | None = None

        self._req_ns = 0
        self._future = None
        self._tries = 0
        self._first_use_ns: int | None = None

    def _on_motion_enabled(self, msg: Bool):
        self._motion_enabled = bool(msg.data)

    # ── 对外 API ────────────────────────────────────────────────────

    def reset(self):
        """新一轮 dock 开始: 回 IDLE (重置时狗应为站立/运动模式)。"""
        self._phase = self.IDLE
        self.failure = ''
        self._future = None
        self._tries = 0

    def ensure_crouch(self, now_ns: int) -> bool:
        """True → 匍匐就绪 (可开始双码对准); False → 切换中或失败。"""
        if self._phase == self.CROUCHED:
            return True
        if self._phase == self.FAILED:
            return False
        if self._phase == self.IDLE:
            if not self._request(self._cli_lie, now_ns, '趴下 (lie_down, 匍匐)'):
                return False
            self._phase = self.CROUCHING
            return False
        # CROUCHING: 等响应 + settle
        if not self._poll(now_ns, 'lie_down', self.CROUCHING):
            return False
        # 响应成功且 settle 满 → 检查桥是否允许匍匐行走
        if (self._motion_enabled is False
                and now_ns - self._req_ns > self._crouch_settle_ns):
            self._fail('lie_down 后 motion_enabled=False (桥锁了 cmd_vel), '
                       '无法匍匐行走 —— 检查桥的 lie_down 门控行为')
            return False
        if now_ns - self._req_ns >= self._crouch_settle_ns:
            self._phase = self.CROUCHED
            if self._motion_enabled is None:
                self._node.get_logger().warn(
                    'lie_down 已确认但无 motion_enabled 回传 —— 假定匍匐可行走, '
                    '若现场卡住请检查桥')
            self._node.get_logger().info('匍匐就绪 (趴下完成并稳定) → 开始双码对准')
            return True
        return False

    def ensure_stand(self, now_ns: int) -> bool:
        """True → 站立就绪 (motion_enabled 已确认, 可发直行); False → 切换中/失败。"""
        if self._phase == self.STANDED:
            return True
        if self._phase == self.FAILED:
            return False
        if self._phase in (self.IDLE, self.CROUCHED):
            if not self._request(self._cli_stand, now_ns, '站立 (stand_up)'):
                return False
            self._phase = self.STANDING
            return False
        # STANDING: motion_enabled 是唯一确认 (它就是 cmd_vel 门控本身)
        if self._motion_enabled is True:
            self._phase = self.STANDED
            self._node.get_logger().info('站立完成 (motion_enabled=True) → locked 纯直行')
            return True
        if self._motion_enabled is None:
            if self._first_use_ns is None:
                self._first_use_ns = now_ns
            if (now_ns - self._first_use_ns > self._service_wait_ns
                    and not self._cli_stand.service_is_ready()):
                self._fail('无 motion_enabled 回传且 stand_up 服务不可用, '
                           '无法确认站立')
                return False
        if now_ns - self._req_ns > self._ack_ns:
            if self._tries <= self._retries:
                self._tries += 1
                self._req_ns = now_ns
                self._future = self._cli_stand.call_async(Trigger.Request())
                self._node.get_logger().warn(
                    f'stand_up 未确认, 重试 ({self._tries}/{self._retries + 1})')
                return False
            self._fail('stand_up 确认超时 (motion_enabled 未变 True)')
        return False

    # ── 内部 ────────────────────────────────────────────────────────

    def _request(self, client, now_ns: int, what: str) -> bool:
        """发服务请求; 服务不可用给宽限, 超宽限落 FAILED。"""
        if not client.service_is_ready():
            if self._first_use_ns is None:
                self._first_use_ns = now_ns
            if now_ns - self._first_use_ns > self._service_wait_ns:
                self._fail(f'{what} 服务不可用 (l1w_control 桥缺失?)')
            return False
        self._first_use_ns = None
        self._node.get_logger().info(f'双码姿态: 请求{what}')
        self._req_ns = now_ns
        self._tries = 1
        self._future = client.call_async(Trigger.Request())
        return True

    def _poll(self, now_ns: int, name: str, retry_phase: int) -> bool:
        """CROUCHING 专用: 响应失败重试/耗尽; 成功交由 settle 计时收尾。"""
        if self._future is not None and self._future.done():
            try:
                result = self._future.result()
            except Exception as exc:
                result = None
                self._node.get_logger().warn(f'{name} 服务异常: {exc}')
            if result is None or not result.success:
                if self._tries <= self._retries:
                    self._tries += 1
                    self._req_ns = now_ns
                    client = (self._cli_lie if name == 'lie_down'
                              else self._cli_stand)
                    self._future = client.call_async(Trigger.Request())
                    self._node.get_logger().warn(
                        f'{name} 未受理, 重试 ({self._tries}/{self._retries + 1})')
                    return False
                self._fail(f'{name} 服务失败/被拒, 重试耗尽')
                return False
        if now_ns - self._req_ns > self._ack_ns + self._crouch_settle_ns:
            if self._tries <= self._retries:
                self._tries += 1
                self._req_ns = now_ns
                client = (self._cli_lie if name == 'lie_down'
                          else self._cli_stand)
                self._future = client.call_async(Trigger.Request())
                self._node.get_logger().warn(
                    f'{name} 响应超时, 重试 ({self._tries}/{self._retries + 1})')
                return False
            self._fail(f'{name} 确认超时 (响应与 settle 均未在时限内完成)')
            return False
        return True

    def _fail(self, reason: str):
        self._phase = self.FAILED
        self.failure = reason
        self._node.get_logger().error('双码姿态切换失败: ' + reason)
