"""充电收尾 (DOCKED 后) 模式序列管理器 — 静止(锁定) → 阻尼。

对接成功后狗站在充电桩上方, 锁定后直接切阻尼泄力, 保持站立 (不趴下):

    static_stand (CMD_LOCK_MODE, 静止锁定)
      → 等 posture_state==static_stand
    passive (CMD_EMERGENCY_STOP, 电机泄力 = 本 SDK 的"阻尼")
      → 等服务成功响应，再计 passive_settle_sec（不等于硬件或电流反馈）
      (charge.passive=false 时跳过本步, 收尾止于锁定)
      (charge.static_stand=false 时跳过锁定, 直接阻尼)
    → DONE

与 posture.enable 完全无关: 即使关闭了中途的锁定/解锁 (呼吸抑制),
DOCKED 后也必须执行整个序列 —— 呼吸抑制管的是"停-看循环里的量测稳定",
充电收尾管的是"泊完后把狗放好", 两者互不相干。

所有调用全部非阻塞 (只用 call_async, 由 20Hz 控制循环在 DOCKED 态轮询
tick 推进), 绝不在回调里 sleep / spin_until_future_complete —— 与
posture_mode 同一套约定。

泊出衔接: 收尾完成后狗站立锁定/阻尼且 cmd_vel 被桥门控 (motion_enabled=False),
直接盲退发不出速度。start_undock 后由 _run_undock 调 motion_ready():
中断序列 → stand_up (CMD_STAND_UP) → 等 motion_enabled==True 才放行,
含重试, 耗尽则 abort_motion 落 MOTION_FAILED (狗已泄力无法泊出, 必须
显式失败而不是干等超时)。

降级策略: 桥不存在 (无 l1w_control 的机器狗/纯台架) 时, service_wait_sec
宽限后序列落 FAILED 并告警一次 —— DOCKED 是成功终态, 收尾失败绝不回写
错误状态、绝不卡死控制循环。
"""

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


class ChargeMode:
    """DOCKED 后的 静止(锁定)→阻尼 序列, 每个控制 tick 轮询推进。

    状态:
      IDLE        未启动 (等 DOCKED 入口 begin())
      STATIC      static_stand 已发, 等 posture_state 确认
      DAMPING     passive 已发, 等 settle 时长
      DONE        请求受理且等待结束 (非硬件阻尼/充电电流确认)
      RECOVERING  泊出恢复中: stand_up 已发, 等 motion_enabled==True
      FAILED      某步重试耗尽/无桥宽限超时 (仅告警, 不影响 DOCKED 成功)
      DISABLED    charge.enable=false
    """

    IDLE, STATIC, SITTING, DAMPING, DONE, RECOVERING, FAILED, DISABLED = range(8)
    _PHASE_NAMES = {
        IDLE: 'IDLE', STATIC: 'STATIC', SITTING: 'SITTING',
        DAMPING: 'DAMPING', DONE: 'DONE', RECOVERING: 'RECOVERING',
        FAILED: 'FAILED', DISABLED: 'DISABLED',
    }

    def __init__(self, node):
        self._node = node
        self._sm = node._sm

        self._enable = bool(node._p('charge.enable'))
        self._static_ok = self._enable and bool(node._p('charge.static_stand'))
        if self._enable and not self._static_ok:
            node.get_logger().info('充电收尾: static_stand 已跳过, DOCKED 后直接阻尼 (passive, 电机泄力)')
        self._static_ack_ns = int(float(node._p('charge.static_ack_timeout_sec')) * 1e9)
        self._lie_settle_ns = int(float(node._p('charge.lie_down_settle_sec')) * 1e9)
        self._damp_settle_ns = int(float(node._p('charge.passive_settle_sec')) * 1e9)
        self._retries = int(node._p('charge.retries'))
        self._service_wait_ns = int(float(node._p('charge.service_wait_sec')) * 1e9)

        self._phase = self.DISABLED if not self._enable else self.IDLE
        if not self._enable:
            node.get_logger().info('充电收尾功能已停用 (charge.enable=false)')

        prefix = node._p('base.l1w_prefix')
        self._cli_static = node.create_client(Trigger, f'{prefix}/static_stand')
        self._cli_lie = node.create_client(Trigger, f'{prefix}/lie_down')
        self._cli_passive = node.create_client(Trigger, f'{prefix}/passive')
        self._cli_stand = node.create_client(Trigger, f'{prefix}/stand_up')

        # 与桥的 latched 发布 (transient_local) 匹配, 启动即收到当前快照
        # (与 posture_mode 同一套 QoS 约定)。
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._posture_sub = node.create_subscription(
            String, f'{prefix}/posture_state', self._on_posture_state, qos)
        self._motion_sub = node.create_subscription(
            Bool, f'{prefix}/motion_enabled', self._on_motion_enabled, qos)

        # 桥状态回传 (None=无回传哨兵, 与 posture_mode 同义)
        self._posture_state: str | None = None
        self._motion_enabled: bool | None = None

        # 序列推进状态
        self._step_req_ns = 0           # 当前步发出时刻 (ack/settle 计时)
        self._step_future = None
        self._step_tries = 0            # 当前步已发送次数 (1 + retries 上限)
        self._begin_ns = 0              # 序列启动时刻 (无桥宽限计时)

    # ── 桥状态回传 ─────────────────────────────────────────────────

    def _on_posture_state(self, msg: String):
        self._posture_state = msg.data

    def _on_motion_enabled(self, msg: Bool):
        self._motion_enabled = bool(msg.data)

    # ── 对外 API ───────────────────────────────────────────────────

    def begin(self, now_ns: int) -> None:
        """DOCKED 入口: 启动充电收尾序列 (幂等; FAILED 不自动重来)。"""
        if self._phase == self.DISABLED:
            return
        if self._phase != self.IDLE:
            return
        self._begin_ns = now_ns
        if self._static_ok:
            self._enter_static(now_ns)
        else:
            # 跳过静止锁定, 直接阻尼 (charge.static_stand=false)
            self._node.get_logger().info('充电收尾: 跳过静止站立 → 直接请求阻尼 (passive, 电机泄力)')
            self._enter_damping(now_ns)

    def tick(self, now_ns: int) -> None:
        """DOCKED 态每 tick 推进序列 (非阻塞)。"""
        if self._phase in (self.DISABLED, self.IDLE,
                           self.DONE, self.FAILED, self.RECOVERING):
            return

        # 无桥宽限: 服务始终没出现 → 降级告警, DOCKED 保持成功
        if not self._services_ready() and now_ns - self._begin_ns > self._service_wait_ns:
            self._fail('l1w_control 模式服务不可用, 充电收尾中止 (狗保持当前姿态)')
            return

        if self._phase == self.STATIC:
            if self._posture_state == 'static_stand':
                # 已锁定 → 切阻尼; 不趴下 (保持站立泄力)
                self._node.get_logger().info('充电收尾: 已静止站立 → 请求阻尼 (passive, 电机泄力)')
                self._enter_damping(now_ns)
                return
            self._check_step(now_ns, on_ack=self._enter_damping,
                             on_fail=self._fail_static)

        elif self._phase == self.DAMPING:
            if self._step_future is None:
                return
            if not self._step_future.done():
                if now_ns - self._step_req_ns > self._static_ack_ns:
                    self._step_future.cancel()
                    self._fail('passive response timeout; hardware damping unconfirmed')
                return
            try:
                result = self._step_future.result()
            except Exception as exc:
                self._fail('passive response exception: ' + str(exc))
                return
            if result is None or not result.success:
                self._fail('passive request rejected; hardware damping unconfirmed')
                return
            if self._passive_accepted_ns is None:
                self._passive_accepted_ns = now_ns
                self._node.get_logger().info('passive request accepted (not hardware or charging-current feedback)')
            if now_ns - self._passive_accepted_ns >= self._damp_settle_ns:
                self._phase = self.DONE
                self._node.get_logger().info('passive accepted and settling elapsed; verify damping / charge current externally')

    def motion_ready(self, now_ns: int) -> bool:
        """True → 泊出盲退可以发车 (狗不在锁定/阻尼门控态)。

        _run_undock Case 2 每 tick 调用, 与 posture.motion_ready 串联。
        收尾进行中收到泊出请求 → 立即中断序列转 stand_up 恢复。
        """
        if not self._gated():
            return True

        if self._phase != self.RECOVERING:
            self._begin_recover(now_ns)
            return False

        # RECOVERING: 等 motion_enabled 变 True; 超时重发, 耗尽则中止泊出。
        if now_ns - self._step_req_ns > self._static_ack_ns:
            if self._step_tries <= self._retries:
                self._send_stand(now_ns)
                return False
            self._sm.abort_motion('stand_up 超时, 无法从锁定/阻尼恢复运动模式')
            self._phase = self.FAILED
            return False
        return False

    def reset(self) -> None:
        """新一轮 dock 开始/泊出完成: 回 IDLE, 下次 DOCKED 重新收尾。"""
        if self._phase == self.DISABLED:
            return
        self._phase = self.IDLE
        self._step_tries = 0
        self._step_future = None

    # ── 诊断 ───────────────────────────────────────────────────────

    @property
    def phase_name(self) -> str:
        return self._PHASE_NAMES[self._phase]

    # ── 内部: 序列步进 ─────────────────────────────────────────────

    def _enter_static(self, now_ns: int) -> None:
        self._node.get_logger().info('充电收尾: 请求静止站立 (static_stand)')
        self._phase = self.STATIC
        self._step_req_ns = now_ns
        self._step_tries = 1
        self._step_future = self._cli_static.call_async(Trigger.Request())

    def _enter_sitting(self, now_ns: int) -> None:
        """(已弃用) 趴下步 — 当前流程不再使用 lie_down, 保留仅为兼容旧代码路径。"""
        self._phase = self.SITTING
        self._step_req_ns = now_ns
        self._step_tries = 1
        self._step_future = self._cli_lie.call_async(Trigger.Request())

    def _enter_damping(self, now_ns: int) -> None:
        if not bool(self._node._p('charge.passive')):
            self._phase = self.DONE
            self._node.get_logger().info('passive disabled: no damping request sent')
            return
        self._passive_accepted_ns = None
        self._phase = self.DAMPING
        self._step_req_ns = now_ns
        self._step_tries = 1
        try:
            self._step_future = self._cli_passive.call_async(Trigger.Request())
        except Exception as exc:
            self._fail('passive request exception: ' + str(exc))

    def _check_step(self, now_ns: int, on_ack, on_fail) -> None:
        """STATIC 步专用: 确认到达 → on_ack; 服务失败/超时 → 重试或 on_fail。

        阻尼无正回传 (posture_state 只有 not_standing), 走纯定时,
        因此只有 STATIC 需要 ack 检查。
        """
        # 服务返回失败 (桥明确拒绝) → 重试/失败
        if self._step_future is not None and self._step_future.done():
            try:
                result = self._step_future.result()
            except Exception:
                result = None
            if result is None or not result.success:
                on_fail(now_ns, 'static_stand 服务返回失败')
                return
            # 桥已受理; posture_state 确认可能滞后, 继续等 ack
        if now_ns - self._step_req_ns > self._static_ack_ns:
            if self._step_tries <= self._retries:
                self._step_tries += 1
                self._step_req_ns = now_ns
                self._step_future = self._cli_static.call_async(Trigger.Request())
                self._node.get_logger().warn(
                    f'static_stand 未确认, 重试 ({self._step_tries}/'
                    f'{self._retries + 1})')
                return
            on_fail(now_ns, 'static_stand 确认超时')

    def _fail_static(self, now_ns: int, reason: str) -> None:
        # STATIC 失败时狗还站着 —— 仍尝试切阻尼, 狗站立泄力比完全放弃更接近目标。
        self._node.get_logger().warn(
            f'充电收尾: {reason}, 静止步失败 → 仍尝试阻尼 (passive)')
        self._enter_damping(now_ns)

    def _fail(self, msg: str) -> None:
        self._phase = self.FAILED
        self._node.get_logger().warn(msg)

    # ── 内部: 泊出恢复 ─────────────────────────────────────────────

    def _gated(self) -> bool:
        """狗的 cmd_vel 是否可能被桥门控 (锁定/阻尼过)。"""
        if self._motion_enabled is True:
            return False
        if self._motion_enabled is None:
            # 无状态回传 = 桥多半不存在, 没有 gate 可言
            return False
        if self._posture_state == 'static_stand':
            return True
        return self._phase in (self.STATIC, self.DAMPING, self.DONE)

    def _begin_recover(self, now_ns: int) -> None:
        if self._phase in (self.DAMPING, self.DONE):
            self._node.get_logger().info(
                f'泊出请求 → 中断充电收尾 ({self.phase_name}), '
                f'恢复运动模式 (stand_up)')
        self._phase = self.RECOVERING
        self._send_stand(now_ns)

    def _send_stand(self, now_ns: int) -> None:
        self._step_tries += 1
        self._step_req_ns = now_ns
        self._step_future = self._cli_stand.call_async(Trigger.Request())

    # ── 内部 ───────────────────────────────────────────────────────

    def _services_ready(self) -> bool:
        if self._phase == self.STATIC:
            return self._cli_static.service_is_ready()
        if self._phase == self.DAMPING:
            return self._cli_passive.service_is_ready()
        return True