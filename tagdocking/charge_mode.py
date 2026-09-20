"""充电收尾 (DOCKED 后) 模式序列管理器 — 静止(锁定) → 阻尼。

对接成功后狗站在充电桩上方, 锁定后直接切阻尼泄力, 保持站立 (不趴下):

    static_stand (CMD_LOCK_MODE, 静止锁定)
      → 等 posture_state==static_stand
    passive (CMD_EMERGENCY_STOP, 电机泄力 = 本 SDK 的"阻尼")
      → 等服务成功响应，再计 passive_settle_sec（不等于硬件或电流反馈）
      (charge.passive=false 时跳过本步, 收尾止于锁定)
      (charge.static_stand=false 时跳过锁定, 直接阻尼)
    → DONE

与停-看循环的量测稳定 (稳定帧门) 互不相干: 那套管"停-看循环里的
量测稳定", 充电收尾管的是"泊完后把狗放好"。

所有调用全部非阻塞 (只用 call_async, 由 20Hz 控制循环在相应状态轮询
tick 推进), 绝不在回调里 sleep / spin_until_future_complete。

运动衔接: 狗处于锁定/阻尼态时 cmd_vel 被桥门控 (motion_enabled=False),
停泊与泊出开始前都由控制循环调 motion_ready(): stand_up (CMD_STAND_UP)
→ 等 motion_enabled==True 才放行。恢复全程非阻塞、含重试；耗尽则
abort_motion 落 MOTION_FAILED，避免速度白发后误报里程计/视觉故障。

门控判据只信桥 latched 的 motion_enabled, 不看本进程的 _phase —— 栈是
按需启停的 (supervisor 到终态 30s 后收栈), 泊出那一下往往由一个全新的
docking_node 发起, 它没参与过泊入, _phase 是 IDLE。桥 (zsibot_l1_control)
是独立常驻节点, 活得比栈久, transient_local 让新进程一订阅就拿到真相。

降级策略: 桥不存在 (无 l1w_control 的机器狗/纯台架) 时, service_wait_sec
宽限后序列落 FAILED 并告警一次 —— DOCKED 是成功终态, 收尾失败绝不回写
错误状态、绝不卡死控制循环。
"""

import subprocess
import threading

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from .state_machine import CODE_MOTION_GATED


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

    IDLE, STATIC, DAMPING, DONE, RECOVERING, FAILED, DISABLED = range(7)
    _PHASE_NAMES = {
        IDLE: 'IDLE', STATIC: 'STATIC',
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
        self._damp_settle_ns = int(float(node._p('charge.passive_settle_sec')) * 1e9)
        self._retries = int(node._p('charge.retries'))
        self._service_wait_ns = int(float(node._p('charge.service_wait_sec')) * 1e9)

        # 充电桩使能/断电 (狗控制器 firefly 上的 XG 充电桩脚本, 经 SSH 一次性触发)。
        # 这套与上面的"充电收尾姿态序列"完全正交: 收尾管 l1w_control 的锁定/阻尼
        # 姿态, 这里管充电桩极片带不带电。充电状态锁存在桩子 MCU 一侧 (发一次
        # dog_lying_down 即进入充电, 发一次 dog_status_unknown 即断电), 故只需
        # 一次性把命令送达, 无需常驻 —— 用远端 timeout 兜住不自退出的厂商二进制。
        self._pile_enable = bool(node._p('charge.pile.enable'))
        self._pile_target = str(node._p('charge.pile.ssh_target'))
        self._pile_dir = str(node._p('charge.pile.dir')).rstrip('/')
        self._pile_on_bin = str(node._p('charge.pile.enable_bin'))
        self._pile_off_bin = str(node._p('charge.pile.disable_bin'))
        self._pile_run_timeout = float(node._p('charge.pile.run_timeout_sec'))
        self._pile_ssh_ctimeout = int(float(node._p('charge.pile.ssh_connect_timeout_sec')))
        if not self._pile_enable:
            node.get_logger().info('充电桩联动已停用 (charge.pile.enable=false)')

        self._phase = self.DISABLED if not self._enable else self.IDLE
        if not self._enable:
            node.get_logger().info('充电收尾功能已停用 (charge.enable=false)')

        prefix = node._p('base.l1w_prefix')
        self._cli_static = node.create_client(Trigger, f'{prefix}/static_stand')
        self._cli_passive = node.create_client(Trigger, f'{prefix}/passive')
        self._cli_stand = node.create_client(Trigger, f'{prefix}/stand_up')

        # 与桥的 latched 发布 (transient_local) 匹配, 启动即收到当前快照。
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._posture_sub = node.create_subscription(
            String, f'{prefix}/posture_state', self._on_posture_state, qos)
        self._motion_sub = node.create_subscription(
            Bool, f'{prefix}/motion_enabled', self._on_motion_enabled, qos)

        # 桥状态回传 (None=无回传哨兵)
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

    def motion_ready(self, now_ns: int, operation: str = '泊出') -> bool:
        """True → 停泊/泊出可以发车 (狗不在锁定/阻尼门控态)。

        控制循环每 tick 调用；门控时异步请求 stand_up，等待桥发布
        motion_enabled=True 后才放行。operation 仅用于现场诊断文案。
        """
        if not self._gated():
            if self._phase == self.RECOVERING:
                self._phase = self.IDLE if self._enable else self.DISABLED
                self._step_tries = 0
                self._step_future = None
                self._node.get_logger().info(
                    f'{operation}准备: motion_enabled=True, 运动模式已恢复')
            return True

        if self._phase != self.RECOVERING:
            self._begin_recover(now_ns, operation)
            return False

        # RECOVERING: 等 motion_enabled 变 True; 超时重发, 耗尽则中止本次任务。
        if now_ns - self._step_req_ns > self._static_ack_ns:
            if self._step_tries <= self._retries:
                self._send_stand(now_ns)
                return False
            self._sm.abort_motion(
                f'{operation}前 stand_up 超时, 无法从锁定/阻尼恢复运动模式 '
                f'(重发 {self._step_tries} 次 × {self._static_ack_ns * 1e-9:.1f}s, '
                f'motion_enabled={self._motion_enabled} '
                f'posture_state={self._posture_state!r})',
                CODE_MOTION_GATED)
            self._phase = self.FAILED
            return False
        return False

    def reset(self) -> None:
        """新一轮 dock 开始/泊出完成: 回 IDLE, 下次 DOCKED 重新收尾。

        charge.enable=false 时回 DISABLED 而不是 IDLE —— 泊出恢复
        (_begin_recover) 会把 _phase 写成 RECOVERING, 把"收尾已停用"这件事
        冲掉; 若这里再回 IDLE, 下一次 begin() 就会真去跑收尾。以 _enable 为准。
        """
        self._phase = self.IDLE if self._enable else self.DISABLED
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

    # ── 内部: 运动模式恢复 ─────────────────────────────────────────

    def _gated(self) -> bool:
        """狗的 cmd_vel 是否被桥门控 (锁定/阻尼过)。

        判据只看 /motion_enabled —— 它就是桥的 cmd_vel 门控本身,
        且 transient_local latched,
        新进程一订阅就拿到当前快照。

        原先末行还要求 _phase in (STATIC, DAMPING, DONE): 那是进程内记忆。
        supervisor 按需起栈后 docking_node 是全新进程 (_phase=IDLE), 狗却还
        泊在上一个进程留下的 passive 泄力态 —— 门控看不见, stand_up 不发,
        盲退白发, 里程计不动, 30s 后落 MOTION_FAILED。桥活得比栈久, 以它为准。
        (posture_state=='static_stand' 那条也被覆盖: 锁定态同样 motion_enabled=False。)
        """
        if self._motion_enabled is True:
            return False
        if self._motion_enabled is None:
            # 无状态回传 = 桥多半不存在, 没有 gate 可言
            return False
        return True

    def _begin_recover(self, now_ns: int, operation: str) -> None:
        # stand_up 要拿满自己的重试预算: _step_tries 可能残留 static/passive
        # 那一步的计数 (=1), 不清零会让首次 ack 超时就直接 abort ——
        # charge.retries 形同 0, 与模块头写的"含重试"不符。
        self._step_tries = 0
        # 无条件打印: 按需起栈后 _phase 是 IDLE/DISABLED 也会走恢复,
        # phase_name 正好自述"这次是从哪个相位被打断的"。原先只在
        # DAMPING/DONE 打印, 重启路径会静默恢复、留不下证据。
        self._node.get_logger().info(
            f'{operation}请求 → 中断充电收尾 ({self.phase_name}), '
            f'恢复运动模式 (stand_up)')
        self._phase = self.RECOVERING
        self._send_stand(now_ns)

    def _send_stand(self, now_ns: int) -> None:
        self._step_tries += 1
        self._step_req_ns = now_ns
        self._step_future = self._cli_stand.call_async(Trigger.Request())

    # ── 充电桩使能/断电 (SSH 一次性, 非阻塞) ───────────────────────

    def pile_charge_on(self) -> None:
        """DOCKED 入口: 触发充电桩使能 (dog_lying_down)。一次性、非阻塞。

        桩子收到 DOG_LYING_DOWN + 极片接触良好即自动进入充电并锁存, 无需保活。
        """
        self._run_pile_cmd(self._pile_on_bin, '使能充电 (dog_lying_down)')

    def pile_charge_off(self) -> None:
        """泊出前: 触发充电桩断电 (dog_status_unknown)。一次性、非阻塞。

        桩子收到 DOG_STATUS_UNKNOWN 即给极片断电并锁存。
        """
        self._run_pile_cmd(self._pile_off_bin, '关闭充电 (dog_status_unknown)')

    def _run_pile_cmd(self, bin_name: str, label: str) -> None:
        if not self._pile_enable:
            return
        # 厂商二进制是 while(1) 监控循环、永不自退出; 远端 timeout 到时发信号
        # 结束它 —— 充电状态已在桩端锁存, 进程被杀不影响充放电。
        remote = (f'sudo timeout {self._pile_run_timeout:g} '
                  f'{self._pile_dir}/{bin_name} '
                  f'> /tmp/charge_pile_last.log 2>&1')
        argv = ['ssh', '-o', 'BatchMode=yes', '-o', 'StrictHostKeyChecking=no',
                '-o', f'ConnectTimeout={self._pile_ssh_ctimeout}',
                self._pile_target, remote]
        self._node.get_logger().info(f'充电桩: {label} → ssh {self._pile_target}')
        try:
            proc = subprocess.Popen(
                argv, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True)
        except Exception as exc:
            self._node.get_logger().warn(f'充电桩: {label} 启动失败: {exc}')
            return
        # 后台收尸只为记日志, 绝不阻塞 20Hz 控制循环。
        threading.Thread(
            target=self._reap_pile, args=(proc, label), daemon=True).start()

    def _reap_pile(self, proc: 'subprocess.Popen', label: str) -> None:
        wait_s = self._pile_run_timeout + self._pile_ssh_ctimeout + 10
        try:
            _, err = proc.communicate(timeout=wait_s)
        except Exception as exc:
            proc.kill()
            self._node.get_logger().warn(f'充电桩: {label} 等待异常, 已终止本地 ssh: {exc}')
            return
        rc = proc.returncode
        # 124=远端 timeout 到时结束 (厂商二进制永不自退出, 这是正常结局);
        # 137/143=SIGKILL/SIGTERM 收尾, 同属预期。其余 (如 255=SSH 连不上) 才告警。
        if rc in (0, 124, 137, 143):
            self._node.get_logger().info(f'充电桩: {label} 已下发 (exit={rc})')
        else:
            tail = (err or '').strip().splitlines()[-1:]
            self._node.get_logger().warn(f'充电桩: {label} 失败 exit={rc} {tail}')

    # ── 内部 ───────────────────────────────────────────────────────

    def _services_ready(self) -> bool:
        if self._phase == self.STATIC:
            return self._cli_static.service_is_ready()
        if self._phase == self.DAMPING:
            return self._cli_passive.service_is_ready()
        return True