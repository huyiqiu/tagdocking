"""静止站立(锁定)模式管理器 — 走停对接 × 呼吸抑制的核心。

狗的站立步态会"呼吸"(身体持续微幅起伏), 机器运动期间从不相信相机、静止时
从不相信里程计 —— 但"静止站立"必须真正静止, tag 位姿才可信。本管理器把每个
停-看循环的"停"升级为完整闭环:

    停稳 → static_stand (CMD_LOCK_MODE, 身体锁定不喘)
         → 等 posture_state==static_stand 且距停稳 >= static_settle_sec
         → [调用方] 解冻量测 + 规划 (锁定下进行)
         → stand_up 恢复运动模式 → 等 motion_enabled==True
         → [调用方] 起步 (盲链子步之间不锁)

所有调用全部非阻塞 (只用 call_async, 由 20Hz 控制循环轮询结果), 绝不在定时器
里 sleep / spin_until_future_complete。

接口契约 (zsibot_l1_control, 节点名 l1w_control, 无需改 C++):
  ~/static_stand (Trigger)  → CMD_LOCK_MODE; 身体锁定, 但 cmd_vel 会被桥拒绝
  ~/stand_up    (Trigger)  → CMD_STAND_UP; 恢复运动, 重新使能 cmd_vel (唯一出路)
  ~/posture_state  (String, transient local) → static_stand/standing/not_standing/unavailable
  ~/motion_enabled (Bool,   transient local) → 恰好就是桥的 cmd_vel 门控
      (authority && motion_enabled_ && !static_stand), 解锁确认以此为准。

降级策略: 桥不存在 (备用桥 zsibot_bridge / 纯台架) 时, service_wait_sec 宽限后
自动 DISABLED 并告警一次, 行为退回"只有感知侧 settle"的现状; static_stand 确认
超时则本次停降级为未锁定继续。解锁 (stand_up) 失败不可降级 —— 狗还锁着 cmd_vel
发不出去 —— 重试耗尽后 abort_motion() 直接落 MOTION_FAILED (不能走 fail():
RETRYING→RETRYING 是同态 no-op, 会卡死在倒车步)。

运行期开关: posture.enable 每个停-看边界实时读参 (不缓存), 不是所有机器狗都
带 l1w_control 提供这套接口, 没有的狗 ros2 param set 关掉即可, 免重启免宽限。
中途关闭时若狗正锁着, motion_ready/release 仍会走完解锁流程放出 cmd_vel 门控,
绝不把一只锁死的狗扔在盲走里。
"""

from rclpy.qos import (QoSProfile, ReliabilityPolicy, DurabilityPolicy)
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


class PostureMode:
    """共享的 静止站立(锁定)/运动模式 状态机, 每个停-看循环调用一次。

    状态:
      IDLE           未锁定, 运动模式可用
      LOCKING        static_stand 已发出, 等 posture_state 确认
      LOCKED         已确认锁定 (量测窗口, tag 位姿最稳)
      LOCK_DEGRADED  锁定请求失败/超时, 本次停降级为未锁定继续
      UNLOCKING      stand_up 已发出, 等 motion_enabled==True
      UNLOCK_FAILED  stand_up 重试耗尽 (已 abort_motion), 等下一停重新武装
      DISABLED       功能停用 (posture.enable=false 或运行期发现无桥)
    """

    IDLE, LOCKING, LOCKED, LOCK_DEGRADED, UNLOCKING, UNLOCK_FAILED, DISABLED = range(7)
    _PHASE_NAMES = {
        IDLE: 'IDLE', LOCKING: 'LOCKING', LOCKED: 'LOCKED',
        LOCK_DEGRADED: 'LOCK_DEGRADED', UNLOCKING: 'UNLOCKING',
        UNLOCK_FAILED: 'UNLOCK_FAILED', DISABLED: 'DISABLED',
    }

    def __init__(self, node):
        self._node = node
        self._sm = node._sm

        self._static_settle_ns = int(float(node._p('posture.static_settle_sec')) * 1e9)
        self._lock_ack_ns = int(float(node._p('posture.lock_ack_timeout_sec')) * 1e9)
        self._unlock_ack_ns = int(float(node._p('posture.unlock_ack_timeout_sec')) * 1e9)
        self._unlock_retries = int(node._p('posture.unlock_retries'))
        self._service_wait_ns = int(float(node._p('posture.service_wait_sec')) * 1e9)

        self._phase = self.IDLE if self._enabled() else self.DISABLED
        if self._phase == self.DISABLED:
            node.get_logger().info('静止站立功能已停用 (posture.enable=false)')

        prefix = node._p('base.l1w_prefix')
        self._cli_lock = node.create_client(Trigger, f'{prefix}/static_stand')
        self._cli_unlock = node.create_client(Trigger, f'{prefix}/stand_up')

        # 与桥的 latched 发布 (rclcpp::QoS(1).transient_local) 匹配: 订阅端
        # 必须也是 transient local 才能在启动瞬间收到当前状态快照, 否则要等
        # 桥的 1Hz 心跳重发。
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._posture_sub = node.create_subscription(
            String, f'{prefix}/posture_state', self._on_posture_state, qos)
        self._motion_sub = node.create_subscription(
            Bool, f'{prefix}/motion_enabled', self._on_motion_enabled, qos)

        # 桥状态回传
        self._posture_state: str | None = None
        self._motion_enabled: bool | None = None

        # per-stop 锁定状态
        self._stop_ns = 0               # 本次停稳时刻 (settle 时钟起点)
        self._lock_req_ns = 0
        self._lock_future = None
        self._lock_requested = False    # 本停是否已请求锁定 (外部锁定判定用)
        self._lock_phase_resolved = False   # per-stop 锁存: 解锁期间保持绿

        # per-stop 解锁状态
        self._unlock_req_ns = 0
        self._unlock_future = None
        self._unlock_tries = 0          # stand_up 已发送次数 (1 + unlock_retries 上限)

        # 无桥检测: motion_enabled 从未到达的宽限计时
        # (None=未使用哨兵; 不能用 0 —— sim time/假时钟的首次 now_ns 可能就是 0)
        self._first_use_ns: int | None = None

    # ── 桥状态回传 ─────────────────────────────────────────────────

    def _on_posture_state(self, msg: String):
        self._posture_state = msg.data

    def _on_motion_enabled(self, msg: Bool):
        self._motion_enabled = bool(msg.data)

    # ── 对外 API (停-看循环只用这四个入口) ─────────────────────────

    def _enabled(self) -> bool:
        """posture.enable 实时值 — 运行期可调 (不缓存)。

        不是所有机器狗都带 l1w_control (whale-nav 栈) 提供这套锁定接口:
        没有接口的狗随时 ``ros2 param set <节点> posture.enable false`` 即可
        整体关掉呼吸抑制, 免重启、免无桥宽限等待。每个停-看边界重新读一次,
        开关即时生效 (含中途切换的状态迁移, 见各入口的守卫)。
        """
        return bool(self._node._p('posture.enable'))

    def on_stop(self, now_ns: int) -> None:
        """停稳边界: 武装本次停的锁定请求 (每停一次, 重复调用幂等)。"""
        if not self._enabled():
            return
        if self._phase == self.DISABLED:
            # 运行期重新开启 (或无桥停用后恢复): 回 IDLE 重新武装。
            # 若接口其实仍不存在, 会在 service_wait_sec 宽限后再次自动停用。
            self._phase = self.IDLE
        if self._phase in (self.IDLE, self.UNLOCK_FAILED, self.LOCK_DEGRADED):
            # 新的一次停稳: 重置 settle 时钟 (上次降级不代表这次也降级, 桥可能
            # 已恢复, 每停都重新尝试)。已锁定 (posture_state 已是 static_stand)
            # 则直接幂等复锁, 不重发服务请求。
            self._stop_ns = now_ns
            self._lock_phase_resolved = False
            if self._posture_state == 'static_stand':
                self._phase = self.LOCKED
                self._lock_phase_resolved = True
            else:
                self._phase = self.LOCKING
                self._lock_req_ns = now_ns
                self._lock_requested = True
                self._lock_future = self._cli_lock.call_async(Trigger.Request())
                self._node.get_logger().info('停稳 → 请求静止站立 (锁定, 抑制呼吸)')
        # LOCKING/LOCK_DEGRADED/UNLOCKING: 请求在途或已降级, 不重复发。

    def lock_settled(self, now_ns: int) -> bool:
        """True → 允许进入量测窗口 (已锁定且停振窗满, 或已降级/停用)。

        自武装: 若没有任何停稳边界触发过 (如 SEARCH_TAG → APPROACH 直接
        交接、executor 从未 mark_stop_time), 在此补发 static_stand 并开始
        settle 计时, 保证首次量测也不在呼吸中取样。
        """
        if not self._enabled():
            return True
        if self._phase == self.DISABLED:
            return True
        if self._phase == self.IDLE:
            self.on_stop(now_ns)
        if self._phase == self.LOCKING:
            if self._posture_state == 'static_stand':
                self._phase = self.LOCKED
                self._lock_phase_resolved = True
                self._node.get_logger().info('静止站立已锁定 (呼吸抑制生效)')
            elif self._lock_future is not None and self._lock_future.done():
                try:
                    result = self._lock_future.result()
                except Exception:
                    result = None
                if result is None or not result.success:
                    self._phase = self.LOCK_DEGRADED
                    self._lock_phase_resolved = True
                    self._warn_degraded('static_stand 服务返回失败, 降级为未锁定继续')
                elif now_ns - self._lock_req_ns > self._lock_ack_ns:
                    self._remove_pending(self._lock_future)
                    self._lock_future = None
                    self._phase = self.LOCK_DEGRADED
                    self._lock_phase_resolved = True
                    self._warn_degraded(
                        'static_stand 已受理但 posture_state 未确认, 降级为未锁定继续')
            elif now_ns - self._lock_req_ns > self._lock_ack_ns:
                self._remove_pending(self._lock_future)
                self._lock_future = None
                self._phase = self.LOCK_DEGRADED
                self._lock_phase_resolved = True
                self._warn_degraded('static_stand 确认超时, 降级为未锁定继续')
            elif (not self._cli_lock.service_is_ready()
                  and now_ns - self._lock_req_ns > self._service_wait_ns):
                self._disable('l1w_control 模式服务不可用, 本次会话停用静止站立')
                return True
        # LOCKED / LOCK_DEGRADED / UNLOCKING → _lock_phase_resolved 已置位
        return (self._lock_phase_resolved
                and (now_ns - self._stop_ns) >= self._static_settle_ns)

    def motion_ready(self, now_ns: int) -> bool:
        """True → cmd_vel 已确认会被桥接受 (必要时不阻塞地走 stand_up 流程)。

        确认依据是 /motion_enabled (它就是桥的 cmd_vel 门控本身), 不猜内部
        标志 —— 固件退出 LOCK 模式有滞后, 只有 Bool 变 True 才真的能走。
        """
        if self._phase == self.DISABLED:
            return True
        if not self._enabled():
            # 运行期关闭: 若狗还锁着 (cmd_vel 被门控拒绝), 必须先走解锁流程
            # 把它放出来 —— 直接放行会让后续盲走全部发不出去、里程计不动,
            # 被误判成运动卡死。没锁 (或早已解锁) 就等同于 DISABLED 放行。
            if (self._posture_state != 'static_stand'
                    and self._phase not in (self.LOCKING, self.LOCKED,
                                            self.UNLOCKING)):
                return True
            # 狗还锁着 → 落入下方常规解锁流程
        if self._motion_enabled is True:
            # 已确认可运动 (覆盖"从未锁定"与"解锁完成"两种情况)
            self._phase = self.IDLE
            self._lock_requested = False
            return True

        # 状态从未到达 → 桥多半不存在 (备用桥/纯台架): 宽限后停用功能,
        # 让对接行为退回现状而不是卡死。
        if self._motion_enabled is None:
            if self._first_use_ns is None:
                self._first_use_ns = now_ns
            if (now_ns - self._first_use_ns > self._service_wait_ns
                    and not self._cli_unlock.service_is_ready()):
                self._disable('l1w_control 模式服务不可用, 本次会话停用静止站立')
                return True

        if self._phase != self.UNLOCKING:
            self._begin_unlock(now_ns)
            return False

        # UNLOCKING: 等 motion_enabled 变 True; 超时重发, 耗尽则中止。
        if now_ns - self._unlock_req_ns > self._unlock_ack_ns:
            if self._unlock_tries <= self._unlock_retries:
                self._send_unlock(now_ns)
                return False
            if self._motion_enabled is None:
                # 有服务但始终无状态回传: 无法确认, 停用而非中止 (与无桥同待遇)
                self._disable('无法确认运动模式 (状态话题无回传), 本次会话停用静止站立')
                return True
            self._sm.abort_motion('stand_up 超时, 无法恢复运动模式')
            self._phase = self.UNLOCK_FAILED
            return False
        return False

    def release(self, now_ns: int, reason: str = '') -> None:
        """终态善后: 尽力恢复运动模式 (fire-and-forget, 绝不阻塞/中止)。

        DOCKED 不调用本方法 (按约定保持锁定); 取消/超时/失败等错误终态
        由 _control_loop 的过渡规则调用, 把狗还给遥控。
        """
        if self._phase == self.DISABLED:
            return
        # 运行期关闭也不拦终态善后: 狗真锁着 (posture_state 残留 static_stand)
        # 仍要尽力放出, 别把一只锁死的狗还给遥控。
        if not self._enabled() and self._posture_state != 'static_stand':
            return
        if self._motion_enabled is True:
            return
        if not self._cli_unlock.service_is_ready():
            return
        if reason:
            self._node.get_logger().info(f'{reason} → 恢复运动模式 (stand_up)')
        self._phase = self.UNLOCKING
        self._unlock_req_ns = now_ns
        self._unlock_tries = 1
        self._unlock_future = self._cli_unlock.call_async(Trigger.Request())

    # ── 诊断 ───────────────────────────────────────────────────────

    @property
    def locked(self) -> bool:
        """桥侧当前是否处于静止站立 (posture_state 回传)。"""
        return self._posture_state == 'static_stand'

    @property
    def external_lock(self) -> bool:
        """非本节点请求的外部锁定 (如网页台"静止站立"按钮)。

        必须同时要求 motion_enabled is False: 本节点解锁刚完成时
        posture_state 可能还残留 static_stand (桥 1Hz 事件发布滞后),
        只看姿态会在起步瞬间误判。真正的外部锁定必然使能门控关闭。
        """
        return (self._phase not in (self.LOCKING, self.LOCKED, self.UNLOCKING)
                and self._posture_state == 'static_stand'
                and self._motion_enabled is False)

    # ── 内部 ───────────────────────────────────────────────────────

    def _begin_unlock(self, now_ns: int) -> None:
        self._phase = self.UNLOCKING
        self._unlock_req_ns = now_ns
        self._unlock_tries = 1
        self._unlock_future = self._cli_unlock.call_async(Trigger.Request())
        self._node.get_logger().info('静止站立 → 请求恢复运动模式 (stand_up)')

    def _send_unlock(self, now_ns: int) -> None:
        self._unlock_tries += 1
        self._unlock_req_ns = now_ns
        self._unlock_future = self._cli_unlock.call_async(Trigger.Request())
        self._node.get_logger().warn(
            f'stand_up 未确认, 重试 ({self._unlock_tries}/'
            f'{self._unlock_retries + 1})')

    def _disable(self, reason: str) -> None:
        if self._phase == self.DISABLED:
            return
        self._phase = self.DISABLED
        self._node.get_logger().warn(f'{reason}')

    def _warn_degraded(self, msg: str) -> None:
        self._node.get_logger().warn(msg, throttle_duration_sec=10.0)

    @staticmethod
    def _remove_pending(future) -> None:
        """放弃未完成的 service future, 避免悬挂回调。"""
        if future is None:
            return
        try:
            if not future.done():
                future.cancel()
        except Exception:
            pass
