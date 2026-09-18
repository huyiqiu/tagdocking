"""Docking state machine — full lifecycle management (dual-only).

States:
    IDLE         — waiting for start command
    SEARCH_TAG   — rotating/pausing to find the AprilTag
    APPROACH     — dual-tag optical stop-and-go (双码全权驱动走停闭环)
    DOCKED       — success: dual complete, robot settled on the pile
    UNDOCKING    — pulling out: blind reverse a distance + 180° turn
    UNDOCKED     — success: undock maneuver complete

Error terminal states:
    TIMEOUT      — overall docking timeout exceeded (含搜索超时)
    MOTION_FAILED— commanded motion not reflected in odometry / abort_motion
    CANCELLED    — user cancelled the docking

双码路径里 APPROACH 由 DualTagDocking 全权驱动: 失败一律 abort_motion
直落 MOTION_FAILED (本机曾经的单码 fail()→RETRYING 倒车重试链在双码下
不可达, 已随 v1.0 删除)。APPROACH 的墙码丢失看门狗 (tag_loss_timeout_sec)
在本文件 evaluate() 里, 是该参数唯一的消费者。

失败留档 (对上层应用的契约): 每一条通往错误终态的路都必须记下
reason (人读的现场串) + code (FAILURE_CODES 里的稳定机器码), 统一经
_record_failure() 写入, 由 docking_node 的 ~/outcome 话题发出去。
_transition_to 有一道守卫: 进错误终态却没记账就 WARN 自述, 所以新增
失败路径漏记会自己喊出来, 不会静默给上层一个空原因。
"""

import enum


class DockingState(enum.IntEnum):
    IDLE = 0
    SEARCH_TAG = 2
    APPROACH = 4
    DOCKED = 6
    UNDOCKING = 9         # 泊出(活动态): 盲退一段距离 + 原地转180°
    # Error states
    TIMEOUT = 11
    MOTION_FAILED = 12
    CANCELLED = 13
    UNDOCKED = 14         # 泊出成功(终态)


# States that are considered "active" (not terminal)
_ACTIVE_STATES = {
    DockingState.SEARCH_TAG,
    DockingState.APPROACH,
    DockingState.UNDOCKING,
}

# Terminal success states
_SUCCESS_STATES = {DockingState.DOCKED, DockingState.UNDOCKED}

# Terminal error states
_ERROR_STATES = {
    DockingState.TIMEOUT,
    DockingState.MOTION_FAILED,
    DockingState.CANCELLED,
}


# ── 失败码 (对上层应用的稳定契约) ──────────────────────────────────
#
# 按**上层该怎么办**分族, 不按内部失败点分 —— 上层的 if/else 不会随我们
# 新增一个 abort 点而爆炸。精确的现场留在 reason 串里给人读。
#
# 用 snake_case 字符串而非数字常量: 全仓没有任何 error_code 先例
# (navigo 也不传 nav2 的 error_code), 而小写状态名已经是在用的机器可读
# token (stack_supervisor.py:75-78 / index.html 的状态族都按它), 码与它同构。
CODE_TAG_NOT_FOUND = 'tag_not_found'          # 码不在视野/丢了 → 重新引导进视野
CODE_VISION_NO_PROGRESS = 'vision_no_progress'  # 视觉闭环修不动 → 换起始位姿
CODE_MOTION_GATED = 'motion_gated'            # cmd_vel 被门控(泄力/锁定) → 查桥, 别盲重试
CODE_MOTION_STALLED = 'motion_stalled'        # 底盘不响应指令 → 报警, 人工介入
CODE_TIMEOUT = 'timeout'                      # 整体超时 → 可直接重试
CODE_CANCELLED = 'cancelled'                  # 用户取消 → 不算故障

# 守卫兜底: 进了错误终态却没人记账。**不是**对外承诺的一族 ——
# 它出现就意味着有个 abort 点漏了 code, 见 _transition_to 的 WARN。
CODE_UNSPECIFIED = 'unspecified'

FAILURE_CODES = frozenset({
    CODE_TAG_NOT_FOUND, CODE_VISION_NO_PROGRESS, CODE_MOTION_GATED,
    CODE_MOTION_STALLED, CODE_TIMEOUT, CODE_CANCELLED,
})


class DockingStateMachine:
    """Manages state transitions, timeouts, and docking logic.

    The state machine is EVALUATED (transition decisions) in the control loop
    but ACTUATED (velocity commands) by the main node based on current state.

    Usage:
        sm = DockingStateMachine(params)
        sm.start()
        # In control loop:
        sm.evaluate(tag_pose, tag_visible, odom_data, now_ns)
        state = sm.state
    """

    def __init__(self, node):
        self._node = node
        self._state = DockingState.IDLE
        self._state_start_ns = 0
        self._docking_start_ns = 0

        # Per-state sub-phase tracking
        self._search_tag_hold_start_ns = 0
        # 墙码丢失看门狗的连续未采纳 tick 计数 (evaluate 里累加)。
        self._tag_lost_count = 0
        # 失败理由留档: 过去它只活在一行 error 日志里, 之后无处可取, 通过
        # action 调用的客户端连一个字都拿不到 (只有 'motion_failed')。纯诊断,
        # 没有任何判据读它。
        self._abort_reason = ''
        # 失败码: reason 是给人读的自由串, code 是给上层应用分支用的稳定契约
        # (FAILURE_CODES)。两者同时记, 缺一不可 —— 只有串上层没法可靠分支,
        # 只有码现场无从下手。
        self._abort_code = ''
        # 第几次停泊/泊出。上层靠它区分"这是新一次的结果"还是"上次的残留" ——
        # outcome 话题是锁存的, 不带轮次号就分不出来 (墙上时钟的年龄判不出:
        # 锁存值年龄一直涨, 每 tick 发布则恒为 ~0.05s, 两种都没用)。
        self._run_seq = 0

    # ── Properties ──────────────────────────────────────────────────

    @property
    def state(self) -> DockingState:
        return self._state

    @property
    def state_name(self) -> str:
        return self._state.name

    @property
    def is_active(self) -> bool:
        return self._state in _ACTIVE_STATES

    @property
    def is_terminal(self) -> bool:
        return self._state in _ERROR_STATES or self._state in _SUCCESS_STATES

    @property
    def is_success(self) -> bool:
        return self._state in _SUCCESS_STATES

    @property
    def is_error(self) -> bool:
        return self._state in _ERROR_STATES

    def elapsed_ns(self, now_ns: int) -> int:
        """Nanoseconds since docking started."""
        if self._docking_start_ns == 0:
            return 0
        return now_ns - self._docking_start_ns

    def state_elapsed_ns(self, now_ns: int) -> int:
        """Nanoseconds since current state was entered."""
        if self._state_start_ns == 0:
            return 0
        return now_ns - self._state_start_ns

    # ── Actions ─────────────────────────────────────────────────────

    def start(self):
        """Initiate docking. Transitions straight to SEARCH_TAG.

        Navigation (Nav2 pre-dock) has been removed — an external service is
        expected to bring the robot into tag range before calling start.

        Allowed from any non-active state: IDLE, a success terminal (DOCKED /
        UNDOCKED), or an error terminal. In particular re-docking after an
        undock (UNDOCKED) must be allowed — otherwise a dock→undock→dock cycle
        gets stuck at "无法启动：当前状态=UNDOCKED".
        """
        if self._state not in (DockingState.IDLE, *_SUCCESS_STATES, *_ERROR_STATES):
            self._node.get_logger().warn(f'无法启动：当前状态={self._state.name}')
            return False

        self._begin_run()
        self._transition_to(DockingState.SEARCH_TAG)
        # 每次用户触发都是一次全新的停泊: 重置整体超时起点。
        self._docking_start_ns = self._state_start_ns
        return True

    def start_undock(self):
        """Initiate undocking (pull out): blind reverse + 180° turn.

        Allowed from any non-active state (IDLE / DOCKED / error terminals) —
        typically called after DOCKED. Rejected if a docking sequence is in
        progress. The node drives the blind two-step maneuver; on completion
        it calls finish_undock() → UNDOCKED.
        """
        if self._state in _ACTIVE_STATES:
            self._node.get_logger().warn(
                f'无法泊出：停泊进行中({self._state.name})')
            return False
        self._begin_run()
        self._transition_to(DockingState.UNDOCKING)
        # 泊出是一次独立操作: 重置整体超时起点, 否则 UNDOCKING 也受全局超时
        # (_ACTIVE_STATES) 管辖, 而 _docking_start_ns 还停留在上次停泊的 t0,
        # 隔一阵再触发泊出会立刻 elapsed>timeout_sec 落 TIMEOUT。
        self._docking_start_ns = self._state_start_ns
        return True

    def _begin_run(self) -> None:
        """新一轮停泊/泊出的开场: 轮次号自增 + 上一轮的失败留档清零。

        必须在这里清, 不能只靠 reset() —— reset() 包里没有任何节点调用过
        (只有测试调), 所以上一次的失败原因会一路活到下一次, 被当成这一次的
        原因报给上层。轮次号让锁存的 outcome 能被认出是哪一轮的。
        """
        self._run_seq += 1
        self._abort_reason = ''
        self._abort_code = ''

    def finish_dual(self):
        """Called only after controller-confirmed settled finish and zero/cancel."""
        if self._state == DockingState.APPROACH:
            self._transition_to(DockingState.DOCKED)

    def finish_undock(self):
        """Node calls this when the undock maneuver completed → UNDOCKED."""
        self._transition_to(DockingState.UNDOCKED)

    def cancel(self):
        """User cancel — transition to CANCELLED."""
        # 记账: 取消也是一种终态结果, 上层要能分清"取消"和"真故障"
        # (cancelled 那一族的约定就是"不算故障")。
        self._record_failure('用户取消停泊', CODE_CANCELLED)
        self._transition_to(DockingState.CANCELLED)

    @property
    def failure_reason(self) -> str:
        """最后一次失败的理由 (人读), 供 action 结果与 outcome 话题回传。"""
        return getattr(self, '_abort_reason', '')

    @property
    def failure_code(self) -> str:
        """最后一次失败的码 (机器读, 见 FAILURE_CODES); 未失败时为空串。"""
        return getattr(self, '_abort_code', '')

    @property
    def run_seq(self) -> int:
        """第几次停泊/泊出 (start/start_undock 各自 +1)。"""
        return getattr(self, '_run_seq', 0)

    def _record_failure(self, reason: str, code: str) -> None:
        """失败留档的唯一入口 (reason + code 一起写, 不许只写一半)。"""
        if reason:
            self._abort_reason = reason
        self._abort_code = code or CODE_UNSPECIFIED

    def _wall_loss_reason(self, params) -> str:
        """双码外层丢码看门狗的失败串 —— 开头四个字必须是"墙码丢失"。

        旧串 'dual wall stream lost; no search/reverse' 有两处误导:
        它断言"流断了"(而多数情况下墙码一直看得见, 只是被采纳前的判据否掉),
        且完全不提预算被 dual.settle_sec 停稳窗结构性吃掉一截。现场 2.5s 预算
        里有 1.57s 花在按设计拒收上, 真实宽限只剩 0.93s —— 而 yaml 自己写着
        检测流实测有 1.6~3s 空档。这里把这笔账算给操作员看。

        证据经 params 下发 (字符串或惰性闭包皆可); 没有节点喂证据时 (单测假
        params) 仍须给出可读中文串, 绝不 KeyError、绝不空串。
        """
        budget = params.get('tag', {}).get('tag_loss_timeout_sec', 2.5)
        lost = self._tag_lost_count * 0.05
        head = (f'墙码丢失 — 外层丢码看门狗预算耗尽: 连续 {lost:.2f}s 没有一帧墙码被采纳 '
                f'(> tag.tag_loss_timeout_sec {budget:.2f}s, 20Hz 累计 '
                f'{self._tag_lost_count} tick); 双码路径到此直落 MOTION_FAILED '
                '—— 不退回 SEARCH_TAG、不搜索、不后退 (单码路径才会退回搜索)')
        now_ns = params.get('dual_now_ns') or 0
        settle_until = params.get('dual_settle_until_ns') or 0
        if now_ns and settle_until:
            # 丢码计数只会在 maneuver_active 转假后才开始涨, 而停稳窗正是在
            # 那一 tick 武装的, 所以窗起点 = now - lost 这个估计对现场是准的。
            overlap = max(0.0, (min(now_ns, settle_until) - (now_ns - lost * 1e9)) / 1e9)
            if overlap > 0.05:
                head += (f'; 其中 {overlap:.2f}s 落在停稳静止窗内 '
                         f'(dual.settle_sec {params.get("dual_settle_sec", 1.5):.2f}s, '
                         f'窗内每一帧按设计丢弃) → (c) 真正的丢码宽限只有 '
                         f'{max(0.0, lost - overlap):.2f}s; 先调 tag.tag_loss_timeout_sec '
                         '或缩短 dual.settle_sec, 这一条几乎从不是"检测流断了"')
        evidence = params.get('dual_wall_loss')
        if callable(evidence):
            evidence = evidence()
        return head + '; ' + (evidence
                              or '证据不可得 (节点未提供取证, 只能翻 dual detection <reason> 日志)')

    def abort_motion(self, reason: str = '', code: str = CODE_UNSPECIFIED):
        """系统级失败 — 直接落 MOTION_FAILED, 不重试。

        双码路径唯一的失败入口: 双码下没有"倒车重试"这回事, 视觉闭环修不动
        (vision_no_progress)、墙码丢失 (tag_not_found)、门控 (motion_gated)
        等一律 abort_motion 直落终态, 由上层决定要不要重新触发停泊。

        code 默认 CODE_UNSPECIFIED 而不是某个具体族: 漏传要能被 _transition_to
        的守卫抓出来, 默认成一个像样的码只会把漏传藏起来。
        """
        if reason:
            self._node.get_logger().error(f'运动中止：{reason}')
        self._record_failure(reason, code)
        self._transition_to(DockingState.MOTION_FAILED)

    def reset(self):
        """Full reset to IDLE."""
        self._state = DockingState.IDLE
        self._state_start_ns = 0
        self._docking_start_ns = 0
        self._search_tag_hold_start_ns = 0
        self._tag_lost_count = 0
        self._abort_reason = ''
        self._abort_code = ''

    # ── State machine evaluation ────────────────────────────────────

    def evaluate(self,
                 tag_pose,          # TagPose or None
                 tag_visible: bool,
                 motion_stalled: bool,
                 now_ns: int,
                 params: dict,
                 maneuver_active: bool = False):
        """Evaluate state transitions. Call at control loop rate.

        Args:
            tag_pose: Latest valid TagPose from pose buffer, or None.
            tag_visible: True if tag is fresh (within tag_fresh_timeout_sec).
            motion_stalled: True if motion stalled (handled by action_executor now).
            now_ns: Current ROS time in nanoseconds.
            params: Dict of all relevant parameters (see _get_params_keys).
            maneuver_active: True while a blind turn-drive-turn maneuver is
                executing. The tag legitimately leaves view during the blind
                motion, so tag-loss must NOT be counted while this is True.

        Returns:
            DockingState — the current (possibly new) state.
        """
        state = self._state

        # ── Global timeout ────────────────────────────────────────
        if state in _ACTIVE_STATES:
            overall_sec = self.elapsed_ns(now_ns) * 1e-9
            if overall_sec > params.get('timeout_sec', 120.0):
                self._node.get_logger().error(
                    f'整体超时（{overall_sec:.0f}s）')
                self._record_failure(
                    f'整体超时 {overall_sec:.0f}s > timeout_sec='
                    f'{params.get("timeout_sec", 120.0):.0f}s '
                    f'(卡在 {state.name})', CODE_TIMEOUT)
                self._transition_to(DockingState.TIMEOUT)
                return self._state

            # APPROACH is owned end-to-end by the dual controller: never use
            # single-tag too-close success or re-search on loss. The wall-loss
            # watchdog below is the only tag_loss_timeout_sec consumer.
            if state == DockingState.APPROACH:
                if maneuver_active:
                    self._tag_lost_count = 0
                elif not tag_visible:
                    self._tag_lost_count += 1
                else:
                    self._tag_lost_count = 0
                if self._tag_lost_count * 0.05 > params.get('tag', {}).get('tag_loss_timeout_sec', 2.5):
                    self.abort_motion(self._wall_loss_reason(params),
                                      CODE_TAG_NOT_FOUND)
                return self._state

            # Motion failure check
            if motion_stalled:
                self._node.get_logger().error('运动卡死 — MOTION_FAILED')
                self._record_failure(
                    f'运动卡死: 指令已发但里程计不动 (卡在 {state.name}) —— '
                    f'检查底盘是否响应 /cmd_vel、里程计话题是否正确',
                    CODE_MOTION_STALLED)
                self._transition_to(DockingState.MOTION_FAILED)
                return self._state

        # ── Per-state evaluation ─────────────────────────────────
        if state == DockingState.IDLE:
            pass  # wait for start command

        elif state == DockingState.SEARCH_TAG:
            self._eval_search(tag_visible, tag_pose, now_ns, params)

        elif state == DockingState.UNDOCKING:
            # 泊出超时保护: 里程计不走(底盘卡死/失联)会卡在 UNDOCKING,
            # 直接落 MOTION_FAILED 终态。带上节点每 tick 记的 _undock_note,
            # 区分"卡在桥门控没解开"和"指令发了但底盘不动"——
            # 光一个 motion_failed 现场无从下手。
            undock_timeout = params.get('undock', {}).get('timeout_sec', 30.0)
            if self.state_elapsed_ns(now_ns) * 1e-9 > undock_timeout:
                note = getattr(self._node, '_undock_note', '')
                # 码同样由节点给: 它才知道这次是卡在门控 (motion_gated,
                # 查桥别盲重试) 还是指令发了没走 (motion_stalled, 查底盘)。
                # 两者的上层处置完全不同, 合成一个码等于没分。
                code = getattr(self._node, '_undock_code', '') \
                    or CODE_MOTION_STALLED
                self.abort_motion(
                    f'泊出超时 {undock_timeout:.0f}s'
                    + (f' — {note}' if note else ''), code)

        elif state in _ERROR_STATES or state in _SUCCESS_STATES:
            pass  # terminal states

        return self._state

    # ── Per-state evaluators ───────────────────────────────────────

    def _eval_search(self, tag_visible, tag_pose, now_ns, params):
        """SEARCH_TAG: 角度步进扫描；节点负责转/检循环，这里只判超时和锁定。

        节点的 _run_search 负责转固定角度(里程计闭环)→停稳→检测的循环。
        转动期节点冻结检测(_frozen=True)，_on_detections 直接返回，故
        tag_visible 在转动期恒为 False，这里的 tag-lock 不会在转动中误触发。
        """
        search_cfg = params.get('search', {})
        hold_time = search_cfg.get('hold_time_sec', 0.5)
        search_timeout = search_cfg.get('timeout_sec', 60.0)

        # Per-state timeout
        if self.state_elapsed_ns(now_ns) * 1e-9 > search_timeout:
            self._node.get_logger().error('搜索超时 — 未找到二维码')
            self._record_failure(
                f'搜索超时 {search_timeout:.0f}s 未找到二维码 —— '
                f'检查 dock_tag_id / 光照 / tag.size, 或先把机器人导引进视野',
                CODE_TAG_NOT_FOUND)
            self._transition_to(DockingState.TIMEOUT)
            return

        # Tag lock: 检测期持续可见 hold_time → APPROACH
        if tag_visible and tag_pose is not None:
            if self._search_tag_hold_start_ns == 0:
                self._search_tag_hold_start_ns = now_ns
            else:
                hold_elapsed = (now_ns - self._search_tag_hold_start_ns) * 1e-9
                if hold_elapsed >= hold_time:
                    self._node.get_logger().info(
                        f'二维码已锁定：距离={tag_pose.dist:.3f}m → APPROACH')
                    self._transition_to(DockingState.APPROACH)
                    return
        else:
            self._search_tag_hold_start_ns = 0

    # ── Internal ────────────────────────────────────────────────────

    def _transition_to(self, new_state: DockingState):
        if self._state == new_state:
            return
        old = self._state
        self._state = new_state
        self._state_start_ns = self._node.get_clock().now().nanoseconds

        # 进错误终态必须有记账 —— 否则上层收到的是个光秃秃的状态名, 与这次
        # 改动要解决的问题原封不动。漏了就自己喊出来 (以后新增失败路径忘记
        # 传 code 会在这里现形, 而不是静默给上层一个空原因)。
        if new_state in _ERROR_STATES and not self._abort_code:
            self._abort_code = CODE_UNSPECIFIED
            if not self._abort_reason:
                self._abort_reason = f'{new_state.name} (失败路径未记账)'
            self._node.get_logger().warn(
                f'未记账的失败路径: {old.name} → {new_state.name} —— '
                f'该处应调 abort_motion(reason, code=...) 或 _record_failure(), '
                f'请给它补一个 FAILURE_CODES 里的码')

        # Reset per-state tracking
        self._search_tag_hold_start_ns = 0
        self._tag_lost_count = 0

        self._node.get_logger().info(f'状态：{old.name} → {new_state.name}')
