"""Odom-closed-loop action executor — jog and turn with odometry dead reckoning.

Each motion is executed at a constant rate, and completion is determined
by odometry displacement (not timed). This eliminates overshoot and
undershoot from acceleration profiles — the robot moves exactly the
commanded distance or angle, verified by wheel odometry.

During motion, the camera is NOT consulted (image blur from movement
degrades detection quality). Visual checks are only used for early-stop
safety (e.g., tag distance already at target).

Usage:
    exec = ActionExecutor()
    exec.start_jog(0.5, rate=0.08)       # jog 0.5m forward at 0.08 m/s
    exec.start_turn(0.3, rate=0.3)       # turn 0.3 rad at 0.3 rad/s
    # In control loop:
    done = exec.update(odom_x, odom_y, odom_yaw, tag_visible, raw_dist, raw_lat,
                       bearing_fn, theta_bounds_fn, now_ns)
    if done:
        plan_next_step()
"""

import math
from typing import NamedTuple

from .utils import normalize_angle


class HeadingHold(NamedTuple):
    """行进中航向保持的继电参数; None = 不启用 (普通 jog / 盲腿保持原行为)。

    比例式航向保持发不出去: |wz| < l1w_control 的 min_angular_z(0.10) 会被
    clampAxis 直接截成 0 (见 ActionExecutor._min_angular_rate 注释), 所以只能
    是带迟滞的继电 —— 输出 0 或 ±rate, 无中间档。

    参数走 start_jog() 入参而非 __init__: __init__ 在节点构造时读一次就冻结,
    而这几个值恰恰最需要现场 `ros2 param set` 边跑边调 (_p() 每次现读, 改完
    下一趟生效, 不必重启)。
    """
    rate: float           # rad/s, 下发幅值 (start_jog 用 min_angular_rate 抬底)
    engage: float         # rad, |err| >= 此值接通
    release: float        # rad, |err| <= 此值断开 (绝不取 0: 报告滞后必过冲)
    min_engage_ns: int    # 最短接通时长: 单周期 wz 脉冲底盘可能不响应
    cooldown_ns: int      # 断开后最短静默: 让下一次决策基于已沉降的 odom
    budget: float         # rad, 单程累计下发角上限 (诊断闸, 非安全边界)


class ActionExecutor:
    """Odom-closed-loop action: jog (straight) and turn (rotate in place)."""

    def __init__(self,
                 turn_settle_sec: float = 0.5,
                 turn_undershoot: float = 0.75,
                 max_turn_step: float = 0.3,
                 small_turn_rad: float = 0.1,
                 final_approach_distance: float = 1.0,
                 yaw_threshold: float = 0.05,
                 turn_lead_per_speed: float = 0.30,
                 turn_slow_rad: float = 0.14,
                 min_angular_rate: float = 0.12):
        self._turn_settle_ns = int(turn_settle_sec * 1e9)
        self._turn_undershoot = turn_undershoot
        self._max_turn_step = max_turn_step
        self._small_turn_rad = small_turn_rad
        self._final_approach_distance = final_approach_distance
        # 盲转停止滞后补偿 — 与 scripts/test_turn_angle 同款的两板斧。
        # 判停依据是 /dog/odom 累计角, 但从"odom 判停"到"底盘真停"之间存在
        # 控制周期(50ms)+里程计延迟+底盘减速惯性: 实测 0.3rad/s 全速盲转
        # 每次多转 ~5-7°(对接日志: 目标 ±9.7° 实转 15-17°, 误差符号每步翻转,
        # 法线对准 2° 门槛永远进不去 → 原地摆头极限环, 横移分支永远触发不了)。
        #   1) 距目标 turn_slow_rad 内减速到半速 — 减小惯性冲量;
        #   2) 提前量判停: 剩余角 ≤ turn_lead_per_speed×当前速率 时提前发零速,
        #      让滞后滑行正好补足剩余角 (lead 与速率成线性, 同 test_turn_angle
        #      的 _LEAD_PER_SPEED)。lead 默认 0.30 由对接日志反推: 全速 0.3rad/s
        #      每次多转 5.2-7.8° → 实际滞后 0.30-0.43s; 半速 0.15rad/s 下残差
        #      ≤ (0.43−0.30)×0.15 ≈ 1.1°, 落进 2° 法线门槛内。实测欠/过量恒定时
        #      微调本值。
        self._turn_lead_per_speed = turn_lead_per_speed
        self._turn_slow_rad = turn_slow_rad
        # 必须 > l1w_control 的 min_angular_z 死区 (0.10): 低于死区的角速度会被
        # clampAxis 直接截成 0, 狗原地不动。与 stopgo.lateral_rate 同一道理,
        # 但转向通道有两级减速会叠乘, 更容易掉进死区:
        #   3° 转向 < small_turn_rad(0.1rad) → 0.3×0.5 = 0.15
        #   起步即 remaining < turn_slow_rad(0.14rad) → 再 ×0.5 = 0.075 < 0.10
        # → 命令被清零, 狗不动, 里程计只剩噪声, 反向看门狗 (0.34° 门槛) 立刻
        # 判 "opposite commanded direction"。双码 yaw 步长上限只有 3°, 每一个
        # 候选都落在这个区间, 所以双码的转向修正曾经从来没有真正执行过。
        # 减速的目的是削惯性冲量, 不是发不出去的指令 —— 掉到死区以下就抬回。
        self._min_angular_rate = max(0.0, min_angular_rate)
        # 本步起始角速度(含 start_turn 的小角半速), 近目标减速段的基准,
        # 保证只降一档、不会逐帧累乘到 0。
        self._turn_base_angular = 0.0
        # Fixed bearing tolerance the PLANNER uses to decide a yaw is needed.
        # The turn's visual early-stop must be at least this tight, otherwise
        # the planner keeps demanding a turn (|bearing| > yaw_threshold) while
        # the executor aborts it immediately against the much looser dynamic
        # theta_bounds — a deadlock at mid range (see _update_turn).
        self._yaw_threshold = yaw_threshold

        # Action state: 'idle' | 'jogging' | 'turning' | 'lateral'
        self._action = 'idle'
        self._action_linear = 0.0
        self._action_angular = 0.0
        self._action_lateral = 0.0
        self._action_target = 0.0

        # Jog tracking
        self._jog_start_x = 0.0
        self._jog_start_y = 0.0
        self._jog_start_yaw = 0.0
        self._jog_start_bearing = 0.0
        self._jog_blind = False
        # 行进中航向保持 (见 HeadingHold)。_jog_hold None = 本次行程不保持,
        # angular 恒 0, 与改动前逐位相同。
        self._jog_hold: HeadingHold | None = None
        self._jog_hold_engaged = 0.0     # 当前下发的 wz (0 = 未接通)
        self._jog_hold_engage_ns = 0
        self._jog_hold_release_ns = 0
        self._jog_hold_tick_ns = 0       # 上一周期时刻, 用于积分已下发角
        # 以下三个是诊断量, 供完成日志读取 —— 故意不在 _stop() 里清零:
        # 完成日志在 update() 返回 True 之后才打, 那时 _stop() 已经跑过。
        # 清零统一在 start_jog (下一次行程开始时)。
        self._jog_yaw_error = 0.0        # 最近一次 normalize_angle(yaw - 出发 yaw)
        self._jog_hold_used = 0.0        # 累计已下发角 (rad)
        self._jog_hold_spent = False     # 预算耗尽 / yaw 不可信 → 本程不再保持

        # Turn tracking
        self._turn_start_yaw = 0.0
        self._turn_blind_cap = None
        # Unwrapped accumulated rotation (rad) since the turn started. The raw
        # odom yaw wraps at ±π, so "turned = abs(normalize_angle(delta))" caps
        # at π and a 180° turn (target π) never completes cleanly: the robot
        # overshoots, the capped measure drops back below π, and it spins a full
        # extra revolution until delta lands on π again. Accumulate the wrapped
        # per-tick increments instead so the true total keeps growing.
        self._turn_accumulated = 0.0
        self._turn_prev_yaw = 0.0

        # Visual settle after turn
        self._last_stop_ns: int | None = None

        # Counters (diagnostic)
        self.jog_count = 0
        self.turn_count = 0

    # ── Properties ──────────────────────────────────────────────────

    @property
    def is_active(self) -> bool:
        return self._action != 'idle'

    @property
    def action_kind(self) -> str:
        return self._action

    @property
    def linear_cmd(self) -> float:
        return self._action_linear

    @property
    def angular_cmd(self) -> float:
        return self._action_angular

    @property
    def lateral_cmd(self) -> float:
        return self._action_lateral

    @property
    def jog_yaw_error(self) -> float:
        """最近一次 normalize_angle(odom_yaw - 出发 yaw); 日志/现场标定用。"""
        return self._jog_yaw_error

    @property
    def jog_hold_used(self) -> float:
        """本次行程航向保持累计已下发角 (rad)。"""
        return self._jog_hold_used

    @property
    def jog_hold_spent(self) -> bool:
        """True = 预算耗尽或 yaw 不可信, 本程后段已退回纯直行。"""
        return self._jog_hold_spent

    # ── Start actions ───────────────────────────────────────────────

    def start_jog(self, distance: float, linear_rate: float,
                  blind: bool = False, odom_scale: float = 1.0,
                  hold: HeadingHold | None = None):
        """Start a straight-line jog of `distance` metres.

        Positive = forward, negative = reverse.
        `linear_rate` is the signed constant speed (m/s).
        `blind` = True disables the visual early-stops (distance-at-target and
        bearing-drift) so the jog runs purely on odometry — required for the
        blind turn-drive-turn maneuver, where the pre-move tag pose is stale and
        the whole leg must be driven to completion regardless of what the
        (frozen) camera reading says.

        odom_scale: some legged chassis (ZSL-1) under-report translation in
        their odometry (measured ~2x low in reverse, ~4x low laterally; only
        the IMU yaw is trustworthy). `distance` is expressed in REAL metres;
        the stop target is divided by odom_scale so the run stops when the
        TRUE displacement — not the under-reported odometry — reaches it.

        hold: 行进中航向保持的继电参数, None = 不保持 (angular 恒 0, 与改动前
        逐位相同)。只有双码纯直行区那一步"一次走完"的长直行才传 (节点侧按
        plan.continuous 武装) —— 普通 jog、泊出/重试盲腿一律 None。
        """
        if self._action != 'idle':
            return
        self.jog_count += 1
        self._action = 'jogging'
        self._action_linear = linear_rate if distance >= 0 else -abs(linear_rate)
        self._action_angular = 0.0
        self._action_target = abs(distance) / max(odom_scale, 0.05)
        self._jog_blind = blind
        # rate 抬到 min_angular_rate 之上: 结构上不可能配进 l1w_control 的 0.10
        # 死区 (同 start_turn 的减速抬底, 让死区不可达而不是靠启动校验报错)。
        self._jog_hold = hold and hold._replace(
            rate=max(abs(hold.rate), self._min_angular_rate))
        self._jog_hold_engaged = 0.0
        self._jog_hold_engage_ns = 0
        self._jog_hold_release_ns = 0
        self._jog_hold_tick_ns = 0
        self._jog_yaw_error = 0.0
        self._jog_hold_used = 0.0
        self._jog_hold_spent = False

    def start_turn(self, angle: float, angular_rate: float,
                   full: bool = False) -> bool:
        """Start an in-place rotation of `angle` radians.

        Positive = CCW (left turn), negative = CW (right turn).

        `full` = False (default, legacy aim-and-go): apply undershoot and
        max-step clamping — a conservative fraction of the angle, re-measured
        each step.

        `full` = True (turn-drive-turn maneuver): execute the ENTIRE computed
        angle, NO undershoot, NO max-step cap. The turn-drive-turn geometry is
        computed as one atomic path; clamping turn1 here would send the robot
        off along the wrong heading for the full drive leg. Odometry closes the
        loop and the planner re-measures after the whole sequence, so the full
        angle is both wanted and safe.

        Returns True if the action actually started, False if the angle was too
        small to bother (caller should advance to the next queued step).
        """
        if self._action != 'idle':
            return False

        if full:
            damped = angle  # execute the full computed angle
            min_turn = 0.005  # rad (~0.3°) — below this, not worth a move
        else:
            # Undershoot: only turn a fraction of the requested angle to
            # prevent overshoot from chassis inertia
            damped = angle * self._turn_undershoot
            if abs(damped) > self._max_turn_step:
                damped = self._max_turn_step if damped > 0 else -self._max_turn_step
            min_turn = 0.02

        if abs(damped) < min_turn:  # too small to bother
            return False

        self.turn_count += 1
        self._action = 'turning'
        # Slow down for small turns, but never below the chassis dead zone.
        rate = abs(angular_rate)
        if abs(damped) < self._small_turn_rad:
            rate = max(rate * 0.5, min(self._min_angular_rate, abs(angular_rate)))
        self._action_angular = rate if damped >= 0 else -rate
        self._action_linear = 0.0
        self._action_target = abs(damped)
        self._turn_base_angular = self._action_angular
        return True

    def start_jog_lateral(self, distance: float, lateral_rate: float,
                          odom_scale: float = 1.0):
        """Start a pure lateral move (omni/mecanum only).

        Positive distance = move left, negative = move right.
        Completion is tracked by projecting odometry displacement onto
        the lateral axis at the start of the action.

        odom_scale: some legged chassis (ZSL-1) under-report lateral
        displacement in their odometry (measured ~4x low). `distance`
        is expressed in REAL metres; the odometry stop target is divided
        by odom_scale so the run stops when the TRUE displacement — not
        the under-reported odometry — reaches `distance`.
        """
        if self._action != 'idle':
            return
        self.jog_count += 1
        self._action = 'lateral'
        self._action_lateral = lateral_rate if distance >= 0 else -abs(lateral_rate)
        self._action_linear = 0.0
        self._action_angular = 0.0
        self._action_target = abs(distance) / max(odom_scale, 0.05)

    # ── Set odometry reference ──────────────────────────────────────

    def set_odom_ref(self, x: float, y: float, yaw: float, bearing: float = 0.0,
                     blind_cap: float | None = None):
        """Record the current odometry as the reference for the active action.

        Must be called ONCE after start_jog/start_turn, when the robot is
        considered to have begun moving from this odometry position.
        """
        if self._action == 'jogging':
            self._jog_start_x = x
            self._jog_start_y = y
            self._jog_start_yaw = yaw
            self._jog_start_bearing = bearing
        elif self._action == 'lateral':
            self._jog_start_x = x
            self._jog_start_y = y
            self._jog_start_yaw = yaw
        elif self._action == 'turning':
            self._turn_start_yaw = yaw
            self._turn_blind_cap = blind_cap
            self._turn_accumulated = 0.0
            self._turn_prev_yaw = yaw

    # ── Update (call at control-loop rate) ──────────────────────────

    def update(self, odom_x: float, odom_y: float, odom_yaw: float,
               tag_visible: bool, raw_dist: float | None,
               bearing_fn, theta_bounds_fn,
               target_distance: float, drift_tol: float,
               now_ns: int) -> bool:
        """Check if the current action has completed via odometry.

        Returns True when the action is done (robot has moved the
        commanded distance/angle, or a visual safety check triggers
        early stop).

        After returning True, the caller should read fresh tag data
        and plan the next step.
        """
        if self._action == 'idle':
            return True

        if self._action == 'jogging':
            return self._update_jog(odom_x, odom_y, odom_yaw,
                                    tag_visible, raw_dist, bearing_fn,
                                    theta_bounds_fn, target_distance, drift_tol,
                                    now_ns)

        if self._action == 'lateral':
            return self._update_lateral(odom_x, odom_y,
                                         tag_visible, raw_dist, target_distance)

        if self._action == 'turning':
            return self._update_turn(odom_yaw, tag_visible, raw_dist,
                                     bearing_fn, theta_bounds_fn)

        return False

    def _update_jog(self, ox, oy, oyaw, tag_visible, raw_dist,
                    bearing_fn, theta_bounds_fn, target_dist, drift_tol,
                    now_ns) -> bool:
        dx = ox - self._jog_start_x
        dy = oy - self._jog_start_y
        traveled = math.hypot(dx, dy)

        # 航向保持先算, 再走判停 —— 双码直行一律 blind=True (docking_node
        # _launch_step), 放在下面的 _jog_blind 早返回之后就永远执行不到。
        self._update_heading_hold(oyaw, traveled, now_ns)

        # Blind jog (turn-drive-turn leg): odometry-only, NO visual early-stop.
        # The pre-move tag pose is stale for the whole maneuver and the leg was
        # computed as part of one atomic path — a visual short-circuit here
        # would truncate the drive and strand the robot off the normal line.
        if self._jog_blind:
            if traveled >= self._action_target:
                self._stop()
                return True
            return False

        # Safety: visual distance already at target → early stop
        if tag_visible and raw_dist is not None and raw_dist <= target_dist:
            self._stop()
            return True

        # Safety: bearing drift during jog (chassis may run an arc)
        # If bearing has drifted too far, stop early and re-plan
        if tag_visible and raw_dist is not None:
            current_bearing = bearing_fn()
            drift = abs(normalize_angle(current_bearing - self._jog_start_bearing))
            if drift > 2.0 * theta_bounds_fn():
                self._stop()
                return True

        # Odometry target reached
        if traveled >= self._action_target:
            self._stop()
            return True

        return False

    # 20°: 0.08m/s 走 6.5s 物理上不可能真偏这么多 (真偏了墙码早出画、另有丢失
    # 闭锁), 只能是读数故障。不给参数 —— 它不是一个可调的取舍。
    _HOLD_SANITY_RAD = 0.35

    def _update_heading_hold(self, oyaw, traveled, now_ns):
        """继电式航向保持: 输出 0 或 ±rate, 只能让 |err| 下降。

        err = normalize_angle(odom_yaw - 出发 yaw)。参考是"出发那一刻的朝向",
        只做差分用 —— 我们问的不是"绝对朝向对不对"(那由上一停的视觉对准决定,
        本方法不碰), 而是"从出发到现在有没有漂"。起始朝向即便本身偏了, 保持
        也只是原样保留它 (= wz≡0 的老行为), 不会放大。

        wz = -copysign(rate, err) 的符号规则保证保持永远不会主动把狗转离出发
        朝向, 所以"越修越歪"的唯一通道是 odom yaw 增量本身错了 —— 那会同时
        打坏系统里每一次 start_turn (同一个 odom_yaw), 不是本功能新增的风险。
        预算 (budget) 因此不是安全边界, 而是"底盘不响应 wz / odom yaw 疯了"
        的诊断闸: 正常一趟只需 5-10°。

        为什么不用墙码 bearing 做参考: ① 行程中根本没有 bearing (盲动期检测
        冻结); ② 近场 bearing 带相机横向力臂增益 1+t_x/z, 最不可信 —— 那正是
        区内禁转向的理由; ③ bearing 把横偏和航向混在一起, 追它就是 pure
        pursuit (横向控制器); ④ 这个底盘只有 IMU yaw 可信 (见 start_jog)。
        """
        h = self._jog_hold
        if h is None:
            return
        # 读数故障 → 断开并停用本程保持, 退化成 wz≡0 (即老行为)。odom 过期/
        # 非有限的主防线在上游 (ActionWatch 每周期查, 过期即掐掉整个动作),
        # 这里是给没有那层保护的路径 (单码/脚本) 兜底。
        if not math.isfinite(oyaw):
            self._jog_hold_engaged = self._action_angular = 0.0
            self._jog_hold_spent = True
            return
        err = normalize_angle(oyaw - self._jog_start_yaw)
        self._jog_yaw_error = err
        if abs(err) > self._HOLD_SANITY_RAD:
            self._jog_hold_engaged = self._action_angular = 0.0
            self._jog_hold_spent = True
            return
        # dt 由时间差积分, 不写死控制周期 —— 免得与 docking_node 的定时器
        # 周期形成隐式耦合 (改一个忘了改另一个, 预算就会静默偏掉)。
        dt = (now_ns-self._jog_hold_tick_ns)*1e-9 if self._jog_hold_tick_ns else 0.
        self._jog_hold_tick_ns = now_ns
        if self._jog_hold_engaged:
            self._jog_hold_used += abs(self._jog_hold_engaged)*max(dt, 0.)
            if self._jog_hold_used >= h.budget:
                self._jog_hold_spent = True
            held_ns = now_ns-self._jog_hold_engage_ns
            # 已越过零点并反向: 立刻断开, 不等 min_engage —— 继续发就是在
            # 主动把狗往反方向推。
            crossed = err*self._jog_hold_engaged > 0
            if (self._jog_hold_spent
                    or (abs(err) <= h.release and held_ns >= h.min_engage_ns)
                    or (crossed and abs(err) > h.release)):
                self._jog_hold_engaged = 0.0
                self._jog_hold_release_ns = now_ns
            # 否则维持原符号原幅值: 接通期内符号锁定是防抖振的结构性保证,
            # 不依赖"我们猜对了底盘的停止滞后"。
        elif (not self._jog_hold_spent
                and abs(err) >= h.engage
                and now_ns-self._jog_hold_release_ns >= h.cooldown_ns
                and self._jog_hold_used + h.rate*h.min_engage_ns*1e-9 < h.budget
                # 尾段留直: 剩余行程不够跑完一次最短接通 + 沉降就不再开,
                # 停泊那一刻狗不在弧上。门槛由已有参数导出, 不新增配置项。
                # (注: _action_target 已除过 odom_scale; 当前 jog_odom_scale
                # = 1.0, 两者同为真实米。将来若标定出 scale≠1, 这里的米制
                # 会失真, 需要改成用真实行程比较。)
                and (self._action_target-traveled) > abs(self._action_linear)
                     * (h.min_engage_ns+h.cooldown_ns)*1e-9):
            self._jog_hold_engaged = -math.copysign(h.rate, err)
            self._jog_hold_engage_ns = now_ns
        self._action_angular = self._jog_hold_engaged

    def _update_lateral(self, ox, oy, tag_visible, raw_dist, target_dist) -> bool:
        """Check lateral odometry displacement against target."""
        dx = ox - self._jog_start_x
        dy = oy - self._jog_start_y
        # Project (dx, dy) onto the lateral axis at start of action.
        # Forward = (cos_yaw, sin_yaw), lateral = (-sin_yaw, cos_yaw).
        cos_yaw = math.cos(self._jog_start_yaw)
        sin_yaw = math.sin(self._jog_start_yaw)
        lateral = -sin_yaw * dx + cos_yaw * dy

        # Safety: visual distance already at target → early stop
        if tag_visible and raw_dist is not None and raw_dist <= target_dist:
            self._stop()
            return True

        if abs(lateral) >= self._action_target:
            self._stop()
            return True

        return False

    def _update_turn(self, odom_yaw, tag_visible, raw_dist,
                     bearing_fn, theta_bounds_fn) -> bool:
        # Accumulate the unwrapped rotation. odom_yaw wraps at ±π, so a naive
        # abs(normalize_angle(odom_yaw - start)) caps at π and breaks for any
        # turn ≥ π (e.g. the 180° undock): the robot overshoots, the measure
        # drops back below the target, and it spins a full extra revolution.
        d = odom_yaw - self._turn_prev_yaw
        d = math.atan2(math.sin(d), math.cos(d))   # wrap per-tick delta to [-π, π]
        self._turn_accumulated += d
        self._turn_prev_yaw = odom_yaw
        turned = abs(self._turn_accumulated)

        # Determine completion target: use blind cap if tag not visible
        target_now = self._action_target
        if not (tag_visible and raw_dist is not None):
            if self._turn_blind_cap is not None:
                target_now = min(self._turn_blind_cap, self._action_target)

        if turned >= target_now:
            self._stop()
            return True

        # ── 停止滞后补偿 (近目标减速 + 提前量判停) ─────────────────
        # 顺序: 先减速档, 再按减速后的当前速率算提前量。
        # 减速只降一档 (基准是本步起始速率), 不会逐帧累乘到 0。
        remaining = target_now - turned
        if (remaining < self._turn_slow_rad
                and abs(self._action_angular)
                > abs(self._turn_base_angular) * 0.5 + 1e-9):
            slowed = max(abs(self._turn_base_angular) * 0.5,
                         min(self._min_angular_rate, abs(self._turn_base_angular)))
            self._action_angular = math.copysign(slowed, self._action_angular)
        # 提前量按当前速率线性缩放; 钳到目标一半, 保证极小目标角至少执行
        # 一半 —— 否则 1° 级微调会在起步前就被提前量整个吞掉, 规划器看到
        # 误差不变, 无限重发同一小转。
        lead = min(self._turn_lead_per_speed * abs(self._action_angular),
                   0.5 * target_now)
        if remaining <= lead:
            self._stop()
            return True

        # NO visual early-stop for turns.
        #
        # In stop-and-go the camera is not trusted mid-motion: docking_node
        # enforces this with its `_frozen` gate — during a blind maneuver all
        # detections are dropped, so `_raw_*`/bearing_fn() here are ALWAYS the
        # stale pre-turn values, and `tag_visible` may go stale too.
        #
        # A turn is commanded by the planner precisely because that pre-turn
        # bearing/lat is out of tolerance. The lateral trigger fires at a very
        # small bearing (atan2(lateral_threshold, dist) — e.g. 2.3° at 1.25 m),
        # far below yaw_threshold (10°). Gating turn completion on that same
        # stale bearing therefore aborted the turn at ZERO rotation on the first
        # tick, the robot never moved, the re-measured pose was identical, and
        # the planner re-demanded the same turn forever — an infinite no-progress
        # loop.
        #
        # Turn completion is governed by ODOMETRY alone (turned >= target_now),
        # plus the stop-latency compensation above (近目标减速 + 提前量判停) —
        # full=True 路径没有 undershoot/max_turn_step 保护 (docking omni 走停
        # 与泊出都走 full=True), 不补偿的话每次盲转实转比目标多 5-7°。
        return False

    # ── Visual settle after turn ────────────────────────────────────

    def wait_visual_settle(self, tag_visible: bool, now_ns: int) -> bool:
        """Return True if we should wait for visual to refresh after a turn.

        After a turn completes, the camera image may be blurry. We force
        a short wait to let fresh, sharp frames arrive before reading the
        tag pose.
        """
        if self._last_stop_ns is None:
            return False
        elapsed_ns = now_ns - self._last_stop_ns
        return elapsed_ns < self._turn_settle_ns

    # ── Stop ────────────────────────────────────────────────────────

    def _stop(self):
        was_turning = (self._action == 'turning')
        self._action = 'idle'
        self._action_linear = 0.0
        self._action_angular = 0.0
        self._jog_blind = False
        # 停用航向保持, 但 _jog_yaw_error/_jog_hold_used/_jog_hold_spent 三个
        # 诊断量故意留着: 完成日志在 update() 返回 True 之后才打, 那时这里
        # 已经跑过了 —— 清掉就永远读不到。它们在下一次 start_jog 清。
        self._jog_hold = None
        self._jog_hold_engaged = 0.0
        if was_turning:
            self._last_stop_ns = None  # caller sets this via mark_stop_time

    def mark_stop_time(self, now_ns: int):
        """Record when the action stopped, for visual settle timing."""
        self._last_stop_ns = now_ns

    def cancel(self):
        """Abort current action immediately."""
        self._action = 'idle'
        self._action_linear = 0.0
        self._action_angular = 0.0
        self._action_lateral = 0.0
        self._jog_blind = False
        self._jog_hold = None
        self._jog_hold_engaged = 0.0
        self._last_stop_ns = None
