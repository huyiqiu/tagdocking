"""Tag alignment geometry planner — aim-and-go, all chassis unified.

The planner computes the next discrete action from the tag pose relative
to the robot's base_link frame.

Diff-drive (and omni once centred) — aim-and-go / pure pursuit:
  1. If |lat| exceeds the lateral tolerance OR the bearing exceeds the yaw
     tolerance → turn to aim directly at the tag (null the bearing).
  2. Otherwise → drive straight toward the tag.
  Aiming at the tag and driving toward it shrinks lat monotonically to ~0
  at contact.  Driving straight preserves lat, so "will a straight approach
  stay within the lateral tolerance?" is exactly "is |lat| within tolerance?"
  — no diverging turn-away/turn-back oblique manoeuvre.

Omni / quadruped: turn in place to align the heading with the tag normal,
then direct lateral slide onto the normal line, then straight in.

Every motion is executed odometry-closed (stop-and-go): the robot stops,
grabs a sharp frame, plans one bounded step, dead-reckons it via odometry,
stops, and re-measures.  The camera is never trusted mid-motion.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from .utils import normalize_angle


@dataclass
class ActionPlan:
    """A single discrete action for the ActionExecutor.

    kind: 'yaw' | 'forward' | 'done'
    """
    kind: str = 'done'
    turn_angle: float = 0.0      # rad, for 'yaw' actions
    jog_distance: float = 0.0    # m,   for 'forward' actions
    lateral_distance: float = 0.0  # m, for omni lateral correction (packed in 'forward')
    # 纯直行区一次走完的连续前进 (双码专用): 路程可远超单步上限, ActionWatch
    # 据此把 deadline 放宽到实际行程, 否则 6s 动作超时会掐死 6.3s 的长直行。
    # 第二个消费者: docking_node._launch_step 据此武装"行进中航向保持"——
    # 既然区内不停下来纠方向, 就得在走的过程中守住方向, 两者是同一个决定的
    # 两半。所以这个字段的语义是"区内一次走完", 不是泛指的"步长较长"; 给
    # 非区内行程置 True 会连带拿到航向保持 (节点侧另有 jog_distance>0 显式门)。
    continuous: bool = False


class GeometryPlanner:
    """Tag normal-line alignment planner.

    Stateless aside from the diff-drive multi-phase lateral correction
    sequence (turn→jog→turn).  After each action completes and a fresh
    tag observation is available, call :meth:`plan`.
    """

    def __init__(self,
                 target_distance: float = 0.30,
                 lateral_threshold: float = 0.04,
                 yaw_threshold: float = 0.06,
                 tune_angle: float = 0.0,
                 jog_min: float = 0.05,
                 jog_max: float = 0.50,
                 position_tol: float = 0.02,
                 base_type: str = 'diff_drive'):
        self._target_dist = target_distance
        self._lateral_threshold = lateral_threshold
        self._yaw_threshold = yaw_threshold
        # 方位(bearing)转向门独立于法线对准门: 节点近场只收紧前者
        # (normal 近场抖动大, 跟着收紧会追噪声摆头), 见 set_tolerances。
        self._bearing_yaw_threshold = yaw_threshold
        self._tune_angle = tune_angle
        self._jog_min = jog_min
        self._jog_max = jog_max
        self._pos_tol = position_tol
        self._is_omni = base_type in ('omni', 'quadruped')

    # ── Public API ──────────────────────────────────────────────────

    def plan(self, dist: float, lat: float, yaw: float,
             normal: float | None = None) -> ActionPlan:
        """Return the next action based on the current tag pose.

        Args:
            dist: distance from robot COR to tag along robot's x-axis (m).
            lat:  lateral offset of tag in robot frame (m) — positive = left.
            yaw:  bearing to the tag (rad, CCW+) == atan2(lat, dist).  The
                  system measures only the *direction* to the tag, not the
                  tag's own facing, so yaw and lat are coupled.
            normal: tag outward normal direction in robot frame (rad, CCW+).
                  Omni only — enables the heading-alignment turn.  None keeps
                  the legacy bearing-only behaviour (diff-drive unaffected).

        Returns:
            ActionPlan with the next discrete action.

        Strategy — omni (with ``normal``): align → slide → straight.
            A lateral slide is docking-correct only when it moves the robot
            ONTO the tag's normal line; the final straight-in then rides that
            line into the dock.  The omni order per stop-and-go iteration is:
              1. heading error vs normal (normal + π) beyond threshold →
                 turn in place (node clamps to max_turn_step, re-measures);
              2. otherwise, perpendicular offset to the measured normal line,
                 perp = dist·sin(normal) − lat·cos(normal), beyond tolerance →
                 slide by perp.  Unlike a raw-lat slide this stays exact under
                 residual heading error: lat conflates the true offset with
                 dist·sin(heading_err) — at 1.1 m a 5° residual contributes
                 ~10 cm of "phantom lateral" that can flip a few-cm true
                 offset's SIGN (2026-09 log: robot 5.7 cm right of the line,
                 tag right in frame → lat=−0.044 slid RIGHT, away from the
                 line; perp=+0.057 slides LEFT onto it).  Reduces to lat
                 exactly when normal = π (heading square).  Normal noise is
                 amplified by dist (~1.3 cm at 1.1 m for the measured σ≈0.7°)
                 — bounded by the alignment-turn gate, EMA, and per-stop
                 re-measure feedback.  Skipped when |heading_err| > 90°
                 (mirror-flipped / garbage normal): no perp slide there —
                 falls through to the bearing aim-and-go below.
              3. otherwise straight in along the (now normal) line.
            The perpendicular construction was once rejected as "±10°
            far-field normal noise → ±0.3 m slide noise"; far-field slides no
            longer happen (normal=None there), and near field the normal is
            EMA-filtered and the residual heading error bounded by the
            alignment-turn gate — the sign robustness it buys is worth the
            noise it adds.

        Strategy — aim-and-go (pure pursuit, diff-drive):
            Driving straight forward preserves ``lat`` (the robot moves along
            its own x-axis).  So "will a forward jog leave us outside the
            ±lateral tolerance at the target?" reduces to "is |lat| already
            outside tolerance?" — a distance-independent, exact test.

            * |lat| within tolerance AND bearing within tolerance → drive
              straight.  A small residual lat/yaw is left as-is; it is inside
              the docking tolerance and needs no correction.
            * otherwise → turn to aim directly at the tag (null the bearing).
              Aiming at the tag and then driving toward it monotonically
              shrinks lat to ~0 at contact — it converges, unlike the old
              turn-away/turn-back oblique scheme which diverged.

            Every turn is executed odometry-closed with undershoot in the
            ActionExecutor, so a turn never overshoots and flings the tag out
            of frame.
        """
        # ── Omni / quadruped: real lateral DOF → align, then slide ──
        # Step 1 — heading alignment with the tag normal.  Rotation about the
        # robot's own axis never changes the line-of-sight-to-normal angle, but
        # it does change the heading-vs-normal error by exactly the turn — so
        # this converges in clamped steps, re-measured at every stop.  Skipped
        # beyond ±90°: there the tag faces away (or the solvePnP flip leaked
        # through the EMA) and chasing it would spin the robot.
        #
        # ── 横移: 平移到「测得法线」上, 用垂直偏距而非画面横向 lat ──
        # 只在「法线对准」模式下触发 (normal is not None)。normal=None 是
        # pure-pursuit / 纯方位模式 (远场), 远场不横移 (法线噪声 ±10° 会被
        # dist 放大成 ±0.3m 横移噪声)。
        #
        # 为什么不能按 lat 横移: lat = 真实垂直偏距 − dist·sin(残余航向误差)。
        # 车头没完全转正时, 1.1m 处 5° 残余误差贡献 ~10cm "假横向", 能淹没并
        # 反转真实 4-5cm 偏距的符号 (2026-09 实测: 狗在线右侧 5.7cm、车头左偏
        # 5.3°、tag 在画面右侧 → lat=-0.044 按右横移, 离线更远; 正确动作是
        # 左移 5.7cm = dist·sin(n)−lat·cos(n))。垂直偏距把航向误差项减掉,
        # 车头未转正也移向正确的线; n=π (已转正) 时严格退化为 lat。
        # 噪声 = dist×(normal 读数误差): 近场受转向门槛钳制 + 循环 EMA,
        # 实测 σ≈0.7° → ~1.3cm; 且每个停看点重测, 有反馈兜底。
        #
        # 垂直偏距只在「法线可信」区间使用 (|heading_err| ≤ 90°): 转向分支
        # 未触发的两种落法 —— 对准区 (|err| ≤ 门槛, perp 修正项有意义) 与
        # 翻转/垃圾区 (|err| > 90°, normal 可能是镜像解)。垃圾区若仍按 perp
        # 横移, 45° 垃圾法线会算出 dist·sin(45°)≈0.7m 幻影横移 (旧代码按 lat
        # 横移虽无此放大但也无意义) —— 统一交给下方 aim-and-go 的 bearing
        # 转向处理 (bearing 只依赖 tag 位置, 稳定可靠), 下一停重测。
        if self._is_omni and normal is not None:
            heading_err = normalize_angle(normal + math.pi)
            if self._yaw_threshold < abs(heading_err) <= math.pi / 2:
                return ActionPlan(kind='yaw', turn_angle=heading_err)
            if abs(heading_err) <= math.pi / 2:
                perp = dist * math.sin(normal) - lat * math.cos(normal)
                if abs(perp) > self._lateral_threshold:
                    return self._start_lateral(perp)

        # ── Diff-drive (and omni once centred): aim-and-go ──
        # Turn only when a straight approach would miss the lateral tolerance
        # (|lat| too big) or we are pointed too far off the tag (|yaw| too big).
        if abs(lat) > self._lateral_threshold or abs(yaw) > self._bearing_yaw_threshold:
            return ActionPlan(kind='yaw', turn_angle=yaw)

        # ── Straight approach along the line of sight ──
        remaining = dist - self._target_dist
        if abs(remaining) <= self._pos_tol:
            return ActionPlan(kind='done')

        # Clamp jog distance
        jog = min(abs(remaining), self._jog_max)
        jog = max(jog, self._jog_min) if jog > 0 else 0.0

        if jog < 1e-3:
            return ActionPlan(kind='done')

        jog_signed = jog if remaining > 0 else -jog
        return ActionPlan(kind='forward', jog_distance=jog_signed)

    def reset(self):
        """No-op retained for API compatibility (aim-and-go is stateless).

        The old diff-drive multi-phase lateral sequence held state here; the
        current planner has none, so callers (abort/start/cancel) need do
        nothing, but the method is kept so those call sites stay valid.
        """
        pass

    def set_tolerances(self, lateral_threshold: float, yaw_threshold: float,
                       bearing_yaw_threshold: float) -> None:
        """运行期同步修正容差 (与 set_jog_limits 同款免重启机制)。

        节点在近场把方位(bearing)/横向修正门槛收紧到直行入口包络, 让
        阶段1 主动把 3~10°/3~5cm 的小偏差修掉, 而不是"入口判不合格、
        规划器却认为无需修正"。法线(normal)对准门槛同样在近场收紧到
        normal_yaw_threshold_deg —— 横移已改垂直距离补偿航向误差, 收紧后
        "先对齐法线再横移"的次序成立, 直行入口才真正正对轴线。
        """
        self._lateral_threshold = lateral_threshold
        self._yaw_threshold = yaw_threshold
        self._bearing_yaw_threshold = bearing_yaw_threshold

    def set_target_distance(self, target_distance: float) -> None:
        """运行期同步停泊目标距离 (免重启, 同 set_jog_limits 机制)。

        双码模式每个停看点把 dual.dock_distance + cam_dx (相机系→
        base_link 系换算) 同步进来, plan()/plan_straight() 的剩余距离
        与 done 判据随之切换; 单码模式不调用, 保持构造值不变。
        """
        self._target_dist = target_distance

    def set_jog_limits(self, jog_min: float, jog_max: float) -> None:
        """运行期更新走停步长上下限。

        jog_max/jog_min 是 ROS 参数，但构造时一次性拷进本类 —— 不同步的话
        ``ros2 param set`` 改了也不生效。节点在每次规划前调用本方法把当前
        参数值同步进来，实现免重启调整。
        """
        self._jog_min = jog_min
        self._jog_max = jog_max

    # ── Turn-drive-turn sequence (normal-line docking) ───────────────

    def plan_sequence(self, dist: float, lat: float, normal: float,
                      yaw_tol: float | None = None) -> list[ActionPlan]:
        """Plan a full blind maneuver onto the tag's normal line.

        Instead of the fragile incremental "turn a little, look, turn again"
        loop — which swings the tag toward the FOV edge and loses it — this
        computes the ENTIRE path from a single good measurement and returns it
        as a list of actions the executor runs back-to-back by odometry, with
        the camera not consulted until the maneuver finishes. The tag leaving
        view mid-maneuver is expected and fine.

        Geometry (all in base_link, REP-103: x fwd, y left, +yaw CCW):

            T = (dist, lat)                     tag position
            n = (cos normal, sin normal)        tag outward normal (already
                                                corrected to point toward the
                                                robot side)
            A = T + d_target · n                docking standoff point on the
                                                normal line, d_target from tag

            turn1 = atan2(A_y, A_x)             rotate to face A
            drive = |A|                         straight to A
            turn2 = (normal + pi) - turn1       at A, face the tag (-n dir)

        After turn1+drive+turn2 the robot sits on the tag's normal line at
        d_target, facing the tag squarely. A final straight-in jog is then just
        the residual (handled by re-measuring — see the node's iterate loop).

        Returns [] when already within tolerance (docked).

        Note: relies on `normal`, which is AprilTag's least-reliable DOF far
        away. The node re-measures and re-plans at each stop (iterative
        refine): as the robot nears and squares up, `normal` stabilizes and the
        sequence converges. Early iterations need only be approximately right.
        """
        yt = yaw_tol if yaw_tol is not None else self._yaw_threshold

        # Standoff point A on the normal line.
        n_x, n_y = math.cos(normal), math.sin(normal)
        a_x = dist + self._target_dist * n_x
        a_y = lat + self._target_dist * n_y

        # Are we already docked? On the normal line at d_target, facing tag.
        # Residual position error = distance from robot origin to A.
        pos_err = math.hypot(a_x, a_y)
        # Residual heading error: direction robot must face to look at tag.
        bearing = math.atan2(lat, dist)
        # "Squareness": how far the current line-of-sight is off the tag normal.
        # When square-on, bearing == normal + pi (robot faces along -n).
        square_err = abs(normalize_angle((normal + math.pi) - bearing))

        if pos_err <= self._pos_tol and square_err <= yt:
            return [ActionPlan(kind='done')]

        turn1 = math.atan2(a_y, a_x)
        drive = pos_err

        # Cap the blind straight leg. A very long dead-reckoned drive accumulates
        # odometry error and (at low frame rate) keeps the tag out of view too
        # long, so we advance at most jog_max per iteration and re-measure.
        clamped = drive > self._jog_max
        if clamped:
            drive = self._jog_max

        # Final turn — CRUCIAL for a narrow FOV. Every blind maneuver must END
        # with the robot facing the tag, or it can never re-acquire it and is
        # declared lost. Two cases:
        #   • reached A (not clamped): face the tag SQUARE-ON along -normal, so
        #     the next straight-in jog stays on the normal line.
        #   • clamped (stopped short of A): we are NOT on the normal line yet, so
        #     square-on would point away from the tag. Instead re-aim straight AT
        #     the tag from the position we actually reach, keeping it centred in
        #     frame for the next measurement. Iterative refine converges as the
        #     robot nears and the clamp stops biting.
        if clamped:
            # Position reached after turn1 + drive (robot heading == turn1).
            p_x = drive * math.cos(turn1)
            p_y = drive * math.sin(turn1)
            # Direction to the tag from there, in the ORIGINAL frame …
            phi = math.atan2(lat - p_y, dist - p_x)
            # … expressed relative to the robot's post-drive heading (turn1).
            turn2 = normalize_angle(phi - turn1)
        else:
            turn2 = normalize_angle((normal + math.pi) - turn1)

        seq: list[ActionPlan] = []
        if abs(turn1) > 1e-3:
            seq.append(ActionPlan(kind='yaw', turn_angle=turn1))
        if drive > self._jog_min * 0.5:
            seq.append(ActionPlan(kind='forward', jog_distance=drive))
        if abs(turn2) > 1e-3:
            seq.append(ActionPlan(kind='yaw', turn_angle=turn2))

        # If everything rounded away but we weren't "done", nudge straight so we
        # never return an empty non-done list (which would stall the loop).
        if not seq:
            return [ActionPlan(kind='done')]
        return seq

    # ── Straight-line final approach (two-phase docking phase 2) ────

    def plan_straight(self, dist: float) -> list[ActionPlan]:
        """Plan a pure straight-line forward jog — no angle adjustment.

        Two-phase docking phase 2: the caller has already confirmed the robot
        is within ``start_distance`` of the tag and the heading is square-on
        within the dedicated yaw threshold. Drive straight forward along the
        current heading until within ``position_tol`` of ``target_distance``.

        Lateral and yaw errors are intentionally NOT corrected — the robot
        drives straight regardless, like parking into a garage. Any lateral
        offset present at the phase-2 handoff is preserved to the final pose.

        Returns ``[done]`` when within position tolerance of target_distance.
        """
        residual = dist - self._target_dist
        if abs(residual) <= self._pos_tol:
            return [ActionPlan(kind='done')]
        # Clamp to jog_max — iterative stop-and-go, re-measure after each step
        # (preserves re-measure safety and narrow-FOV tag retention).
        drive = min(residual, self._jog_max)
        if drive <= self._jog_min * 0.5:
            return [ActionPlan(kind='done')]
        return [ActionPlan(kind='forward', jog_distance=drive)]

    # ── Lateral correction helpers ──────────────────────────────────

    def _start_lateral(self, perp: float) -> ActionPlan:
        """Omni / quadruped only: direct lateral slide onto the normal line.

        Called by plan() step 2 with the PERPENDICULAR offset to the measured
        normal line, ``dist·sin(normal) − lat·cos(normal)`` — not the raw
        image-frame lat, which conflates the true offset with
        dist·sin(heading_err) and reverses the slide direction when a ~5°
        residual heading error dominates a few-cm true offset (2026-09 log:
        robot 5.7 cm right of the line, tag right in frame → lat=−0.044 slid
        RIGHT, away from the line; perp=+0.057 slides LEFT onto it).

        Signed ``perp``: positive ⇒ robot right of the normal line (tag to
        the left) ⇒ slide left (positive), matching
        ActionExecutor.start_jog_lateral ("positive = move left").
        """
        return ActionPlan(kind='forward',
                          jog_distance=abs(perp),
                          lateral_distance=perp)
