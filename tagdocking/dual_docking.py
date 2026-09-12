"""Optical dual-tag stop/look controller; no wall-only steering fallback.

Coordinates are calibrated optical x-right/y-down/z-forward. Ground-plane
geometry uses the complete optical-to-base rigid transform, not tag normals.
The historical parallax solver is retained below for offline compatibility.
"""
import math

from .geometry_planner import ActionPlan


DEFAULTS = {
    # 站位 1.8m: 站立全程下桩码 (~0.9m) 在此清晰可见 (z_min = 0.0745+1.108·y,
    # 见 README 6.5b); straight_start_distance=1.70 = obs - tol, 现在只剩两处
    # 用法 (排序校验 / yaw_cap 远近分档) —— 桩码垂直离场放行与丢失闭锁已改用
    # straight_envelope (= obs + tol), 见该 property 的死区说明。
    'observation_distance': 1.8, 'observation_tolerance': 0.1,
    # 前进/后退步长。五处发出点原来都写 min(.05, self.p('forward_step')) ——
    # 一个写死的硬上限, 把配置项变成了装饰 (和 yaw_cap 那次同一个病, 见
    # yaw_step_deg 注释; 现场 docking.yaml 把 reverse_step 调到 0.10 一直没有
    # 任何效果)。5cm 在 1.8→0.5m 这 1.3m 上是 26 步, 每步起停+停稳+重测
    # ~2.9s ≈ 75s, 开销远大于位移本身; 10cm 减半到 13 步。
    # 精度不受影响: 终点步仍按 remaining 收缩 (见 _advance), dock_tolerance 不变。
    # 放大步长不牺牲可见性: visible() 逐段采样整条轨迹 (_emit 对一切计划
    # 过这把尺), 会否掉把 tag 甩出视野的大步。曾写"stopgo.jog_max (0.20)
    # 是执行器侧的硬闸" —— 错: jog_max 的 clamp 全部在单码 GeometryPlanner
    # (geometry_planner.py 的 plan/plan_sequence/plan_straight), 执行器
    # start_jog 对任何距离都不钳制, 双码从来不受它管。双码真实上限就是
    # 本参数 (区外) 与纯直行区的"一次走完"。
    # 后退预算按距离而非步数守: reverse_limit 0.80m 不变, 步数自然从 16 掉到
    # 8 (reverse_count 退化为不起作用的天花板), 站位账 1.00m → 10 步。
    # 横移步长是同一个病的第三处: 候选枚举里写死 min(.03, lateral_step), 于是
    # docking.yaml 的 0.05 在现场跑出来仍是 0.03。硬上限一并拆除, 改由
    # __init__ 的 [0.005, 0.10] 区间校验兜底 (单步盲走没有途中反馈, 上界
    # 取保守的合理值, 防一步把两码一起推出画面)。
    'forward_step': 0.10, 'reverse_step': 0.10, 'lateral_step': 0.05,
    'yaw_step_deg': 8.0, 'yaw_fine_step_deg': 3.0,
    # 纯直行区上界: 墙码光学 z ≤ 此值后不再调方向 (转向与横移都禁), 只许前进。
    # 近场调方向是负期望的交易: 相机在 base 上有横向力臂, bearing 增益
    # 1+t_x/z 在 z=0.6 时已到 1.33 (命令 2.86° 实变 3.30°), 而底盘停止滞后
    # ~2° 与命令本身同量级 —— 越近, 一步转向的不确定度越大、可纠偏的余量
    # 越小。代价是放弃最后 ~0.5m 的 locked pure pursuit 横向收缩
    # (0.5/1.3 ≈ 0.38, 见 README 设计决定 1); 换来的是近场不再摆头/横移。
    # 取值须 target < 本值 <= straight_start_distance (启动校验)。
    'steering_stop_distance': 1.0,
    # locked 直行的墙码 bearing 微调: 容差必须 ≤ align_tolerance_deg (它是最后
    # 的精修, 门槛比入口门还松就等于能撤销入口门), 下限 0.5° 是底盘转向分辨率。
    'straight_yaw_tol_deg': 1.5, 'straight_yaw_max_turns': 3,
    'straight_yaw_lag_deg': 2.0,
    'prealign_tolerance_deg': 5.0, 'prealign_step_deg': 8.0,
    'prealign_max_steps': 12,
    'settle_sec': 1.5, 'min_frames': 3,
    'fresh_sec': 0.6, 'tf_skew_sec': 0.02, 'missing_confirm_sec': 1.5,
    'missing_timeout_sec': 8.0, 'qualification_sec': 12.0,
    # 站位后退独立预算 (现场起点 1.371m → 1.7m 站位 = 0.33m = 7 步, 3× 裕量);
    # acquire 后退现在要从 ~1.37m 开到 1.8m, 与脱困共用主账 → 0.8m/16 次。
    'reverse_limit': 0.80, 'reverse_count': 16,
    'standoff_reverse_limit': 1.00, 'standoff_reverse_count': 20,
    'acquire_timeout_sec': 60.0,
    'dock_tolerance': 0.02, 'mount_tolerance_deg': 2.0,
    'pile_lock_distance': 0.30, 'crouch_settle_sec': 3.0,
    'posture_retries': 2,
    'max_actions': 160, 'stable_position_m': 0.03,
    'min_lateral_m': 0.003, 'lateral_tolerance_m': 0.02,
    'observe_timeout_sec': 120.0, 'camera_wait_sec': 5.0,
    'visibility_margin_px': 8.0, 'visibility_sample_pad_px': 2.0,
    'visibility_samples': 12, 'score_improvement': 0.002,
    'feedback_min_improvement': 0.01, 'feedback_fail_windows': 3,
    'no_candidate_windows': 3, 'log_period_sec': 2.0,
    'odom_fresh_sec': 0.5, 'action_timeout_sec': 6.0,
    'response_timeout_sec': 2.0, 'odom_noise_m': 0.001,
    'odom_noise_rad': 0.002, 'action_startup_sec': 0.6,
}


def rotation(q):
    """Quaternion xyzw to optical-to-base rotation; reject invalid calibration."""
    x, y, z, w = q
    n = sum(v*v for v in q)
    if not math.isfinite(n) or abs(n - 1.0) > 0.01:
        raise ValueError('camera extrinsic quaternion must be normalized')
    return ((1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)),
            (2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)),
            (2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)))


def transform(point, r, t):
    return tuple(sum(row[j]*point[j] for j in range(3))+t[i]
                 for i, row in enumerate(r))


def predict(point, r, t, forward=0., lateral=0., yaw=0.):
    """Full SE(2) base displacement, including camera lever arm and pitch."""
    b = transform(point, r, t)
    x, y = b[0]-forward, b[1]-lateral
    c, sn = math.cos(yaw), math.sin(yaw)
    bnext = (c*x+sn*y-t[0], -sn*x+c*y-t[1], b[2]-t[2])
    return tuple(sum(r[j][i]*bnext[j] for j in range(3)) for i in range(3))


def geometry(wall, pile, r, t, tol):
    bw, bp = transform(wall, r, t), transform(pile, r, t)
    dx, dy = bw[0]-bp[0], bw[1]-bp[1]
    if not (0.08 <= math.hypot(dx, dy) <= 2.) or dx <= 0:
        raise ValueError('invalid dual ground-plane baseline')
    theta = math.atan2(dy, dx)
    if abs(theta) > math.radians(45):
        raise ValueError('dual heading exceeds safe geometry envelope')
    e = -math.sin(theta)*(bp[0]-t[0])+math.cos(theta)*(bp[1]-t[1])
    bearings = tuple(math.atan2(p[0], p[2]) for p in (wall, pile))
    j = (0.5*sum((b/tol)**2 for b in bearings) + (max(map(abs,bearings))/tol)**2
         + 0.25*(theta/math.radians(3))**2 + 0.25*(e/.03)**2)
    return bearings, theta, e, j


class DualTagDocking:
    """One observation window -> at most one bounded action.

    action_started commits budgets; action_completed commits forward progress.
    Discarded pending plans cannot create qualification. Missing detections and
    failed TF queries are different inputs: only the former can qualify loss.
    """
    def __init__(self, node):
        self._node = node
        self.enabled = bool(node._p('dual.enable'))
        self._pile_tag_id = int(node._p('dual.pile_tag_id'))
        self.r = self.t = None
        self.camera = None
        # 匍匐 profile 开关 (dual.crouch_enable): false = 全程站立, 不在此切站立;
        # true = 原匍匐流程, 桩码到锁定距离且对准成立时切站立。enable=false 也读,
        # 保证属性恒存在。
        self.crouch = bool(node._p('dual.crouch_enable'))
        self.reset()
        if self.enabled:
            if self._pile_tag_id == int(node._p('tag.id')):
                raise ValueError('dual wall and pile IDs must differ')
            if node._p('base.type') not in ('omni', 'quadruped'):
                raise ValueError('dual docking requires a lateral-capable base')
            for name, value in DEFAULTS.items():
                v = self.p(name)
                if not math.isfinite(v) or v <= 0:
                    raise ValueError('dual.' + name + ' must be finite and positive')
            for name in ('visibility_samples', 'feedback_fail_windows', 'no_candidate_windows',
                         'min_frames', 'max_actions', 'reverse_count', 'prealign_max_steps',
                         'standoff_reverse_count', 'straight_yaw_max_turns'):
                if self.p(name) < 1 or not self.p(name).is_integer():
                    raise ValueError('dual.' + name + ' must be a positive integer')
            if self.p('visibility_samples') > 256:
                raise ValueError('dual.visibility_samples must be <= 256')
            if self.p('min_lateral_m') > self.p('lateral_step'):
                raise ValueError('dual.min_lateral_m exceeds dual.lateral_step')
            # 横移步长同样不再有 3cm 硬上限 (原 min(.03, lateral_step), 与 yaw_cap
            # 和 forward_step 同一种病: 配置项是装饰)。上限留 0.10m —— 横移是
            # 单步盲走, 没有转向那样的角度反馈, 一步太大轻则把两码推出画面
            # (visible() 会拦), 重则误差无处吸收。执行器对任何通道都不钳制
            # 距离 (jog_max 只闸单码规划器), 本区间就是双码横移的全部防线。
            if not 0.005 <= self.p('lateral_step') <= .10:
                raise ValueError('dual.lateral_step must be in [0.005, 0.10] m')
            # yaw 步长不再有 3° 硬上限 (见 yaw_cap): 3° 命令的停止滞后与命令本身
            # 同量级, 提前量被 0.5*target 钳位 → 每步只转一半, 远场 20° 偏差要
            # 30 步才收敛, 必然撞上 observe_timeout。但上限也不能无界: 大步转向
            # 会把 tag 甩出视野 (visible() 会否掉, 于是一个候选都不剩), 且
            # max_turn_step (0.17rad≈9.7°) 是执行器侧的硬闸。留 15° 作为理智上限。
            for name in ('yaw_step_deg', 'yaw_fine_step_deg'):
                if not 0.5 <= self.p(name) <= 15.:
                    raise ValueError('dual.' + name + ' must be in [0.5, 15] degrees')
            if self.p('yaw_fine_step_deg') > self.p('yaw_step_deg'):
                raise ValueError('dual.yaw_fine_step_deg must not exceed yaw_step_deg')
            # 单码粗对准门槛必须比双码对准门槛宽: 否则粗对准要去做双码的活,
            # 而单码量测 (墙码 bearing) 的噪声正是双码几何要绕开的东西。
            if self.p('prealign_tolerance_deg') < float(node._p('dual.align_tolerance_deg')):
                raise ValueError('dual.prealign_tolerance_deg must be >= align_tolerance_deg')
            if not 1. <= self.p('prealign_step_deg') <= 15.:
                raise ValueError('dual.prealign_step_deg must be in [1, 15] degrees')
            for name in ('wall_tag_size', 'pile_tag_size'):
                if not math.isfinite(self.p(name)) or self.p(name) <= 0:
                    raise ValueError('dual.' + name + ' must be finite and positive')
            if not (0 < self.target < self.near < self.p('observation_distance')):
                raise ValueError(
                    'require dock_distance < straight_start_distance < observation_distance, got '
                    f"{self.target:.2f} / {self.near:.2f} / {self.p('observation_distance'):.2f} "
                    '(站立式: observation_distance=1.8, straight_start_distance=1.70)')
            if not 0 < self.p('pile_lock_distance') < self.p('observation_distance'):
                raise ValueError('require 0 < pile_lock_distance < observation_distance')
            # 纯直行区必须夹在停泊点与站位之间: 低于 dock_distance 等于从不生效
            # (禁令区在终点之后), 高于站位则把 observe 的纠偏一起禁掉 —— 而双码
            # 对准本来就只在站位处做, 禁掉它整场就没有对准环节了。
            # 取 == near 合法: "整个 approach 全程纯直行"是一个有意义的配置。
            if not self.target < self.p('steering_stop_distance') <= self.near:
                raise ValueError(
                    'require dock_distance < steering_stop_distance <= '
                    'straight_start_distance, got '
                    f"{self.target:.2f} / {self.p('steering_stop_distance'):.2f} / "
                    f'{self.near:.2f}')
            # locked 的墙码 bearing 微调是最后的精修: 容差比双码对准门还松就
            # 等于能撤销入口门; 下限 0.5° 以下落进底盘转向分辨率, 是命令噪声。
            if not 0.5 <= self.p('straight_yaw_tol_deg') <= float(node._p('dual.align_tolerance_deg')):
                raise ValueError('dual.straight_yaw_tol_deg must be in [0.5, align_tolerance_deg] degrees')
            # 上界 15°: 门槛 = atan(target·sin(lag)/(z−target)), lag 再大只是
            # 把区内转向整体关掉 (= 旧行为), 不会不安全; 拦在这里只为让
            # "本以为配了个小旋钮、实际关掉了整个功能" 在启动时就被看见。
            if not 0 < self.p('straight_yaw_lag_deg') <= 15:
                raise ValueError('dual.straight_yaw_lag_deg must be in (0, 15] degrees')
            # 站位必须在观察窗内 (≤ obs-tol): 它同时是 approach 桩码丢失闭锁的
            # 窗口上界、yaw_cap 远/近分档与站位守卫的基准, 站在窗外没有意义。
            if self.near > self.p('observation_distance') - self.p('observation_tolerance') + 1e-9:
                raise ValueError(
                    'require straight_start_distance <= observation_distance - observation_tolerance, got '
                    f"{self.near:.2f} > {self.p('observation_distance'):.2f}-"
                    f"{self.p('observation_tolerance'):.2f}")
            if self.p('dock_tolerance') >= self.target:
                raise ValueError('dual dock_tolerance must be smaller than target')
            if not (0.3 <= float(node._p('dual.align_tolerance_deg')) <= 10):
                raise ValueError('dual.align_tolerance_deg must be in [0.3, 10]')

    def p(self, name):
        return float(self._node._p('dual.' + name))

    @property
    def target(self):
        return float(self._node._p('dual.dock_distance'))

    @property
    def near(self):
        # Compatibility: the old name is now ONLY the yaw_cap far/near split.
        return float(self._node._p('dual.straight_start_distance'))

    @property
    def straight_envelope(self):
        """墙码深度上界, 之内视为"直行段已归位" —— 桩码垂直离场放行与桩码
        丢失闭锁共用这一个数, 二者是同一个判断的两半。

        原来两处都用 near (= obs - tol = 1.70), 而 approach 的提交发生在观察
        窗内任意处 (obs ± tol, 即 1.70~1.90)。于是 1.78m 提交直行的狗落进一
        个死区: 放行要求 z ≤ 1.70, 而走到 1.70 的唯一办法是先前进 —— 前进
        又要放行。2026-09-11 现场就死在这里 (桩码底边余量已被吃到 <10px,
        bearing ≈ 1°, J ≈ 0.03, 直行本可成功), 三窗耗尽判 MOTION_FAILED。
        提到窗上沿后整个直行段连续可行, 同时保留硬距离上界: 远场 (站位窗外)
        的 stage 陈旧不得借此放行。
        闭锁必须跟着一起提: 若桩码可在 1.70~1.90 合法离场而闭锁仍只认
        ≤ 1.70, 那段离场就会落进 'pile missing outside qualified final entry'
        硬失败 —— 放行制造出的新失效模式。
        """
        return self.p('observation_distance')+self.p('observation_tolerance')

    @property
    def steering_stop(self):
        """纯直行区上界 (墙码光学 z): 之内禁横移, 转向按盈亏平衡门槛收紧。

        等价于把 locked 的纯直行行为提前到这个距离 —— 区别只在桩码仍可见时
        approach 不再纠偏。近场调方向越调越不准: bearing 的相机力臂增益
        1+t_x/z 在 0.6m 处已 1.33, 底盘 ~2° 停止滞后与单步命令同量级。

        但"不准"不等于"不做": 2026-09-12 现场证明整体停摆会让 bearing 一路
        涨到 11°、终点横偏 91mm。区内转向现在交给 _bearing_tol 的门槛裁决
        (小的不纠、大的纠、贴到停泊点自动禁转), 横移仍然整体禁止 —— 横移没有
        对应的收益模型, 且近场横移正是 2026-09-11 那次失效的成因。
        """
        return self.p('steering_stop_distance')

    @property
    def heading_committed(self):
        """直行航向是否有新鲜证据。两个独立来源, 任一即可:

        ① progress + qualified_ns —— 一步合格直行真正执行完成 (最强证据);
        ② committed_ns —— approach 阶段两码持住对准 (aligned + align_hold_sec),
           也正是 observe→approach 那一跳所依据的同一份证据。

        首步只有 ②: 要求 ① 就等于"要先走一步才准走第一步"。两者都由纠偏
        撤销 (action_started / observe), 预测本身永远不产生证据。
        """
        window = self.p('qualification_sec')*1e9
        return any(stamp and self.stamp-stamp <= window
                   for stamp in (self.qualified_ns if self.progress else 0,
                                 self.committed_ns))

    @property
    def yaw_cap(self):
        """按距离排的单步转向上限: 远场大步收敛, 近场小步精调。

        原来是 min(3°, yaw_step_deg) —— 一个写死的硬上限, 把配置项变成了装饰。
        3° 在近场是对的 (步子小, 过冲代价低), 在远场是灾难: 底盘停止滞后约
        2°, 与命令同量级, turn_lead 的 0.5*target 钳位会让每步只转 target/2,
        于是 20° 偏差要 ~30 步 × 2.4s ≈ 70s, 顶着 observe_timeout 走。
        远场放到 yaw_step_deg (6~8°) 后滞后只占一小部分, 提前量按 speed 正常
        生效, 单步真正转到位。

        分界用墙码光学 z 与 near (= straight_start_distance): 没有量测时保守取
        近场细步。放大上限不牺牲安全 —— visible() 会逐段采样整条转向轨迹, 会
        把 tag 甩出视野的大步直接否掉, 而 cap/2、cap/4 的小候选依然在枚举里。
        """
        coarse, fine = self.p('yaw_step_deg'), self.p('yaw_fine_step_deg')
        far = self.wall is not None and self.wall[2] > self.near
        return math.radians(coarse if far else fine)

    @property
    def pile_tag_id(self):
        return self._pile_tag_id

    @property
    def pile_frame(self):
        return f"tag{self._node._p('tag.family')}:{self._pile_tag_id}"

    @property
    def cam_offset_known(self):
        return self.r is not None

    def effective_dock_distance(self):
        return self.target if self.enabled else float(self._node._p('dock_target.distance'))

    def set_extrinsics(self, translation, quaternion):
        r = rotation(quaternion)
        if not all(math.isfinite(v) for v in translation):
            raise ValueError('nonfinite camera translation')
        tol = math.radians(self.p('mount_tolerance_deg'))
        # Optical right must point base-right horizontally (zero roll), optical
        # forward projected on the ground must point base-forward. Pitch is OK.
        if (r[0][2] < 0.2 or abs(math.atan2(r[1][2], r[0][2])) > tol
                or abs(r[2][0]) > math.sin(tol) or r[1][0] > -math.cos(tol)):
            raise ValueError('camera optical horizontal axis / roll incompatible with straight docking')
        self.r, self.t = r, tuple(translation)

    def reset(self):
        self.stage = 'acquire'
        self.failure = ''
        self.complete = False
        self.reverse_total = 0.0
        self.reverse_actions = 0
        # 站位守卫的后退独立计账 (standoff_reverse_*), 不与脱困/找桩码预算共账:
        # 把站位拉回观察窗是常态操作, 吃掉脱困预算会在真脱困时误报耗尽。
        self.standoff_total = 0.0
        self.standoff_actions = 0
        self.actions = 0
        self.progress = False
        # 对准成立且桩码光学 z 到达锁定距离后置位 —— 节点据此执行切站立
        # (匍匐进不了桩底座), 站立确认后由节点清除。
        self.request_stand = False
        self._travel_qualification = 0
        self.qualified_ns = 0
        # 直行承诺时刻 (两码持住对准): 桩码垂直离场放行的第二个证据来源,
        # 见 heading_committed。与 qualified_ns 同生共死, 任何纠偏都撤销。
        self.committed_ns = 0
        self.started_ns = 0
        self.observe_started_ns = 0
        self.no_candidates = 0
        # locked 连续转向计数: 任何 locked 前进清零 (见 plan_dual locked 分支)
        self._locked_turns = 0
        self.feedback_bad = 0
        self.feedback_anchor = None
        self.feedback_count = 0
        self.feedback_pending = None
        self.active_feedback = None
        self.pending_metrics = None
        self.settle_until_ns = 0
        self.last_seen_stamp = 0
        self.pending_stamp = 0
        self.pending_aligned = False
        self.pending_relaxed = False
        self.pending_reverse_kind = ''
        self.reset_filter()

    def reset_filter(self):
        self.wall = self.pile = None
        self.stamp = 0
        self.frames = self.wall_frames = 0
        self.hold_ns = 0
        self.missing_ns = 0
        self.window_ns = 0

    def stopped(self, now):
        self.reset_filter()
        self.settle_until_ns = now + int(max(1.5, self.p('settle_sec'),
            float(self._node._p('posture.static_settle_sec'))) * 1e9)

    def fresh(self, now, stamp=None):
        stamp = self.stamp if stamp is None else stamp
        return stamp > 0 and 0 <= now-stamp <= self.p('fresh_sec')*1e9

    def invalidate(self):
        self.reset_filter()

    def observe(self, stamp, now, wall, pile=None, pile_missing=False, moving=False):
        """Intake an exact-time TF pair. Return whether wall is accepted."""
        if moving or stamp <= self.last_seen_stamp or stamp < self.settle_until_ns:
            return False
        self.last_seen_stamp = stamp
        if not self.fresh(now, stamp) or wall is None:
            self.invalidate()
            return False
        if self.stage == 'locked':
            pile = None  # Reappearance cannot change locked commands or veto wall depth.
        for point in (wall, pile):
            if point is not None and (len(point) != 3 or not all(math.isfinite(v) for v in point)
                                      or point[2] <= 0):
                self.invalidate()
                return False
        continuous = self.fresh(stamp)
        if not continuous:
            self.reset_filter()
        if not self.window_ns:
            self.window_ns = stamp
        def moved(a, b):
            return a is not None and b is not None and math.dist(a, b) > self.p('stable_position_m')
        if moved(wall, self.wall) or moved(pile, self.pile):
            self.frames = self.wall_frames = 0
            self.hold_ns = self.missing_ns = 0
        self.wall, self.pile, self.stamp = tuple(wall), pile, stamp
        self.wall_frames += 1
        if pile is None:
            self.frames = 0
            self.hold_ns = 0
            if pile_missing:
                self.missing_ns = self.missing_ns or stamp
            else:
                self.missing_ns = 0
            return True
        self.missing_ns = 0
        self.frames += 1
        aligned = self.aligned(wall, pile)
        self.hold_ns = (self.hold_ns or stamp) if aligned else 0
        if not aligned:
            self.qualified_ns = self.committed_ns = 0
        return True

    @property
    def tol(self):
        return math.radians(float(self._node._p('dual.align_tolerance_deg')))

    def metrics(self, wall=None, pile=None):
        return geometry(self.wall if wall is None else wall,
                        self.pile if pile is None else pile, self.r, self.t, self.tol)

    def aligned(self, wall, pile):
        if self.r is None or wall is None or pile is None:
            return False
        try:
            bearings, theta, e, _ = self.metrics(wall, pile)
            # theta 门 = 对准门本身 (曾是 min(tol,1°)): 站立式下入场的航向残差
            # 由直行段 pure-pursuit 每停归零墙码 bearing 收敛 (横偏按 0.38 收缩
            # 到接触点), 1° 硬门只会把 bearing/e 都已合格、仅 theta 1~3° 的局面
            # 卡死在"纠偏无改善→振荡失败"里 —— 2026-09-11 现场: bearings
            # +1.9/+1.9、e≈10mm、theta≈1.5°, 横移执行误差与 2° 停止滞后同量级,
            # 三窗无净改善 → MOTION_FAILED, 而此时直行即可。两码 bearing 反号的
            # 大 theta 仍被各自 ≤tol 的 bearing 门配出 theta≤tol 挡住, 对准门
            # 没有松成摆设; 平行横偏由 e 门独立把守。
            return (all(abs(b) <= self.tol for b in bearings)
                    and abs(theta) <= self.tol
                    and abs(e) <= self.p('lateral_tolerance_m'))
        except ValueError:
            return False

    def pending_valid(self, now):
        # 复检必须用当初放行这条计划的那把尺子。脱困后退是在包络已被突破的
        # 位姿上、经 improving() (只要求不恶化任何一条边) 放行的; 若这里改用
        # 严格 visible() 复检, 它必然失败 —— fraction 0 就是那个已经出界的当
        # 前位姿。后果是活锁而不是报错: 规划发出后退 → 复检否掉 → stopped()
        # 重置滤波并把 settle 再推 1.5s → 重新采帧 → 规划同一条后退 …… 计划
        # 编号原地不动 (actions 不增), 现场只看到"后退脱困"告警每 2s 刷一次,
        # 底盘一步不动, 直到外层 approach_timeout 兜底。
        # 同理, "仍然对准"只在当初确实看着两码规划时才是可检验的断言。locked
        # 下 observe() 强制 pile=None, 而 aligned() 在无桩码时恒 False —— 若无
        # pending_pile 这一条豁免, _advance 发出的每一步 locked 前进 (aligned=
        # True) 都会被复检否掉, 又是一个原地打转的活锁: 丢弃 → stopped() 推
        # settle → 重测 → 同一步 → 丢弃, actions 不增, max_actions 永不触发,
        # 狗在直行阶段一步不动直到外层超时。真正的桩码丢失仍被下面最后一条
        # 子句抓住 (pending_pile 非 None 而 self.pile 为 None → 重新规划)。
        gate = self.improving if self.pending_relaxed else self.visible
        return (self.fresh(now, self.pending_stamp) and self.fresh(now)
                and not self.failure and self.stamp >= self.pending_stamp
                and self.wall is not None
                and gate(self.pending_plan)[0]
                and (not self.pending_aligned or self.pending_pile is None
                     or self.aligned(self.wall, self.pile))
                and math.dist(self.wall, self.pending_wall) <= self.p('stable_position_m')
                and ((self.pile is None and self.pending_pile is None)
                     or (self.pile is not None and self.pending_pile is not None
                         and math.dist(self.pile, self.pending_pile) <= self.p('stable_position_m'))))

    def action_started(self, plan, now):
        self.actions += 1
        self.active_feedback = self.pending_metrics
        self._node.get_logger().info(
            f'dual action START #{self.actions} stage={self.stage} '
            f'wall_xyz={self.pending_wall} pile_xyz={self.pending_pile} '
            f'action={plan} prediction={self.pending_metrics}')
        if plan.jog_distance < 0:
            # 落账按计划发出时登记的那本账 (pending_reverse_kind): 计划被
            # pending_valid 丢弃时不计账, 只有真正起步的 action_started 才记。
            if self.pending_reverse_kind == 'standoff':
                self.standoff_total += abs(plan.jog_distance)
                self.standoff_actions += 1
            else:
                self.reverse_total += abs(plan.jog_distance)
                self.reverse_actions += 1
        self._travel_qualification = 0
        if self.pending_aligned and plan.jog_distance > 0 and not plan.lateral_distance:
            self._travel_qualification = self.pending_stamp
        elif plan.turn_angle or plan.lateral_distance:
            # A correction after qualified travel invalidates that heading.
            self.qualified_ns = self.committed_ns = 0
        self.pending_aligned = False
        self.pending_reverse_kind = ''

    def action_completed(self):
        """Commit progress only after the odometry executor reports completion."""
        self.feedback_pending = self.active_feedback
        self.active_feedback = None
        if self._travel_qualification:
            self.progress = True
            self.qualified_ns = self._travel_qualification
        self._travel_qualification = 0

    def _fail(self, reason):
        self.failure = reason
        return None

    def _emit(self, plan, aligned=False, relaxed=False, reverse_kind=''):
        strict, margins = self.visible(plan)
        safe = strict
        if not safe and relaxed:
            safe, margins = self.improving(plan)
        if not safe:
            return self._no_candidate('no visible translation candidate')
        self.no_candidates = 0
        correction = bool(plan.turn_angle or plan.lateral_distance)
        try:
            before = self.metrics() if self.pile is not None else None
            after = self.metrics(*self.predicted_pair(plan)) if before else None
        except ValueError as exc:
            return self._no_candidate('invalid predicted geometry: ' + str(exc))
        self.pending_metrics = dict(before=before, after=after, margins=margins,
                                    correction=correction)
        self.pending_plan = plan
        self.pending_stamp = self.stamp
        self.pending_wall, self.pending_pile = self.wall, self.pile
        self.pending_aligned = aligned
        # 记下这条计划是走宽松门 (improving) 还是严格门 (visible) 放行的 ——
        # pending_valid 复检必须用同一把尺子, 否则脱困后退永远发不出去。
        self.pending_relaxed = bool(relaxed and not strict)
        # 后退计划登记所属预算 ('' / 'acquisition' → 主账, 'standoff' → 站位账),
        # 与 pending_aligned 一样只由 action_started 消费一次。
        self.pending_reverse_kind = reverse_kind
        return [plan]

    def predicted_pair(self, plan, fraction=1.):
        fwd = 0. if plan.lateral_distance else plan.jog_distance
        return tuple(predict(p, self.r, self.t, fwd*fraction,
                             plan.lateral_distance*fraction, plan.turn_angle*fraction)
                     if p is not None else None for p in (self.wall, self.pile))

    @property
    def required_margin(self):
        return self.p('visibility_margin_px')+self.p('visibility_sample_pad_px')

    def visible(self, plan, full=False):
        """Sampled visibility verdict plus the worst margin on each image edge.

        full=True keeps sampling past the first violation so the returned
        margins describe the whole path, not just its first bad sample. Only
        the recovery predicate needs that; the normal gate short-circuits.
        """
        if self.camera is None:
            return False, None
        # 纯直行区内不再要求"两码对准": 那一条前提原是为了只在真要直行时才
        # 宽恕桩码垂直离场, 而 z ≤ steering_stop 之后航向已被策略冻结 ——
        # 未对准既不可被采纳为纠偏, 拦下这一步前进也换不回任何东西, 只会把
        # 一场合法的纯直行变成三窗耗尽 (2026-09-11 那个失效模式的近场版本)。
        # 航向证据仍然要 (绝不按一个从未验证过的航向盲走), 包络上界仍然要。
        allow_exit = (self.stage == 'approach' and self.heading_committed
                      and self.wall[2] <= self.straight_envelope
                      and plan.jog_distance > 0
                      and not plan.lateral_distance and not plan.turn_angle
                      and (self.wall[2] <= self.steering_stop
                           or self.aligned(self.wall, self.pile)))
        margins = [float('inf')]*4
        required = self.required_margin
        ok = True
        for i in range(int(self.p('visibility_samples'))+1):
            for index, pt in enumerate(self.predicted_pair(plan, i/int(self.p('visibility_samples')))):
                if pt is None:
                    continue
                # 终局整段 (continuous = 一个动作走完到停泊点) 对桩码整体免检,
                # 不只是下沿: 狗是骑跨在桩上充电的 —— 机体下方的电极片对准桩上
                # 的电极片, 所以到位时桩码必然在机体下方、必然出画。要求它在
                # 路径终点仍可见, 等于要求一个"成功时必然不成立"的条件。
                # (桩码贴近时保守包围立方体的投影还会横向炸开, 于是连 allow_exit
                # 保留的左右边也会否掉整段 —— 那不是"走错了", 那是到位了。)
                # 这一段结束即停泊: 下一个窗口只做容差判定或修剪, 不再需要桩码,
                # 所以这里放弃的也不是任何后续要用的量测。
                if index == 1 and allow_exit and plan.continuous:
                    continue
                try:
                    bounds = self.camera.bounds(pt, float(self._node._p(
                        'dual.wall_tag_size' if index == 0 else 'dual.pile_tag_size')))
                except ValueError:
                    # Near-plane crossing has no finite enclosure to report.
                    # 这里不给桩码开 allow_exit 的口子 —— 不是不该放行 (骑跨
                    # 充电, 桩码到位时本就在机体下方), 而是到不了这里: 终局
                    # 整段在上面就整体免检、根本不调 bounds; 任何更短的前进,
                    # 采样点都细到必然先落进"横向炸开窗" (x≈0 时
                    # z∈(0.035, 0.053), 包围立方体投影在 z→半径 时发散),
                    # 被 allow_exit 保留的左右边先判否。写一个到不了的分支
                    # 只会让人以为近场逐步走已经宽恕过桩码 —— 它没有。
                    if not full:
                        return False, tuple(margins)
                    ok = False
                    continue
                margins = [min(a,b) for a,b in zip(margins,bounds)]
                check = bounds[:2] if index == 1 and allow_exit else bounds
                if min(check) < required:
                    if not full:
                        return False, tuple(margins)
                    ok = False
        return ok, tuple(margins)

    def current_margins(self):
        """Per-edge margins of the pose as observed, with no action applied.

        Fraction 0 of every sampled path is this pose, so once it violates the
        envelope every candidate does too, whichever way it moves.
        """
        if self.camera is None or self.wall is None:
            return None
        margins = [float('inf')]*4
        for index, pt in enumerate((self.wall, self.pile)):
            if pt is None:
                continue
            try:
                bounds = self.camera.bounds(pt, float(self._node._p(
                    'dual.wall_tag_size' if index == 0 else 'dual.pile_tag_size')))
            except ValueError:
                return (float('-inf'),)*4
            margins = [min(a,b) for a,b in zip(margins,bounds)]
        return tuple(margins)

    def envelope_violated(self):
        cur = self.current_margins()
        return cur is not None and min(cur) < self.required_margin

    def improving(self, plan):
        """Recovery gate: the pose is already out, so demand only that the move
        worsen no edge. A reverse increases both optical depths, carrying both
        tags back toward the principal point, and satisfies this by geometry."""
        base = self.current_margins()
        _, after = self.visible(plan, full=True)
        if base is None or after is None:
            return False, after
        return min(after) >= min(base), after

    def exit_report(self):
        """桩码垂直离场放行的逐条前提 —— 现场"为什么前进被否"的唯一答案。

        余量表只说桩码底边剩几 px, 不说那几 px 该不该拦人。2026-09-11 的日志
        里余量一路掉到 +8px 然后三窗判死, 而当时缺的其实是放行 (bearing 1°、
        J 0.03, 直行就能成)。四条前提逐条打出来, 下一次一眼就能看到是哪条。
        """
        checks = (('stage=approach', self.stage == 'approach'),
                  ('航向有证据', bool(self.heading_committed)),
                  (f'z<=直行包络{self.straight_envelope:.2f}m',
                   self.wall is not None and self.wall[2] <= self.straight_envelope),
                  # 纯直行区内这一条自动成立 (航向已被策略冻结, 见 visible)。
                  (f'两码对准或z<=纯直行{self.steering_stop:.2f}m',
                   self.aligned(self.wall, self.pile)
                   or (self.wall is not None and self.wall[2] <= self.steering_stop)))
        bad = [name for name, ok in checks if not ok]
        return '桩码垂直离场=' + ('放行' if not bad else '不放行(缺 '+'/'.join(bad)+')')

    def margin_report(self):
        """Per-tag edge margins of the current pose, for failure logs."""
        if self.camera is None:
            return 'margins=<no CameraInfo>'
        parts = [f'required={self.required_margin:.0f}px']
        for name, pt, key in (('wall', self.wall, 'wall_tag_size'),
                              ('pile', self.pile, 'pile_tag_size')):
            if pt is None:
                parts.append(f'{name}=<none>')
                continue
            try:
                b = self.camera.bounds(pt, float(self._node._p('dual.'+key)))
            except ValueError as exc:
                parts.append(f'{name}=<{exc}>')
                continue
            parts.append(f'{name} L/R/T/B={b[0]:.0f}/{b[1]:.0f}/{b[2]:.0f}/{b[3]:.0f}px')
        parts.append(self.exit_report())
        return ' '.join(parts)

    def _no_candidate(self, reason):
        # Consume the window; timer retries cannot count the same observations.
        self.no_candidates += 1
        self._node.get_logger().info(
            f'dual STOP: {reason}; window={self.no_candidates}; {self.margin_report()}')
        self.reset_filter()
        if self.no_candidates >= int(self.p('no_candidate_windows')):
            return self._fail(reason + '; independent stable-window budget exhausted')
        return None

    def _correction(self, metrics):
        bearings, theta, e, score = metrics
        candidates = []
        # 每轮枚举的逐候选台账: 现场唯一能回答"为什么选了这个/为什么一个都
        # 没选"的证据。被否的原因分三类, 必须可区分 —— 不可见(包络)、几何无
        # 解(metrics 抛错)、改善不足(附带 ΔJ), 三者的现场处置完全不同。
        ledger = []
        for kind, cap, residual, minimum in (
                ('yaw', self.yaw_cap, theta, .005),
                ('lateral', self.p('lateral_step'), e/math.cos(theta), self.p('min_lateral_m'))):
            amounts = [cap, cap/2, cap/4, min(cap,abs(residual)), minimum]
            seen = set()
            for size in amounts:
                for sign in (1., -1.):
                    amount = size*sign
                    if size < minimum or size > cap or round(amount,10) in seen:
                        continue
                    seen.add(round(amount,10))
                    tag = (f'{kind}{math.degrees(amount):+.2f}deg' if kind == 'yaw'
                           else f'{kind}{amount*1e3:+.0f}mm')
                    plan = (ActionPlan(kind='yaw',turn_angle=amount) if kind == 'yaw' else
                            ActionPlan(kind='forward',jog_distance=abs(amount),lateral_distance=amount))
                    ok, margins = self.visible(plan)
                    if not ok:
                        worst = min(margins) if margins else float('nan')
                        ledger.append(f'{tag}:不可见(最差边{worst:+.0f}px)')
                        continue
                    try:
                        nxt = self.metrics(*self.predicted_pair(plan))[-1]
                    except ValueError as exc:
                        ledger.append(f'{tag}:几何无解({exc})')
                        continue
                    if score-nxt >= self.p('score_improvement'):
                        ledger.append(f'{tag}:J{nxt:.3f}(改善{score-nxt:+.3f})')
                        candidates.append((nxt,size/cap,len(candidates),plan))
                    else:
                        ledger.append(f'{tag}:改善不足({score-nxt:+.4f})')
        self._node.get_logger().info(
            f'dual 规划 #{self.actions+1} stage={self.stage}: '
            f'方位 墙={math.degrees(bearings[0]):+.2f}deg 桩={math.degrees(bearings[1]):+.2f}deg '
            f'| 航向偏差 theta={math.degrees(theta):+.2f}deg 横偏 e={e*1e3:+.0f}mm '
            f'(e>0=桩在狗左侧, 应左移) | J={score:.3f} '
            f'门槛 对准={self.p("align_tolerance_deg"):.1f}deg 改善={self.p("score_improvement")}'
            f' 转向上限={math.degrees(self.yaw_cap):.1f}deg'
            f'({"远场" if self.wall[2] > self.near else "近场"} z={self.wall[2]:.2f}m)'
            f' | 候选 [{", ".join(ledger) if ledger else "无"}]')
        if not candidates:
            # 当前位姿已在包络之外 (桩码近距压到画面下边是典型原因): 每条候选
            # 路径的 fraction 0 就是当前位姿, 于是转/移候选恒不可行, 而唯一能
            # 脱困的后退又不在候选集里 —— 直接判失败会把可恢复的局面做死。
            # 后退单调增大两码光学 z, 把两码带回主点附近, 由 improving() 逐边
            # 校验且仍受后退预算与里程计看门狗约束。
            if self.envelope_violated():
                self._node.get_logger().warn(
                    'dual 可见性包络已被当前位姿突破 (' + self.margin_report()
                    + ') — 转/移候选恒不可行, 后退脱困')
                return self._reverse(relaxed=True)
            return self._no_candidate('no executable improving visible yaw/lateral candidate')
        best = min(c[0] for c in candidates)
        # Near ties prefer smaller normalized actions, then stable generation order.
        chosen = min((c for c in candidates if c[0] <= best+self.p('score_improvement')),
                     key=lambda c: (c[1],c[2]))
        return self._emit(chosen[-1])

    def _check_feedback(self, metrics):
        feedback, self.feedback_pending = self.feedback_pending, None
        if not feedback or not feedback['before']:
            return True
        before, after = feedback['before'][-1], metrics[-1]
        self._node.get_logger().info(f'dual visual feedback J={before:.5f}->{after:.5f} '
                                    f'improvement={before-after:+.5f}')
        if not feedback['correction']:
            self.feedback_anchor = None
            self.feedback_count = self.feedback_bad = 0
            return True
        self.feedback_bad = self.feedback_bad+1 if before-after < self.p('feedback_min_improvement') else 0
        if self.feedback_anchor is None:
            self.feedback_anchor = before
        self.feedback_count += 1
        limit = int(self.p('feedback_fail_windows'))
        if self.feedback_bad >= limit or (self.feedback_count >= limit and
                self.feedback_anchor-after < self.p('feedback_min_improvement')):
            self._fail('dual visual correction no progress / oscillation')
            return False
        if self.feedback_count >= limit:
            self.feedback_anchor, self.feedback_count = after, 0
        return True

    def _reverse(self, relaxed=False, why='acquisition'):
        # why 区分两种耗尽与两本账: 'acquisition' = 找桩码退不出来/脱困, 记主账
        # (reverse_limit/reverse_count); 'standoff' = 站位守卫在跟匍匐爬行赛跑
        # 且没跑赢, 记独立账 (standoff_reverse_*) —— 把站位拉回观察窗是常态
        # 操作, 与脱困共账会在真脱困时误报耗尽。现场下一个问题必然是
        # "为什么退完了", 失败串必须自己回答。'reverse' 一词在两种串里都
        # 保留 (判据依赖它)。
        if why == 'standoff':
            total, actions = self.standoff_total, self.standoff_actions
            limit = self.p('standoff_reverse_limit')
            count = int(self.p('standoff_reverse_count'))
        else:
            total, actions = self.reverse_total, self.reverse_actions
            limit = self.p('reverse_limit')
            count = int(self.p('reverse_count'))
        if (actions >= count or
                total + self.p('reverse_step') > limit + 1e-9):
            return self._fail(f'dual reverse {why} budget exhausted')
        return self._emit(ActionPlan(kind='forward', jog_distance=-self.p('reverse_step')),
                          relaxed=relaxed, reverse_kind=why)

    def plan_dual(self, wall, base_type, planner, now_ns):
        now = now_ns
        self.started_ns = self.started_ns or now
        if self.failure or self.complete:
            return None
        if not self.cam_offset_known:
            return self._fail('dual requires calibrated optical-to-base extrinsics')
        if self.actions >= int(self.p('max_actions')):
            return self._fail('dual action budget exhausted')
        if self.stage == 'acquire' and now-self.started_ns > self.p('acquire_timeout_sec')*1e9:
            return self._fail('dual acquisition timeout')
        if self.stage == 'observe' and now-self.observe_started_ns > self.p('observe_timeout_sec')*1e9:
            return self._fail('dual observation timeout')
        if self.camera is None:
            if now-self.started_ns > self.p('camera_wait_sec')*1e9:
                return self._fail('dual CameraInfo unavailable within wait budget')
            return None
        if not self.fresh(now):
            return None
        # locked (切站立后) 豁免: 姿态切换瞬间相机位姿变化会让墙码 z 读数
        # 跳变几厘米, 直行阶段的 z 只用于推进, 不会突然变近。
        # 合格终局同样豁免: 连续直行后若冲过停泊点且 stage 仍是 approach
        # (区内未对准入口路径), 这个检查跑在 pile 丢失闭锁之前, 会把本应
        # "闭锁 → _advance 回退修剪" 的局面直接判死。豁免前提与闭锁分支
        # 完全一致 (progress + 资格新鲜 + 直行包络内) —— 无资格的异常
        # 贴近照旧判失败, 那才是这条检查要拦的东西。
        if (self.stage != 'locked'
                and self.wall[2] < self.target-self.p('dock_tolerance')
                and not (self.progress and self.qualified_ns
                         and now-self.qualified_ns
                         <= self.p('qualification_sec')*1e9
                         and self.wall[2] <= self.straight_envelope)):
            return self._fail('optical wall depth overshoot; no reverse retry')
        if now < self.settle_until_ns or self.wall_frames < max(3, int(self.p('min_frames'))):
            return None
        d = self.wall[2]
        # 整场余量趋势 (现场 61.6→49.3→39.0→25.8 那条序列本该在破包络前就看见);
        # 站位后退与主后退分开记, 耗尽时两条失败串才能各自回答"为什么退完了"。
        margins = self.current_margins()
        extra = (f' standoff_rev={self.standoff_total:.2f}m' if self.standoff_total else '')
        if margins is not None:
            extra += f' min_margin={min(margins):.0f}px'
        self._node.get_logger().info(
            f'dual stage={self.stage} optical_wall_z={d:.3f}m '
            f'wall_bearing={math.degrees(math.atan2(self.wall[0], d)):+.2f}deg '
            f'pile_bearing={math.degrees(math.atan2(self.pile[0], self.pile[2])) if self.pile else float("nan"):+.2f}deg '
            f'frames={self.frames} reverse={self.reverse_total:.2f}m{extra}',
            throttle_duration_sec=2.0)
        if self.stage == 'locked':
            # 直行阶段的墙码 bearing 微调 (设计决定 1): 每停归零相机 bearing
            # 即对墙码做 pure pursuit —— 不只止住偏航误差增长, 还把横向误差按
            # 0.5/1.3 ≈ 0.38 收缩到接触点。None = 纯前进 (容差内 / 无可行候选 /
            # 连续转向达上限), 任何前进清零连续转向计数。
            # 纯直行区内这条**仍然活着**, 只是门槛随接近收紧 (_bearing_tol) ——
            # 它与区内的"行进中航向保持"是互补两层: 这条在停稳时纠视觉看得见的
            # 残余 (航向 + 横偏), 那条在行进中守住视觉看不见的 yaw 漂移。
            turn = self._locked_bearing(d)
            if turn is None:
                self._locked_turns = 0
                return self._advance(d)
            return turn
        if self.pile is None:
            missing_time = (self.stamp-self.missing_ns)*1e-9 if self.missing_ns else 0
            if (self.stage == 'approach' and self.progress and self.qualified_ns
                    and now-self.qualified_ns <= self.p('qualification_sec')*1e9
                    and d <= self.straight_envelope
                    and missing_time >= self.p('missing_confirm_sec')):
                self.stage = 'locked'
                self._node.get_logger().info(
                    f'dual heading LOCKED: 合格直行段桩码丢失 (墙码 z={d:.3f}m '
                    f'≤ 直行包络 {self.straight_envelope:.2f}m, 丢失确认 '
                    f'{missing_time:.1f}s) — 此后只按墙码直行')
                return self._advance(d)
            if self.stage == 'acquire' and missing_time >= self.p('missing_confirm_sec'):
                # 桩码 5cm 可见性半径有限, 不可见有两种原因, 按墙码距离分流 ——
                # 太远 (d > 观察点) → 匍匐前进逼近, 走进桩码检测半径;
                # 已在观察距离内仍不可见 → 初始太近 (出视野), 后退找回。
                obs = self.p('observation_distance')
                if d > obs + self.p('observation_tolerance'):
                    return self._emit(ActionPlan(kind='forward', jog_distance=min(
                        self.p('forward_step'), (d-obs)/self.r[0][2])))
                return self._reverse()
            if self.window_ns and now-self.window_ns > self.p('missing_timeout_sec')*1e9:
                return self._fail('pile missing / invalid outside qualified final entry')
            return None
        if self.frames < max(3, int(self.p('min_frames'))):
            return None
        if self.stage == 'acquire':
            self.stage = 'observe'
            self.observe_started_ns = now
            # 一次性: 进入 observe 那一刻两码的逐边余量 —— "站位处桩码还剩
            # 多少余量"就是这一行。余量不足的调参优先级: ① 抬
            # observation_distance; ② 相机下俯 (唯一真正增加余量的做法);
            # ③ 最后才、且不情愿地动 visibility_margin_px (只买到 ~2mm)。
            self._node.get_logger().info('dual 进入 observe: ' + self.margin_report())
        try:
            metrics = self.metrics()
        except ValueError as exc:
            return self._fail(str(exc))
        if not self._check_feedback(metrics):
            return None
        # ── 站位守卫: observe 阶段的站位距离不因"正在纠偏"而失管 ──────────
        # 缺口: 原来 observe 的站位检查排在 aligned() 分支之后、held 门之后,
        # 于是只要还在纠偏 (整场纠偏就是常态), 距离一次都没人看。现场墙码 z
        # 从 1.68 一路爬到 1.25, 穿过 1.5±0.1 观察窗而规划器毫无察觉 —— 直到
        # 桩码底边余量被吃穿 (61→49→39→26→+8px), 包络破了才发现。
        #
        # 爬行本身是匍匐步态原地转向时机体整体前移 (每个动作 4.5~7cm, 与动作
        # 类型/大小无关, 执行器侧 _action_linear=0), 那是底盘侧的事; 这里只
        # 负责让站位重回观察窗, 不假装能消除扰动。
        #
        # 只守近端, 不守远端: 太近会把矮桩码压出画面下沿 (唯一真实的失效
        # 模式), 而太远无害 —— 两码都还在视野里, 对准成立后 held 分支自会
        # 推进。纠偏期间主动前进则相反: 航向还没对, 前进就是沿错误方向走远。
        #
        # 只在 observe: approach 是有意逼近 target(0.5m), locked 是锁定直行,
        # 在那两个阶段挂 1.4m 的站位门槛会直接把接近永久堵死。
        if self.stage == 'observe':
            obs, tol = self.p('observation_distance'), self.p('observation_tolerance')
            if d < obs-tol:
                self._node.get_logger().warn(
                    f'dual 站位守卫: 墙码 z={d:.3f}m < 观察窗下沿 {obs-tol:.2f}m '
                    f'({obs:.2f}±{tol:.2f}) — 先后退拉回站位再纠偏。'
                    f'匍匐转向整体前移是已知扰动, 站位后退预算 '
                    f'{self.standoff_actions}/{int(self.p("standoff_reverse_count"))} '
                    f'{self.standoff_total:.2f}/{self.p("standoff_reverse_limit"):.2f}m')
                # 包络若已被突破, 严格 visible() 会否掉一切候选 (fraction 0
                # 就是当前位姿), 此时必须走 improving() 宽松门, 否则守卫自己
                # 就把唯一的出路堵死, 退化成 _no_candidate 计窗口。
                return self._reverse(relaxed=self.envelope_violated(),
                                     why='standoff')
        if not self.aligned(self.wall, self.pile):
            # 纯直行区 (z ≤ steering_stop): 禁转向、禁横移, 未对准也只前进。
            # 落到 _advance 而不是 _fail/_no_candidate —— 近场不再调方向是
            # 有意的取舍, 不是异常; 把一次本可成功的直行变成中止是错的交易
            # (2026-09-11 现场就是这么死的, 见 straight_envelope)。
            # 也不走 _correction 的"无候选→后退脱困": 近场破包络正是桩码被
            # 压出画面下沿的常态, 后退只会白退一场。
            if d <= self.steering_stop:
                self._node.get_logger().info(
                    f'dual 纯直行区 (墙码 z={d:.3f}m ≤ {self.steering_stop:.2f}m): '
                    f'不再调方向, 直行到底 (残余 theta='
                    f'{math.degrees(metrics[1]):+.2f}deg 横偏 e={metrics[2]*1e3:+.0f}mm)',
                    throttle_duration_sec=2.0)
                return self._advance(d)
            return self._correction(metrics)
        # 对准成立 (两码 bearing ≈ 0, 正对充电桩) 且桩码到达锁定距离 → 切站立:
        # 桩码光学 z 再小就要出视野 (30cm 是匍匐视角的可见极限), 此后航向已由
        # 对准锁定 —— 节点读 request_stand 执行 stand_up, 站立后桩码必然丢失,
        # locked 只按墙码纯直行 (匍匐进不了桩底座)。
        # 仅匍匐 profile: 站立全程下桩码在 0.9m 处仍清晰可见, 不在此切站立 --
        # locked 由 approach 的合格桩码丢失闭锁进入 (near 窗内, 见 plan_dual)。
        if self.crouch and self.pile[2] <= self.p('pile_lock_distance'):
            self.stage = 'locked'
            self.request_stand = True
            self._node.get_logger().info(
                f'dual 对准成立且桩码 z={self.pile[2]:.3f}m ≤ '
                f'{self.p("pile_lock_distance"):.2f}m → 切站立 (锁定航向, 墙码直行)')
            return None
        held = self.hold_ns and self.stamp-self.hold_ns >= float(self._node._p('dual.align_hold_sec'))*1e9
        if not held:
            return None
        # 直行承诺登记在此 —— aligned() + held (两码持住对准满 align_hold_sec)
        # 刚刚双双成立, 这正是 observe→approach 那一跳所依据的同一份证据。桩码
        # 垂直离场的放行必须认它, 否则首步死锁 (见 heading_committed)。
        # 必须排在下面的 stage 切换与余量日志之前: 那行日志要如实反映本次决策
        # 用的放行状态, 否则"提交直行"那一刻的日志会说"不放行", 紧接着又发出
        # 前进 —— 现场读日志的人先信哪一句?
        self.committed_ns = self.stamp
        if self.stage == 'observe':
            obs = self.p('observation_distance')
            # 下沿 (d < obs-tol) 不在这里判: 站位守卫排在 aligned() 之前,
            # 每个 observe 窗口都已经过一遍同一门槛, 走到这里 d 必然 ≥ 下沿。
            # 在此重复一遍只会是永不执行的死代码, 误导下一个读代码的人。
            if d > obs+self.p('observation_tolerance'):
                return self._emit(ActionPlan(kind='forward', jog_distance=min(self.p('forward_step'), (d-obs)/self.r[0][2])))
            self.stage = 'approach'
            # 一次性: 提交直行那一刻的余量 —— 之后桩码将随接近离开视野,
            # 这是最后一次能看到两码逐边状态的点。
            self._node.get_logger().info(
                'dual 进入 approach (提交直行): ' + self.margin_report())
        return self._advance(d)

    def _bearing_tol(self, depth):
        """bearing 微调门槛; 纯直行区内随接近收紧, 而不是整体停摆。

        区外恒为 straight_yaw_tol_deg。区内 (depth ≤ steering_stop) 取
        "这一转到底值不值得发"的盈亏平衡点:

          收益 —— 归零 bearing 即对墙码 pure pursuit, 终点横偏由 y 收缩到
                  y·target/depth, 收益 = y·(1 − target/depth), 其中
                  y = depth·tan|b|;
          代价 —— 转向自身约 straight_yaw_lag_deg 的停止滞后残留成航向误差,
                  一路带到接触点, 横向代价 = target·sin(lag)。

        收益 > 代价 即 tan|b| > target·sin(lag)/(depth − target) —— 右边就是
        门槛。它自己会做对两件事:
          · depth → target 时 (depth−target) → 0, 门槛发散到 90°, 最后一截
            没有任何 bearing 能过门 —— 原来那条"近场禁转"的结论被保留下来,
            但落在物理正确的位置上, 而不是一条 1.0m 的硬悬崖;
          · depth 远离 target 时门槛降到 straight_yaw_tol_deg 以下, 由后者
            兜底 (max), 区内门槛因此永远不比区外更松。

        典型值 (target=0.50, lag=2.0°): z=1.00→2.0°, 0.85→2.8°, 0.75→4.1°,
        0.65→6.7°, 0.55→11.1°。现场那趟 z=0.74 时 bearing 已 6.0°, 收益
        25mm 对代价 17mm —— 该纠, 而旧代码把它整个否掉了。

        lag 是唯一的现场旋钮: 调大 = 更保守 = 区内更早停止转向。
        """
        tol = math.radians(self.p('straight_yaw_tol_deg'))
        if depth > self.steering_stop:
            return tol
        gap = depth-self.target
        if gap <= 0:
            # 已到或冲过停泊点: 转向不再有任何横向收敛通道, 纯航向损伤。
            return math.pi
        lag = math.sin(math.radians(self.p('straight_yaw_lag_deg')))
        return max(tol, math.atan2(self.target*lag, gap))

    def _locked_bearing(self, depth):
        """locked 直行的墙码 bearing 微调; 返回转向计划, None = 纯前进。

        直行阶段唯一可观测的自由度是墙码 bearing (站立视角桩码早已丢失)。
        每停把 bearing 归零就是 pure pursuit: 1.7m 处横偏 y0 的狗若每次都瞄着
        墙码走, 走的是一条指向墙码的直线, 到停泊面 (0.5m) 横偏为
        y0·0.5/1.3 ≈ 0.38·y0 —— 不止阻止偏航误差增长, 还主动收敛直行阶段
        本来观测不到的横向误差。

        沿用 _correction 的枚举-预测-过门惯用法 (无桩码即无 J, 改以预测
        |bearing| 打分)。两个符号都枚举, 符号与步长交给 predict() 裁决:
        相机在 base 上有横向力臂 (命令 2.86° 实变 bearing 3.30°, 增益
        1+t_x/z, z=0.6 时 1.33), 手写符号与 min(|b|, step) 会系统性过冲。

        无可行候选时返回 None 直行而非 _no_candidate: 把锦上添花的微调变成
        整场健康直行的中止是错的交易; 墙码仍受保护 —— 不可见的转向根本
        不会被发出。连续转向上限 (straight_yaw_max_turns) 兜住底盘 ~2°
        停止滞后导致的预测与现实偏差, 超限强制前进, 两条路径都不判失败。

        纯直行区 (z ≤ steering_stop) 内**不再整体停摆**, 改为按 _bearing_tol
        的盈亏平衡门槛收紧 —— 2026-09-12 现场推翻了原来的整体停摆: 区内
        bearing 从 +1.88° 单调涨到 +11.22° 全程无人纠, 终点横偏 ~91mm, 而
        dock_tolerance 只有 20mm。原论证 (力臂增益 + ~2° 停止滞后使单步转向
        不确定) 本身没错, 错的是把一个 ~2° 量级的不确定度拿去否决一个 11°
        量级的误差。门槛化之后这条论证以正确的形式保留: 小 bearing 仍不纠
        (纠不过噪声), 大 bearing 纠 (代价远小于收益), 且 depth → target 时
        门槛自动发散, 最后一截仍然禁转 —— 没有硬距离悬崖。

        区内的"行进中航向保持"与这条是互补的两层, 不是重复:
          · 参考量: 这里是墙码 bearing (力臂增益 1+t_x/z, z=0.6 时 1.33);
            那里是 odom/IMU yaw 增量 (旋转不产生力臂误差, 增益恒为 1)。
          · 闭环性: 这里单步开环 (停稳发一次, 下一个停看点才知道结果);
            那里 20Hz 闭环 (单周期 0.34°, 过冲下一周期就被看到)。
          · 纠的对象: 这里纠上一停视觉看见的残余误差 (含航向 + 横偏, bearing
            把两者混在一起); 那里纠行程中新产生的 yaw 漂移 —— 上一停的视觉
            物理上看不见它。现场那 91mm 里两者都有份, 少任何一层都补不齐。
        """
        b = math.atan2(self.wall[0], depth)
        tol = self._bearing_tol(depth)
        near = depth <= self.steering_stop
        if abs(b) <= tol:
            if near:
                self._node.get_logger().info(
                    f'dual 纯直行区 (墙码 z={depth:.3f}m ≤ {self.steering_stop:.2f}m): '
                    f'bearing={math.degrees(b):+.2f}deg ≤ 收紧门槛 '
                    f'{math.degrees(tol):.2f}deg, 直行 (纠它不划算)',
                    throttle_duration_sec=2.0)
            return None
        if near:
            self._node.get_logger().info(
                f'dual 纯直行区 (墙码 z={depth:.3f}m): bearing='
                f'{math.degrees(b):+.2f}deg > 收紧门槛 {math.degrees(tol):.2f}deg '
                f'— 先微调再直行 (终点横偏收益 '
                f'{depth*math.tan(abs(b))*(1-self.target/depth)*1e3:.0f}mm)',
                throttle_duration_sec=2.0)
        turns = int(self.p('straight_yaw_max_turns'))
        if self._locked_turns >= turns:
            self._node.get_logger().warn(
                f'dual locked 连续转向 {self._locked_turns}/{turns} 达上限, '
                f'强制直行 (bearing={math.degrees(b):+.2f}deg, 底盘停止滞后兜底)')
            return None
        cap = self.yaw_cap
        candidates, ledger = [], []
        seen = set()
        for size in (min(abs(b), cap), cap, cap/2, cap/4):
            for sign in (1., -1.):
                amount = size*sign
                if size < math.radians(.5) or round(amount, 10) in seen:
                    continue
                seen.add(round(amount, 10))
                plan = ActionPlan(kind='yaw', turn_angle=amount)
                ok, margins = self.visible(plan)
                if not ok:
                    worst = min(margins) if margins else float('nan')
                    ledger.append(f'{math.degrees(amount):+.2f}deg:'
                                  f'不可见(最差边{worst:+.0f}px)')
                    continue
                wall_after = self.predicted_pair(plan)[0]
                score = abs(math.atan2(wall_after[0], wall_after[2]))
                if abs(b)-score < self.p('score_improvement'):
                    ledger.append(f'{math.degrees(amount):+.2f}deg:改善不足'
                                  f'(|b|{math.degrees(score):.2f}deg)')
                    continue
                candidates.append((score, abs(amount), plan))
                ledger.append(f'{math.degrees(amount):+.2f}deg:'
                              f'|b|→{math.degrees(score):.2f}deg')
        if not candidates:
            self._node.get_logger().info(
                f'dual locked 无可行转向候选, 直行 (bearing={math.degrees(b):+.2f}deg) '
                f'[{", ".join(ledger) or "无候选"}]')
            return None
        best = min(candidates, key=lambda c: (c[0], c[1]))
        self._locked_turns += 1
        self._node.get_logger().info(
            f'dual locked bearing 微调: bearing={math.degrees(b):+.2f}deg '
            f'(tol={self.p("straight_yaw_tol_deg"):.1f}deg '
            f'cap={math.degrees(cap):.1f}deg 连续={self._locked_turns}/{turns}) '
            f'候选 [{", ".join(ledger)}]')
        return self._emit(best[-1])

    def _advance(self, depth):
        remaining = depth-self.target
        if abs(remaining) <= self.p('dock_tolerance'):
            if self.progress and self.stage in ('approach', 'locked'):
                self.complete = True
                return [ActionPlan(kind='done')]
            return self._fail('terminal depth without qualified approach')
        # No jog_min: shrink the last step rather than force an overshoot.
        # aligned 恒 True: 调用方都已在对准/锁定航向下 (_advance 只从 approach
        # 对准分支与 locked 进入) —— locked 前进同样累积航向资格, 否则切站立后
        # progress 永远 False, 终点会误判 'without qualified approach'。
        if remaining < 0:
            # 冲过停泊点 (|remaining| > dock_tolerance): 回退修剪。此前这是
            # min() 公式的隐式行为 (负数穿透 min(0.10, remaining)), 现场日志里
            # 只会看到一条莫名其妙的前进 -0.03; 现在显式分支并说明缘由。
            # 连续直行使冲过成为预期结局之一 (里程计尺度误差无处吸收), 修剪
            # 而非判失败 —— 超出 dock_tolerance 的欠/过都由停稳重测兜住。
            # 修剪单步封顶 reverse_step: 真冲过一大截 (尺度标错/量测异常) 时
            # 分步退、每步带新鲜重测, 比一次长盲退稳; 修剪走主账 (见
            # action_started), max_actions 兜住病态振荡。
            trim = max(remaining/self.r[0][2], -self.p('reverse_step'))
            self._node.get_logger().info(
                f'dual 冲过停泊点 {-remaining*100:.1f}cm, 回退修剪 {trim*100:.1f}cm')
            return self._emit(ActionPlan(kind='forward', jog_distance=trim),
                              aligned=True)
        if depth <= self.steering_stop:
            # 纯直行区: 区内禁转向/禁横移, jog 切分毫无收益只有起停开销 ——
            # 一次连续直行走完剩余距离, 终点精度交给停稳重测的修剪
            # (欠: 本函数的收缩步; 过: 上面的回退修剪)。
            # continuous 标记有两个消费者: ActionWatch 据此放宽 deadline
            # (0.45m @ 0.08m/s ≈ 5.6s, 会撞 6s 动作超时); 节点侧据此武装
            # 行进中航向保持 —— 区内一次走完与行进中守住航向是同一个决定的
            # 两半: 既然不停下来纠方向, 就得在走的过程中不让它歪掉。守的是
            # odom/IMU yaw 相对起步的增量 (连续微步闭环), 与下面 _locked_bearing
            # 禁的"用近场视觉做离散大步开环转向"不是一回事, 那条禁令未被撤销。
            # (0.45m @ 0.08m/s ≈ 5.6s, 会撞 6s 动作超时)。
            # 整段不可见则退回 forward_step 逐步走, 而不是判无候选。此时被否的
            # 只可能是墙码 (桩码在终局整段里整体免检 —— 骑跨充电, 到位时它就在
            # 机体下方, 见 visible): 墙码是修剪和终点判定唯一的依据, 它要在整段
            # 路径上都留在画里, 否则宁可逐步走、每停重测 —— 那是长期现场验证的
            # 老行为。这里多算一次 visible (十几个采样点, 可忽略), 换 _emit 的
            # 记账与无候选语义完全不动。
            run = ActionPlan(kind='forward', jog_distance=remaining/self.r[0][2],
                             continuous=True)
            if self.visible(run)[0]:
                return self._emit(run, aligned=True)
        return self._emit(ActionPlan(kind='forward', jog_distance=min(
            self.p('forward_step'), remaining/self.r[0][2])), aligned=True)


def parallax_solve(b_w: float, d_w: float, b_p: float, d_p: float,
                   min_baseline: float = 0.08, max_baseline: float = 1.0,
                   max_slide: float = 0.6, max_psi: float = 0.7):
    """双码视差解算: 由两码方位/距离解机器人横偏 Y 与航向误差 ψ。

    生成模型 (REP-103, 桩轴为 x', 机器人在轴侧偏 Y、航向偏 ψ):
        b_w = atan(-Y/d_w) - ψ
        b_p = atan(-Y/d_p) - ψ
    联立消 ψ (Δ = b_w − b_p), 用 tan 减法公式化为 Y 的二次方程:
        tanΔ·Y² + (d_p−d_w)·Y + tanΔ·d_w·d_p = 0
    解析解 (取近根 + 有理化, Δ→0 数值稳定):
        Y = 2·tanΔ·d_w·d_p / ((d_w−d_p) + √D)
        D = (d_w−d_p)² − 4·tan²Δ·d_w·d_p
        ψ = atan(−Y/d_w) − b_w
    D < 0 ⟺ 两码方位差超过基线几何上限 → 无物理解。方位差函数在
    |Y| = √(d_w·d_p) 处有极值, 观测落在极值两侧时方程双解 (两位姿产生
    相同观测) —— 取 |Y| 较小的根 (单调可逆区, 观测-位姿一一对应);
    真实位姿远偏到双解区时解算欠幅但方向正确, 走停每停重测闭环自愈。

    Returns:
        (Y, psi) — 修正量为 -Y (横移) / -ψ (原地转); 解不可信时 None:
        基线退化 [min_baseline, max_baseline] 之外、方位差超基线几何
        上限 (D < 0)、或解幅值超合理性范围 (max_slide / max_psi)。
    """
    baseline = d_w - d_p
    if not (min_baseline <= baseline <= max_baseline):
        return None
    db = b_w - b_p                   # 方位差 = atan(-Y/d_w) - atan(-Y/d_p)
    t = math.tan(db)
    # 二次方程判别式: D < 0 ⟺ 方位差超过基线几何上限 → 无物理解。
    # |Y| 恰在 √(d_w·d_p) 处 D 理论为 0, 浮点舍入可出 -1e-17 → 容差防误拒。
    disc = baseline * baseline - 4.0 * t * t * d_w * d_p
    if disc < -1e-12:
        return None
    # 有理化近根 (Δ→0 时分子分母同阶→0, 无 0/0 灾难)
    y = 2.0 * t * d_w * d_p / (baseline + math.sqrt(max(disc, 0.0)))
    psi = math.atan(-y / d_w) - b_w
    if abs(y) > max_slide or abs(psi) > max_psi:
        return None
    return y, psi


