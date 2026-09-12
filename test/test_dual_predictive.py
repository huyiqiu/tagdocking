"""Independent fixed-world SE(2) sensor, camera boundaries and failure cases."""
import math
from types import SimpleNamespace as NS

import pytest

from tagdocking.dual_docking import predict, geometry, DualTagDocking
from tagdocking.dual_camera import CameraModel
from tagdocking.dual_feedback import ActionWatch
from tagdocking.geometry_planner import ActionPlan
from test_dual_docking import Node, controller, frames, plan


def info(**changes):
    values = dict(header=NS(frame_id='optical'), width=1600, height=1296,
        k=[700.,0,800.,0,700.,648.,0,0,1], p=[700.,0,800.,0,0,700.,648.,0,0,0,1,0],
        r=[1.,0,0,0,1,0,0,0,1], d=[0.01,-0.005,0.001,0.001,0.],
        distortion_model='plumb_bob', binning_x=0, binning_y=0,
        roi=NS(x_offset=0,y_offset=0,width=0,height=0,do_rectify=False))
    values.update(changes)
    return NS(**values)


@pytest.mark.parametrize('mode', ['raw','rectified'])
def test_camera_models_and_size_edges(mode):
    cam = CameraModel.from_info(info(),mode,'optical')
    assert min(cam.bounds((0,0,1),.15)) > 500
    assert min(cam.bounds((1.1,0,1),.15)) < 0
    assert cam.bounds((0,0,1),.15)[0] < cam.bounds((0,0,1),.05)[0]
    with pytest.raises(ValueError):
        cam.bounds((0,0,.02),.05)


@pytest.mark.parametrize('changes,mode', [({'width':0},'raw'),
    ({'binning_x':2},'raw'), ({'distortion_model':'equidistant'},'raw'),
    ({'d':[1.]},'raw'), ({'k':[0.]*9},'raw'),
    ({'r':[0.]*9},'rectified'), ({'p':[0.]*12},'rectified'),
    ({'header':NS(frame_id='other')},'raw'), ({},'automatic')])
def test_bad_camera_models(changes,mode):
    with pytest.raises(ValueError):
        CameraModel.from_info(info(**changes),mode,'optical')


def sensor(world, robot, translation, pitch):
    """Independent world->camera rigid transform, never calls predict/geometry."""
    x,y,heading = robot
    c,s = math.cos(heading), math.sin(heading)
    dx,dy = world[0]-x,world[1]-y
    bx,by,bz = c*dx+s*dy-translation[0], -s*dx+c*dy-translation[1],world[2]-translation[2]
    return (-by, math.sin(pitch)*bx-math.cos(pitch)*bz,
            math.cos(pitch)*bx+math.sin(pitch)*bz)


@pytest.mark.parametrize('pitch', [0.,.2,-.15])
@pytest.mark.parametrize('motion', [(0,.03,0),(0,-.03,0),(0,0,.05),(0,0,-.05),(.05,0,0)])
def test_full_extrinsics_prediction_against_world_sensor(pitch,motion):
    c = controller()
    c.r = ((0,math.sin(pitch),math.cos(pitch)),(-1,0,0),(0,-math.cos(pitch),math.sin(pitch)))
    robot = (.1,-.2,.12)
    f,l,a = motion
    nxt = (robot[0]+math.cos(robot[2])*f-math.sin(robot[2])*l,
           robot[1]+math.sin(robot[2])*f+math.cos(robot[2])*l,robot[2]+a)
    for world in ((2,0,.6),(1.1,0,.1)):
        pt = sensor(world,robot,c.t,pitch)
        assert predict(pt,c.r,c.t,f,l,a) == pytest.approx(sensor(world,nxt,c.t,pitch),abs=1e-12)


@pytest.mark.parametrize('offset,heading', [(y,h) for y in (-.12,0.,.12) for h in (-.12,0.,.12)])
def test_closed_loop_center_and_advance_under_realistic_time_budget(offset,heading):
    c = controller()
    c.camera = CameraModel.from_info(info(d=[]),'rectified','optical')
    robot = [0.,offset,heading]
    # 相机在 base x=0.2: 光学 z = 2.0-0.2 = 1.8 = 观察点 (旧值 1.6 在新站位
    # 守卫下沿之下, 用例会先被守卫拖着后退 —— 那不是本用例的主题)。
    world = ((2.0,.03,.6),(1.1,.03,.1))
    now = 10.
    start = now
    lateral = 0
    for iteration in range(80):
        for _ in range(4):
            now += .2
            wall,pile = [sensor(p,robot,c.t,0.) for p in world]
            c.observe(round(now*1e9),round(now*1e9),wall,pile)
        seq = plan(c,round(now*1e9))
        assert not c.failure, (offset,heading,iteration,c.failure)
        if seq is None:
            continue
        step = seq[0]
        if step.jog_distance > 0 and not step.lateral_distance:
            assert c.aligned(wall,pile)
            if c.stage == 'approach':
                assert now-start < 90.
                assert c.actions < 30
                if offset:
                    assert lateral > 0
                return
        c.action_started(step,round(now*1e9))
        f = 0. if step.lateral_distance else step.jog_distance
        l,a = step.lateral_distance,step.turn_angle
        lateral += bool(l)
        robot[0] += math.cos(robot[2])*f-math.sin(robot[2])*l
        robot[1] += math.sin(robot[2])*f+math.cos(robot[2])*l
        robot[2] += a
        now += max(abs(f)/.08,abs(l)/.12,abs(a)/.15)+.4  # posture unlock
        c.action_completed()
        c.stopped(round(now*1e9))
        now += 1.5+.4  # settle and posture/readout overhead
    pytest.fail('did not reach aligned forward motion')


def test_no_candidate_counts_independent_windows_not_timer_ticks():
    # 用不可能达到的 score_improvement 制造"候选全不改善", 而不是用退化相机
    # 制造"候选全不可见": 后者现在是可恢复局面 (见下一个用例), 走后退脱困而
    # 非计窗口, 已不再命中 _no_candidate。
    c = DualTagDocking(Node(**{'dual.score_improvement': 1e9}))
    c.set_extrinsics((.2,.03,.4),(-.5,.5,-.5,.5))
    c.camera = CameraModel(1600,1296,400.,400.,800.,648.)
    now = frames(c,x=.1)
    assert not c.envelope_violated()      # 当前位姿在包络内 → 不触发后退脱困
    assert plan(c,now) is None and c.no_candidates == 1
    for _ in range(50):
        plan(c,now)
    assert c.no_candidates == 1 and not c.failure
    for start in (11.,12.):
        now = frames(c,x=.1,start=start)
        plan(c,now)
    assert c.failure


def test_envelope_violated_pose_reverses_instead_of_dying():
    """当前位姿已出包络 → 转/移候选恒不可行 (fraction 0 就是当前位姿),
    唯一出路是后退; 后退必须仍受预算约束, 不能无限退。"""
    c = controller()
    c.camera = CameraModel(10,10,700,700,5,5)   # 一切都在画面外
    now = frames(c,x=.1)
    assert c.envelope_violated()
    step = plan(c,now)
    assert step and step[0].jog_distance < 0 and not c.no_candidates
    budget = int(c.p('reverse_count'))
    for i in range(budget+2):
        c.reverse_actions = i               # 预算递进, 跳过里程计回环
        now = frames(c,x=.1,start=20.+i)
        if plan(c,now) is None and c.failure:
            break
    assert c.failure and 'reverse' in c.failure


@pytest.mark.parametrize('kind', ['yaw','lateral','forward'])
@pytest.mark.parametrize('fault', ['reverse','stale','no_response','timeout'])
def test_signed_watchdog(kind,fault):
    c = controller()
    step = (ActionPlan(kind='yaw',turn_angle=.02) if kind=='yaw' else
            ActionPlan(kind='forward',jog_distance=.03,lateral_distance=.03 if kind=='lateral' else 0))
    watch = ActionWatch(step,int(10e9),(0,0,0),.02 if kind=='yaw' else .03,.1,c.p)
    # reverse 判据现在有起步瞬态宽限 (action_startup_sec), 取宽限之后取样 ——
    # 宽限内的反向是四足换步瞬态, 不是故障 (见
    # test_startup_transient_is_not_a_reverse_verdict)。
    reverse_at = 10 + c.p('action_startup_sec') + .05
    now = int((13 if fault=='no_response' else 17 if fault=='timeout'
               else reverse_at if fault=='reverse' else 10.2)*1e9)
    pose = ((0,0,-.03) if kind=='yaw' else (0,-.04,0) if kind=='lateral' else (-.04,0,0)) if fault=='reverse' else (0,0,0)
    assert watch.check(now,pose,int(9e9) if fault=='stale' else now)


def test_false_odom_progress_fails_visual_feedback():
    c = controller()
    for i in range(3):
        now = frames(c,x=.1,start=10+i*3)
        seq = plan(c,now)
        if c.failure:
            break
        c.action_started(seq[0],now)
        c.action_completed()  # Claim odom completion but world does not move.
        c.stopped(now)
    now = frames(c,x=.1,start=19)
    plan(c,now)
    assert 'no progress' in c.failure


@pytest.mark.parametrize('name,value', [('visibility_samples',.5),('visibility_samples',257),
    ('feedback_fail_windows',1.5),
    # 最小横移不得超过横移步长。门槛跟着 lateral_step (0.05) 走, 不再是写死的
    # 3cm —— 那个 min(.03, lateral_step) 的硬上限已拆除, 0.04 现在是合法值。
    ('min_lateral_m',.06),
    ('lateral_step',.002),('lateral_step',.12),
    ('pile_tag_size',-1),
    ('straight_yaw_tol_deg',.2),('straight_yaw_tol_deg',5.),
    ('straight_start_distance',1.75),
    # 纯直行区上界须 dock_distance(.5) < 本值 <= straight_start_distance(1.70):
    # 落在终点之后等于从不生效, 越过站位则把 observe 的双码对准一起禁掉。
    ('steering_stop_distance',.5),('steering_stop_distance',1.75)])
def test_invalid_parameter_bounds(name,value):
    from test_dual_docking import Node
    from tagdocking.dual_docking import DualTagDocking
    with pytest.raises(ValueError):
        DualTagDocking(Node(**{'dual.'+name:value}))


def test_camera_wait_is_finite():
    c = controller()
    c.camera = None
    assert plan(c,int(10e9)) is None
    plan(c,int(16e9))
    assert 'CameraInfo unavailable' in c.failure


def test_vertical_exit_never_relaxes_for_corrections_or_uncommitted_headings():
    """桩码垂直离场的放行边界。允许它的是"航向已有新鲜证据"(两个来源),
    不允许它的是纠偏、未提交的阶段、包络之外与过期的承诺。"""
    c = controller()
    c.stage = 'approach'
    c.wall,c.pile = (0,-.2,1.),(0,.3,.1)
    c.stamp = int(20e9)
    forward = ActionPlan(kind='forward',jog_distance=.02)
    assert not c.visible(forward)[0]          # 无任何航向证据 → 不放行
    c.progress,c.qualified_ns = True,c.stamp  # 证据①: 已完成的合格直行
    assert c.visible(forward)[0]
    c.progress,c.qualified_ns = False,0
    c.committed_ns = c.stamp                  # 证据②: 持住对准的直行承诺
    assert c.visible(forward)[0]
    # 纠偏永不享受放行 —— 垂直离场只对纯直行成立。
    assert not c.visible(ActionPlan(kind='yaw',turn_angle=.01))[0]
    assert not c.visible(ActionPlan(kind='forward',jog_distance=.003,lateral_distance=.003))[0]
    c.wall = (0,-.2,c.straight_envelope+.05)  # 直行包络之外 (远场 stage 陈旧)
    assert not c.visible(forward)[0]
    c.wall = (0,-.2,1.)
    c.stage = 'observe'                       # observe 仍须保住两码 (J 要它)
    assert not c.visible(forward)[0]
    c.stage = 'approach'
    c.stamp = int((20+c.p('qualification_sec')+1)*1e9)   # 承诺过期
    assert not c.visible(forward)[0]
    assert c.stage == 'approach'  # Prediction alone never locks heading.


def test_intermediate_sample_is_checked_not_only_endpoint():
    c = controller()
    frames(c)
    original = c.predicted_pair
    calls = []
    def trajectory(step,fraction=1.):
        calls.append(fraction)
        if .4 < fraction < .6:
            return ((99,0,1),c.pile)
        return original(step,fraction)
    c.predicted_pair = trajectory
    assert not c.visible(ActionPlan(kind='forward',jog_distance=.02))[0]
    assert any(0 < f < 1 for f in calls)


@pytest.mark.parametrize('costs', [(10.001,9.999,10.001), (12.,8.,10.), (11.,12.,13.)])
def test_noisy_oscillating_or_overshooting_visual_feedback_fails(costs):
    c = controller()
    before = 10.
    for after in costs:
        c.feedback_pending = {'before':((),0,0,before),'correction':True}
        c._check_feedback(((),0,0,after))
        before = after
    assert c.failure


def test_independent_world_full_approach_real_pile_loss_and_terminal_depth():
    c = controller()
    robot = [0.,0.,0.]
    # A bounded exit-compatible mounting/layout, not a claim for all 0.9m layouts.
    # 相机在 base x=0.2: 光学 z = 1.8 = 观察点 (旧 1.7→光学 1.5 在站位窗之下)。
    world = ((2.0,.03,.6),(1.1,.03,.2))
    now = 10.
    locked = False
    zone_steps = 0
    for iteration in range(140):
        for _ in range(4):
            now += .2
            wall,pile = [sensor(p,robot,c.t,0.) for p in world]
            missing = pile[2] <= 0 or abs(400*pile[1]/pile[2]) > 648
            c.observe(round(now*1e9),round(now*1e9),wall,None if missing else pile,pile_missing=missing)
        seq = plan(c,round(now*1e9))
        assert not c.failure, c.failure
        if seq is None:
            continue
        step = seq[0]
        locked |= c.stage == 'locked'
        if step.kind == 'done':
            assert locked and c.progress and abs(wall[2]-.5) <= .02
            assert now-10 < 300
            # 纯直行区 (1.0→0.5m) 一个动作走完, 旧行为是 10cm × 5 步 × ~2.9s。
            assert zone_steps == 1, f'区内发了 {zone_steps} 个动作'
            return
        assert not step.turn_angle and not step.lateral_distance
        if c.wall[2] <= c.steering_stop:
            zone_steps += 1
            assert step.continuous, '区内长行程必须带看门狗放宽标记'
            assert step.jog_distance == pytest.approx(c.wall[2]-c.target)
        else:
            assert 0 < step.jog_distance <= c.p('forward_step')+1e-9
            assert not step.continuous, '区外不得放宽看门狗'
        c.action_started(step,round(now*1e9))
        robot[0] += step.jog_distance
        now += step.jog_distance/.08+.4
        c.action_completed()
        c.stopped(round(now*1e9))
        now += 1.9
    pytest.fail('did not complete bounded independent approach')


def test_turn_rate_never_falls_into_chassis_dead_zone():
    """两级减速 (小角半速 × 近目标半速) 叠乘后仍须高于桥的 min_angular_z。

    双码 yaw 候选上限 3° (0.052rad), 全部 < small_turn_rad(0.1) 且
    < turn_slow_rad(0.14), 因此每一个候选都会吃满两级减速。修复前
    0.3×0.5×0.5 = 0.075 < 0.10 死区 → clampAxis 清零 → 狗不动。
    """
    from tagdocking.action_executor import ActionExecutor
    dead = 0.10
    for deg in (3.0, 1.5, 0.75, 0.3):
        ex = ActionExecutor(small_turn_rad=0.1, turn_slow_rad=0.14,
                            min_angular_rate=0.12)
        assert ex.start_turn(-math.radians(deg), 0.3, full=True)
        assert abs(ex.angular_cmd) > dead, f'起步速率掉进死区 ({deg}deg)'
        # 起步即 remaining < turn_slow_rad → 近目标减速立刻生效
        ex._update_turn(0.0, False, None, lambda: 0.0, lambda d: (0., 0.))
        assert abs(ex.angular_cmd) > dead, f'近目标减速后掉进死区 ({deg}deg)'
        assert ex.angular_cmd < 0, '转向符号必须保持 CW'


def test_startup_transient_is_not_a_reverse_verdict():
    """四足起步瞬态 (机体先反向晃) 不得判 "opposite commanded direction",
    但真正整步走反必须照抓。"""
    c = controller()
    step = ActionPlan(kind='yaw', turn_angle=-math.radians(3))
    target = math.radians(3)
    watch = ActionWatch(step, int(10e9), (0,0,0), target, .12, c.p)
    grace = c.p('action_startup_sec')

    # 起步 108ms 内反向 0.4° —— 现场实际中止的那一幕, 现在必须放行。
    early = int((10 + .108)*1e9)
    assert not watch.check(early, (0,0,+math.radians(.4)), early)
    # 宽限期内哪怕反向到整步量, 也只是瞬态, 不判 (超时/无响应仍各司其职)。
    assert not watch.check(early, (0,0,+target), early)

    # 过了宽限期: 半程以内的残余抖动仍放行, 整步反向必须抓。
    late = int((10 + grace + .05)*1e9)
    assert not watch.check(late, (0,0,+target*.4), late)
    assert 'opposite' in watch.check(late, (0,0,+target), late)


def test_reverse_threshold_is_not_pinned_to_static_noise_floor():
    """门槛须随指令量缩放, 不能被 3×odom_noise (0.34°) 的静止噪声地板锁死。"""
    c = controller()
    target = math.radians(3)
    watch = ActionWatch(ActionPlan(kind='yaw', turn_angle=-target),
                        int(10e9), (0,0,0), target, .12, c.p)
    late = int((10 + c.p('action_startup_sec') + .05)*1e9)
    assert not watch.check(late, (0,0,+math.radians(1.0)), late), '1° 抖动不应判反向'
    assert 'opposite' in watch.check(late, (0,0,+math.radians(2.0)), late)


def test_relaxed_escape_plan_survives_pending_revalidation():
    """脱困后退必须能真的发出去 —— 这是一个活锁, 不是报错。

    后退是在"当前位姿已出包络"时经 improving() (只要求不恶化) 放行的。
    若 pending_valid 改用严格 visible() 复检, 它必然失败 (fraction 0 就是
    那个已出界的当前位姿), 于是: 规划发后退 → 复检否掉 → stopped() 重置
    滤波 + settle 再推 1.5s → 重新采帧 → 规划同一条后退 …… actions 不增,
    现场只看到告警每 2s 刷一屏而底盘一步不动, 直到外层超时兜底。
    """
    c = controller()
    c.camera = CameraModel(10,10,700,700,5,5)   # 一切都在画面外
    now = frames(c,x=.1)
    assert c.envelope_violated()
    step = plan(c,now)
    assert step and step[0].jog_distance < 0
    assert c.pending_relaxed, '宽松放行必须记在 pending 上'
    assert not c.visible(c.pending_plan)[0], '严格门必然否掉它 (前提条件)'
    assert c.pending_valid(now), '宽松放行的计划被严格复检否掉 → 活锁'


def test_strict_plans_are_not_silently_relaxed_on_revalidation():
    """反向保险: 正常候选仍走严格门, 宽松门不得渗漏成默认。"""
    c = controller()
    now = frames(c,x=.07)
    assert plan(c,now)
    assert not c.pending_relaxed
    c.camera = CameraModel(10,10,700,700,5,5)   # 复检时位姿已不可见
    assert not c.pending_valid(now)


def test_standoff_guard_pulls_back_while_still_correcting():
    """站位守卫必须在纠偏期间生效 —— 那正是它被漏掉的地方。

    原来 observe 的距离检查排在 aligned() 之后、held 之后, 于是"还在纠偏"
    等于"距离无人看管"。现场墙码 z 从 1.68 爬到 1.25 穿过整个观察窗, 规划
    器一次都没察觉, 直到桩码被压出画面下沿、包络破了才暴露。
    """
    c = controller()
    now = frames(c, depth=1.5, x=.1)          # 站位过近 (窗下沿 1.7) + 明显未对准
    assert not c.aligned(c.wall, c.pile), '前提: 此位姿确实未对准'
    step = plan(c, now)
    assert step and step[0].jog_distance < 0, '纠偏期间站位过近必须先后退'
    assert c.stage == 'observe'


def test_standoff_guard_is_near_side_only_and_never_pushes_forward_misaligned():
    """只守近端。太远无害 (两码都在视野里); 航向没对就前进 = 沿错误方向走远。"""
    c = controller()
    now = frames(c, depth=1.95, x=.1)         # 站位过远 (窗上沿 1.9) + 未对准
    step = plan(c, now)
    assert step, '过远且未对准时应继续纠偏, 而不是无动作'
    assert step[0].turn_angle or step[0].lateral_distance, '应纠偏, 不应前进'
    assert step[0].jog_distance >= 0 or step[0].lateral_distance


@pytest.mark.parametrize('stage', ['approach', 'locked'])
def test_standoff_guard_never_blocks_approach_or_locked_travel(stage):
    """approach 是有意逼近 0.5m, locked 是锁定直行 —— 在那里挂 1.7m 门槛
    会把接近永久堵死, 狗退到观察窗就再也进不去。"""
    c = controller()
    c.stage = stage
    c.progress, c.qualified_ns = True, int(10e9)
    # locked 腿显式 x=0.: 否则产出这一步的是 bearing 微调而非 _advance,
    # 用例会静默地不再测它声称测的东西。
    now = frames(c, depth=1.5, x=0.)          # 远低于观察窗下沿
    step = plan(c, now)
    assert step and step[0].jog_distance > 0, f'{stage} 阶段被站位守卫堵死'


def test_standoff_guard_pulls_back_even_when_already_aligned():
    """守卫排在 aligned() 之前, 因此也接管了"已对准但站位过近"。

    这条路径原本由 held 分支里的 `d < obs-tol: return self._reverse()` 负责;
    守卫前置后那行永不可达, 已删。删掉的行为必须在这里被接住, 否则对准的
    狗会从 1.3m 直接 _advance 前进, 把矮桩码顶出画面下沿。
    """
    c = controller()
    now = frames(c, depth=1.3)                # 对准 + 站位过近
    assert c.aligned(c.wall, c.pile), '前提: 此位姿已对准'
    step = plan(c, now)
    assert step and step[0].jog_distance < 0, '对准时站位过近仍须后退, 不得前进'
    assert c.stage == 'observe', '不得借机升到 approach'


def test_standoff_guard_uses_the_relaxed_gate_once_the_envelope_is_broken():
    """包络已破时严格 visible() 会否掉一切候选 (fraction 0 就是当前位姿)。
    守卫若不换宽松门, 就会亲手堵死唯一的出路, 退化成 _no_candidate 计窗口。

    用"已对准"位姿: 未对准会走 _correction, 那里另有一套脱困后退, 测出来
    的就不是守卫自己的门 (最初写成未对准, 拆掉守卫照样通过 —— 无效断言)。
    """
    c = controller()
    c.camera = CameraModel(10,10,700,700,5,5)   # 一切都在画面外
    now = frames(c, depth=1.3)
    assert c.aligned(c.wall, c.pile) and c.envelope_violated()
    step = plan(c, now)
    assert step and step[0].jog_distance < 0
    assert c.pending_relaxed and not c.no_candidates


def test_standoff_budget_exhaustion_names_standoff_not_acquisition():
    """现场下一个问题必然是"为什么退完了" —— 失败串必须自己回答:
    找桩码退不出来, 还是站位守卫在跟匍匐爬行赛跑且没跑赢。"""
    c = controller()
    for i in range(int(c.p('standoff_reverse_count'))+2):
        c.standoff_actions = i                 # 站位预算递进, 跳过里程计回环
        now = frames(c, depth=1.3, x=.1, start=20.+i)
        if plan(c, now) is None and c.failure:
            break
    assert 'reverse' in c.failure and 'standoff' in c.failure


def test_locked_bearing_correction_fires_and_stops_inside_tolerance():
    """locked 不再是纯前进 (设计决定 1): bearing 超容差出转向, 容差内出纯前进。"""
    c = controller()
    c.stage = 'locked'
    n = frames(c, depth=1.8, x=.06)           # bearing ≈ 1.9° > tol 1.5°
    step = plan(c, n)[0]
    assert step.kind == 'yaw' and step.turn_angle < 0, 'b>0 必须配负转向 (光学系)'
    assert not c.pending_aligned, '转向不得产生行进资格'
    n = frames(c, depth=1.8, x=.02, start=14.)  # bearing ≈ 0.6° ≤ tol
    step = plan(c, n)[0]
    assert step.kind == 'forward' and step.turn_angle == 0 and step.jog_distance > 0


def test_locked_turn_sign_is_verified_against_predict():
    """符号不由手写决定而由 predict 裁决: 选出的转向必须严格缩小 |bearing|。
    力臂增益 1+t_x/z (命令 2.86° 实变 bearing 3.30°) 被预测精确吸收 ——
    符号写反的实现无法满足这一条。"""
    c = controller()
    c.stage = 'locked'
    n = frames(c, depth=1.8, x=.06)
    step = plan(c, n)[0]
    assert step.kind == 'yaw'
    b = math.atan2(c.wall[0], c.wall[2])
    wall_after = c.predicted_pair(step)[0]
    assert abs(math.atan2(wall_after[0], wall_after[2])) < abs(b)


def test_late_locked_correction_still_reaches_done():
    """进 locked 后才暴露的 bearing 误差: 转向清掉 qualified_ns, 但 progress
    是粘性的 —— 终点判定不得误报 'terminal depth without qualified approach'。"""
    # 0.62m 在生产的纯直行区 (1.0m) 之内, 那里微调已按设计停摆; 本用例测的是
    # "晚到的转向撤资格后终点仍判 done"这条路径, 把禁令下压到 0.55m 才暴露它。
    c = DualTagDocking(Node(**{'dual.steering_stop_distance': .55}))
    c.set_extrinsics((.2, .03, .4), (-.5, .5, -.5, .5))
    c.camera = CameraModel(1600, 1296, 400., 400., 800., 648.)
    c.stage = 'locked'
    c.progress = True
    n = frames(c, depth=.62, x=.027, count=10)   # 0.62m 处 2.5° bearing
    step = plan(c, n)[0]
    assert step.kind == 'yaw', '前提: 晚到的 locked 转向确实触发'
    c.action_started(step, n)
    c.action_completed()
    assert c.qualified_ns == 0 and c.progress    # 转向撤资格, progress 幸存
    c.stopped(n)
    depth = .57
    for start in (14., 17., 20.):
        n = frames(c, depth=depth, start=start, count=10)
        step = plan(c, n)[0]
        assert not c.failure, c.failure
        if step.kind == 'done':
            break
        assert step.jog_distance > 0
        c.action_started(step, n)
        c.action_completed()
        c.stopped(n)
        depth = round(depth-step.jog_distance, 3)
    assert step.kind == 'done' and c.complete
    assert 'terminal depth' not in c.failure


def test_locked_turn_rejected_by_visibility_drives_straight_instead_of_failing():
    """转向候选全不可见 → 直行, 而不是 _no_candidate 计窗口或判失败:
    把锦上添花的微调变成整场健康直行的中止是错的交易。墙码仍受保护 ——
    不可见的转向根本不会被发出。"""
    c = controller()
    c.stage = 'locked'
    n = frames(c, depth=1.8, x=.06)
    original_visible = c.visible
    def no_turn_visibility(plan, full=False):
        ok, margins = original_visible(plan, full)
        return (False, margins) if plan.turn_angle else (ok, margins)
    c.visible = no_turn_visibility              # 一切转向候选不可见, 前进可见
    step = plan(c, n)[0]
    assert step.jog_distance > 0 and step.turn_angle == 0
    assert c.no_candidates == 0 and not c.failure


def test_locked_consecutive_turn_cap_forces_progress():
    """底盘 ~2° 停止滞后让预测与现实有同量级偏差; 喂永不改善的 bearing,
    连续转向达到 straight_yaw_max_turns 后必须强制前进 —— 两条路径都不判失败。"""
    c = controller()
    c.stage = 'locked'
    cap = int(c.p('straight_yaw_max_turns'))
    turns = 0
    for i in range(cap+2):
        n = frames(c, depth=1.8, x=.06, start=10.+i*4)
        seq = plan(c, n)
        assert not c.failure, c.failure
        step = seq[0]
        if step.kind != 'yaw':
            assert step.jog_distance > 0, '达上限后必须出前进而非失败'
            break
        turns += 1
        c.action_started(step, n)
        c.action_completed()
        c.stopped(n)
    assert turns == cap, f'期望恰 {cap} 次连续转向后强制前进'


def test_standoff_and_escape_budgets_are_independent():
    """站位后退是常态操作, 必须有独立预算: 耗尽站位账时主账 (脱困/找桩码)
    原封不动, 反之亦然 —— 共账会在真脱困时误报耗尽。"""
    c = controller()
    for i in range(int(c.p('standoff_reverse_count'))+2):
        c.standoff_actions = i                  # 站位预算递进
        now = frames(c, depth=1.3, x=.1, start=20.+i, count=10)
        if plan(c, now) is None and c.failure:
            break
    assert 'standoff' in c.failure and 'acquisition' not in c.failure
    assert c.reverse_actions == 0 and c.reverse_total == 0.
    c2 = controller()
    for i in range(int(c.p('reverse_count'))+2):
        c2.reverse_actions = i                  # 主账递进 (acquire 找桩码路径)
        now = frames(c2, missing=True, start=20.+i, count=10)
        if plan(c2, now) is None and c2.failure:
            break
    assert 'acquisition' in c2.failure and 'standoff' not in c2.failure
    assert c2.standoff_actions == 0 and c2.standoff_total == 0.


def test_first_forward_after_locked_entry_is_not_rejected_by_the_pile_envelope():
    """站立进 locked 的正规入口是桩码丢失闭锁; 桩码末次位姿贴近可见下沿时,
    首步 locked 前进的 pending_valid 复检不得被桩码包络绑架 (Step 0 豁免)。"""
    c = controller()
    c.stage = 'approach'
    c.progress = True
    c.qualified_ns = int(10e9)
    n = frames(c, depth=1.12, pile_x=0., count=10)   # 桩码 z=0.22 贴近下沿仍可见
    n = frames(c, depth=1.12, missing=True, start=13., count=10)
    step = plan(c, n)[0]
    assert c.stage == 'locked' and step.jog_distance > 0
    assert c.pending_pile is None and c.pending_valid(n)


def test_pile_loss_anywhere_below_the_standoff_latches():
    """near=1.70 买到的东西 (事实 C): 站立下桩码在墙码 z≈1.5m 处丢失,
    1.55m ≤ near → 干净闭锁进 locked; near=1.0 时这里是硬失败。"""
    c = controller()
    c.stage = 'approach'
    c.progress = True
    c.qualified_ns = int(10e9)
    n = frames(c, depth=1.55, missing=True, count=10)
    step = plan(c, n)
    assert step and step[0].jog_distance > 0 and c.stage == 'locked'
    assert not c.failure


def test_yaw_cap_is_distance_scheduled_and_actually_configurable():
    """曾经写死 min(3°, yaw_step_deg) —— 配置项是装饰。现在远近分档且可调。"""
    c = DualTagDocking(Node(**{'dual.yaw_step_deg': 8., 'dual.yaw_fine_step_deg': 3.}))
    c.set_extrinsics((.2,.03,.4),(-.5,.5,-.5,.5))
    assert c.wall is None
    assert c.yaw_cap == pytest.approx(math.radians(3)), '无量测时保守取近场细步'
    c.wall = (0., -.2, c.near + .4)           # 远场
    assert c.yaw_cap == pytest.approx(math.radians(8))
    c.wall = (0., -.2, c.near - .1)           # 近场
    assert c.yaw_cap == pytest.approx(math.radians(3))
    # 真正可调: 把远场粗步调到 6° 必须体现出来 (旧代码恒为 3°)。
    coarse = DualTagDocking(Node(**{'dual.yaw_step_deg': 6.}))
    coarse.wall = (0., -.2, coarse.near + .4)
    assert coarse.yaw_cap == pytest.approx(math.radians(6))


def test_far_field_coarse_yaw_candidates_reach_the_configured_cap():
    """远场枚举必须真的出现接近上限的大步候选 —— 否则"放开上限"只是改了个数。"""
    c = controller()
    c.camera = CameraModel.from_info(info(d=[]),'rectified','optical')
    now = frames(c, depth=1.85, x=.25)        # 远场 (near=1.7 之上) + 明显方位偏差
    seq = plan(c, now)
    assert seq and not c.failure
    turns = [abs(s.turn_angle) for s in seq if s.turn_angle]
    assert turns and max(turns) > math.radians(3), '远场仍被 3° 卡住'
    assert max(turns) <= c.yaw_cap + 1e-12


def test_yaw_cap_bounds_are_validated():
    for overrides in ({'dual.yaw_step_deg': 20.}, {'dual.yaw_step_deg': .2},
                      {'dual.yaw_fine_step_deg': 20.},
                      {'dual.yaw_step_deg': 3., 'dual.yaw_fine_step_deg': 8.},
                      {'dual.prealign_tolerance_deg': 1.},
                      {'dual.prealign_step_deg': 40.},
                      {'dual.prealign_max_steps': 2.5}):
        with pytest.raises(ValueError):
            DualTagDocking(Node(**overrides))


def shortcam(height):
    """真实相机, 只是画幅下沿更近 —— 等价于现场那台相对桩码装得偏高的相机:
    桩码 (0.9m 处) 中心仍在画面内 (检测器看得见它, 量测照常进来), 但保守的
    外接立方体包络已经戳出下沿, 于是 visible() 的严格门否掉一切前进候选。
    现场的余量序列 61.6→49.3→39.0→25.8→+8px 就是这条路。"""
    return CameraModel(1600, height, 400., 400., 800., 648.)


# 站位处 (墙码光学 z=1.83) 桩码中心在 740 画幅内, 首步之后出画幅 —— 即
# "桩码恰在提交直行处离场", 现场那一幕。
# 墙码故意不放在 2.00 (光学 1.80): 10cm 步长下 1.80 会让离场恰好落在 near
# (1.70) 这个刀口上, 本用例"闭锁发生在 near 之上"的前提就变成了边界巧合而
# 不再是断言。1.83 起步后每一步都离两个门槛都有余量。
COMMIT_WORLD = ((2.03, .03, .6), (1.1, .03, .2))


def commit_frames(c, robot, now, detect=True):
    """四帧稳定窗口; detect=True 时桩码按这台相机的画幅判在不在。"""
    for _ in range(4):
        now += .2
        wall, pile = [sensor(p, robot, c.t, 0.) for p in COMMIT_WORLD]
        missing = detect and (pile[2] <= 0 or
                              400*pile[1]/pile[2]+648 > c.camera.height)
        c.observe(round(now*1e9), round(now*1e9), wall,
                  None if missing else pile, pile_missing=missing)
    return now, wall, pile


def test_first_straight_step_at_the_commit_distance_is_not_deadlocked():
    """2026-09-11 现场死锁: 1.78m 提交直行, bearing≈1°、J≈0.03 —— 直行本可
    成功, 但每条前进候选都被桩码底边否掉, 三窗耗尽判 MOTION_FAILED
    ('no visible translation candidate; independent stable-window budget
    exhausted')。

    两把锁互为前提: 放行要求 progress (已完成一步合格直行) 且 z ≤ near
    (1.70) —— 而这两条都只能靠先前进一步拿到, 前进正是被否掉的那件事。
    """
    c = controller()
    c.camera = shortcam(740)
    now, wall, pile = commit_frames(c, [0., 0., 0.], 10., detect=False)
    assert c.aligned(c.wall, c.pile), '前提: 此位姿确实已对准'
    assert c.near < c.wall[2] <= c.straight_envelope, '前提: 提交距离在观察窗内、near 之上'
    assert min(c.current_margins()) < c.required_margin, '前提: 桩码包络已戳出下沿'
    assert not c.progress, '前提: 首步之前没有任何已完成的合格直行'
    step = plan(c, round(now*1e9))[0]
    assert c.stage == 'approach'
    assert step.jog_distance > 0 and not step.turn_angle and not step.lateral_distance
    assert c.no_candidates == 0 and not c.failure


def test_pile_exit_at_the_commit_distance_latches_and_reaches_done():
    """放行与闭锁是同一个判断的两半: 若桩码可在 1.70~1.90 合法离场而闭锁仍
    只认 ≤ 1.70, 那段离场就会掉进 'pile missing outside qualified final
    entry' 硬失败 —— 修复自己制造的新失效模式。整条链路必须走到 done。"""
    c = controller()
    c.camera = shortcam(740)
    robot = [0., 0., 0.]
    now = 10.
    locked = False
    for _ in range(140):
        now, wall, pile = commit_frames(c, robot, now)
        seq = plan(c, round(now*1e9))
        assert not c.failure, c.failure
        if seq is None:
            continue
        step = seq[0]
        if c.stage == 'locked' and not locked:
            locked = True
            assert wall[2] > c.near, '前提: 闭锁发生在 near 之上 (旧门槛会硬失败)'
        if step.kind == 'done':
            assert locked and c.progress and abs(wall[2]-.5) <= c.p('dock_tolerance')
            return
        assert not step.lateral_distance
        c.action_started(step, round(now*1e9))
        robot[0] += math.cos(robot[2])*step.jog_distance
        robot[1] += math.sin(robot[2])*step.jog_distance
        robot[2] += step.turn_angle
        now += max(abs(step.jog_distance)/.08, abs(step.turn_angle)/.15)+.4
        c.action_completed()
        c.stopped(round(now*1e9))
        now += 1.9
    pytest.fail('提交距离桩码离场后未能走到 done')


def tuned(**overrides):
    """带参数覆写的控制器 (controller() 的同款标定/相机)。"""
    c = DualTagDocking(Node(**{'dual.'+k: v for k, v in overrides.items()}))
    c.set_extrinsics((.2, .03, .4), (-.5, .5, -.5, .5))
    c.camera = CameraModel(1600, 1296, 400., 400., 800., 648.)
    return c


def test_jog_steps_are_actually_configurable_no_hidden_cap():
    """五处发出点原来都写死 min(.05, ...), 配置项是装饰品 —— docking.yaml 里
    reverse_step 早就是 0.10 却从未生效。用非 0.05/0.10 的值断言, 防止哪天又
    冒出一个"顺手"的硬上限 (yaw_cap 那次的同一个病)。"""
    c = tuned(forward_step=.17)
    n = frames(c)                                  # 站位 1.8m, 已对准
    assert plan(c, n)[0].jog_distance == pytest.approx(.17)
    c = tuned(reverse_step=.13)
    n = frames(c, missing=True, count=10)          # acquire 看不见桩码 → 后退找回
    assert plan(c, n)[0].jog_distance == pytest.approx(-.13)
    # 横移是同一个病的第三处: 候选枚举写死 min(.03, lateral_step), 现场把
    # docking.yaml 调到 0.05 跑出来仍是 0.03。用 .07 断言枚举上限真的跟着配置。
    c = tuned(lateral_step=.07, min_lateral_m=.003)
    n = frames(c, x=.0, pile_x=-.25)               # 大横偏 → 最大横移候选被选中
    step = plan(c, n)[0]
    assert step.lateral_distance and abs(step.lateral_distance) > .03
    assert abs(step.lateral_distance) <= .07 + 1e-9


def test_terminal_step_still_shrinks_under_the_larger_jog():
    """步长放大不牺牲停泊精度: 终点步按剩余距离收缩, 不许过冲。"""
    c = tuned(forward_step=.10)
    c.stage, c.progress = 'locked', True
    n = frames(c, depth=.57, missing=True, count=10)
    assert plan(c, n)[0].jog_distance == pytest.approx(.07)   # 而不是 .10
    c.stopped(n)
    n = frames(c, depth=.5, missing=True, start=14., count=10)
    assert plan(c, n)[0].kind == 'done' and c.complete


def straddle_frames(c, depth, start=10., count=4):
    """跨禁令上界 (1.0m) 的 A/B 位姿: 两码都明显未对准, 且两个深度下桩码都
    舒舒服服在画内 —— frames() 的桩码 (y=.3, z=depth-.9) 在 1m 附近已经贴着
    画面下沿, 用它做 A/B 会把"禁令"和"包络破了"两件事混在一起。"""
    for i in range(count):
        stamp = int((start+i*.2)*1e9)
        c.observe(stamp, stamp, (.06, -.2, depth), (-.06, .03, depth-.5))
    return stamp


def test_pure_straight_zone_bans_yaw_and_lateral():
    """纯直行区内转向与横移都禁, 前进是唯一动作 —— 且不判失败: 近场不调
    方向是取舍而非异常。A/B 只差深度跨过禁令上界, 位姿形状完全相同。"""
    inside = tuned()
    inside.stage = 'approach'
    n = straddle_frames(inside, .95)
    assert inside.wall[2] <= inside.steering_stop, '前提: 在禁令区内'
    assert not inside.aligned(inside.wall, inside.pile), '前提: 此位姿确实未对准'
    step = plan(inside, n)[0]
    assert step.kind == 'forward' and step.jog_distance > 0
    assert not step.turn_angle and not step.lateral_distance
    assert not inside.failure and inside.no_candidates == 0
    outside = tuned()                              # 同一位姿, 深度跨到禁令区外
    outside.stage = 'approach'
    n = straddle_frames(outside, 1.05)
    assert outside.wall[2] > outside.steering_stop
    step = plan(outside, n)[0]
    assert step.turn_angle or step.lateral_distance, '区外必须照旧纠偏'


def test_pure_straight_zone_forgives_the_pile_bottom_without_alignment():
    """禁令区内 allow_exit 不再要求"两码对准": 航向已被策略冻结, 未对准既不
    可被采纳为纠偏, 拦下这一步前进也换不回任何东西 —— 只会把一场合法的纯
    直行变成三窗耗尽 (2026-09-11 那个失效模式的近场版本)。
    但航向证据仍然是硬前提: 绝不按一个从未验证过的航向盲走。"""
    c = tuned()
    c.camera = shortcam(700)                       # 桩码中心在画内, 包络戳出下沿
    c.stage = 'approach'
    n = straddle_frames(c, .95)
    assert c.wall[2] <= c.steering_stop, '前提: 在禁令区内'
    assert not c.aligned(c.wall, c.pile), '前提: 未对准'
    assert min(c.current_margins()) < c.required_margin, '前提: 桩码包络已戳出下沿'
    assert 400*c.pile[1]/c.pile[2]+648 < c.camera.height, '前提: 桩码仍被检出 (中心在画内)'
    assert plan(c, n) is None and c.no_candidates == 1, '无航向证据 → 仍不放行'
    # _no_candidate 会 reset_filter(), 必须重新喂满稳定窗口; 证据要在 observe
    # 之后才立 —— 未对准的新两码会撤销它 (observe 里那一笔)。
    n = straddle_frames(c, .95, start=13.)
    c.progress, c.qualified_ns = True, c.stamp     # 一步已完成的合格直行
    step = plan(c, n)[0]
    assert step.kind == 'forward' and step.jog_distance > 0
    assert not step.turn_angle and not step.lateral_distance and not c.failure


def test_locked_bearing_micro_correction_is_gated_not_stopped_inside_the_zone():
    """区内微调**不停摆**, 改为按 _bearing_tol 的盈亏平衡门槛裁决。

    钉住 2026-09-12 那趟现场: 区内 bearing +1.88→+3.75→+6.02→+11.22° 全程
    无人纠, 终点横偏 ~91mm 对 dock_tolerance 20mm。原来的"整体停摆"拿一个
    ~2° 量级的转向不确定度去否决一个 11° 量级的误差 —— 门槛化之后小的仍
    不纠 (纠不过噪声), 大的必须纠。两半都要断言: 只断言"大的会纠"会让
    "门槛恒为 0"通过, 只断言"小的不纠"就是在测旧行为。"""
    small = controller()                           # bearing 1.8° < 区内门槛 2.2°
    small.stage, small.progress = 'locked', True
    n = frames(small, depth=.95, x=.03, missing=True, count=10)
    b = abs(math.degrees(math.atan2(small.wall[0], small.wall[2])))
    assert b > small.p('straight_yaw_tol_deg'), '前提: 已超区外门槛, 只可能被区内门槛拦下'
    assert b < math.degrees(small._bearing_tol(small.wall[2])), '前提: 未超区内收紧门槛'
    step = plan(small, n)[0]
    assert step.kind == 'forward' and step.turn_angle == 0 and step.jog_distance > 0
    assert small._locked_turns == 0 and not small.failure
    big = controller()                             # bearing 6.4° > 区内门槛 3.3°
    big.stage, big.progress = 'locked', True
    n = frames(big, depth=.80, x=.09, missing=True, count=10)
    assert big.wall[2] <= big.steering_stop, '前提: 确实在区内'
    assert abs(math.degrees(math.atan2(big.wall[0], big.wall[2]))) > math.degrees(
        big._bearing_tol(big.wall[2])), '前提: 已超区内收紧门槛'
    step = plan(big, n)[0]
    assert step.kind == 'yaw' and step.turn_angle, '区内大 bearing 必须纠 —— 这是本次改动'
    assert step.turn_angle < 0, '墙码偏右 (x>0) → 右转, 符号不能反'
    far = controller()                             # 禁令区之外同样的 bearing 仍要微调
    far.stage, far.progress = 'locked', True
    n = frames(far, depth=1.8, x=.057, missing=True, count=10)
    assert plan(far, n)[0].kind == 'yaw'


def test_in_zone_bearing_threshold_tightens_toward_the_dock():
    """门槛随接近单调收紧, 贴到停泊点发散, 且永远不比区外更松。

    这三条是"用门槛代替硬距离悬崖"的全部依据: 单调 = 越近越保守;
    发散 = 最后一截自动禁转 (旧结论被保留, 只是落在物理正确的位置);
    不更松 = 区内绝不会纠一个区外都懒得纠的误差。"""
    c = tuned()
    far = c._bearing_tol(c.steering_stop+.5)
    assert far == pytest.approx(math.radians(c.p('straight_yaw_tol_deg')))
    zone = [c._bearing_tol(z) for z in (1.00, .85, .75, .65, .55)]
    assert all(a < b for a, b in zip(zone, zone[1:])), '越近门槛越高'
    assert all(t >= far for t in zone), '区内门槛不得比区外松'
    assert c._bearing_tol(c.target) == math.pi, '到达停泊点: 任何 bearing 都不再转'
    assert c._bearing_tol(c.target-.05) == math.pi, '冲过停泊点同理 (gap ≤ 0)'
    loose, tight = tuned(straight_yaw_lag_deg=.5), tuned(straight_yaw_lag_deg=8.)
    assert tight._bearing_tol(.75) > loose._bearing_tol(.75), 'lag 调大 = 更保守'
    assert math.degrees(tight._bearing_tol(.75)) > 15, 'lag=8° 近似恢复旧的整体停摆'


def test_in_zone_turn_is_followed_by_the_one_shot_continuous_run():
    """微调不把"一次停到位"换回走停: 纠完一次, 下一个窗口仍是整段连续直行。

    转向是停稳时的原地动作, 前进仍然只发一次 —— 用户要的"不走停走停"说的是
    前进被切碎, 不是不许在起点把方向摆正。"""
    c = tuned()
    c.stage, c.progress = 'locked', True
    n = frames(c, depth=.90, x=.09, missing=True, count=10)
    assert plan(c, n)[0].kind == 'yaw'
    c2, n2 = straight_zone_locked(.90)             # 纠完 → bearing 回到门槛内
    step = plan(c2, n2)[0]
    assert step.kind == 'forward' and step.continuous
    assert step.jog_distance == pytest.approx((.90-c2.target)/c2.r[0][2], abs=1e-6)


def straight_zone_locked(depth, **overrides):
    """禁令区内、航向已锁的对准位姿 —— 连续直行的唯一入口形态。

    桩码故意缺失: 区内桩码 (0.9m 基线) 早已出画, locked 就是为这一幕设的,
    此时只按墙码直行。progress=True 是 locked 的既有前提 (走到这里必然已有
    合格前进), 终点 done 判据也依赖它。"""
    c = tuned(**overrides)
    c.stage, c.progress = 'locked', True
    return c, frames(c, depth=depth, missing=True, count=10)


def test_pure_straight_zone_emits_one_continuous_run_to_the_dock():
    """区内一次直行到停泊处, 不再 10cm×5 次 jog。

    区内禁转向/禁横移, jog 切分换不回任何修正机会, 只剩 5 次起停+停稳+重测
    (现场 ~2.9s/停 ≈ 15s)。整段一次走完, 终点精度交给停稳重测。"""
    c, n = straight_zone_locked(.95)
    assert c.wall[2] <= c.steering_stop, '前提: 在禁令区内'
    step = plan(c, n)[0]
    assert step.kind == 'forward' and not step.turn_angle and not step.lateral_distance
    assert step.jog_distance == pytest.approx(.45), '一步走完 0.95→0.50, 不是 0.10'
    assert step.continuous, '看门狗要据此放宽 deadline, 否则 6s 掐死长行程'
    # 走完停稳重测: 到位即 done, 一个动作解决整段。
    c.action_started(step, n)
    c.action_completed()
    c.stopped(n)
    n = frames(c, depth=.50, missing=True, count=10, start=20.)
    assert plan(c, n)[0].kind == 'done' and c.complete and not c.failure


def test_continuous_flag_only_inside_the_zone_and_last_step_still_shrinks():
    """continuous 是区内专属标记: 区外照旧 forward_step 封顶 + 逐步重测
    (那里还要纠偏, 长盲走会把没对准的航向走远)。区内不足一步则自然收缩。"""
    far, n = straight_zone_locked(1.8)
    assert far.wall[2] > far.steering_stop
    step = plan(far, n)[0]
    assert step.jog_distance == pytest.approx(far.p('forward_step'))
    assert not step.continuous, '区外不得放宽看门狗'
    near, n = straight_zone_locked(.57)              # 剩余 0.07 < forward_step
    step = plan(near, n)[0]
    assert step.jog_distance == pytest.approx(.07) and step.continuous


def test_overshoot_inside_the_zone_trims_back_then_docks():
    """冲过停泊点 → 回退修剪, 不是判失败: 连续直行让"冲过"成为预期结局之一
    (里程计尺度误差无处吸收), 超/欠都由停稳重测兜住。"""
    c, n = straight_zone_locked(.47)                 # 冲过 3cm > dock_tolerance
    step = plan(c, n)[0]
    assert step.kind == 'forward'
    assert step.jog_distance == pytest.approx(-.03), '显式负 jog = 回退修剪'
    assert not step.continuous, 'cm 级修剪不该放宽看门狗'
    assert not c.failure
    c.action_started(step, n)
    c.action_completed()
    c.stopped(n)
    n = frames(c, depth=.50, missing=True, count=10, start=20.)
    assert plan(c, n)[0].kind == 'done' and c.complete


def test_overshoot_trim_is_capped_per_step_for_fresh_remeasure():
    """真冲过一大截 (尺度标错/量测异常) 时分步退: 每步带新鲜重测, 比一次
    长盲退稳。修剪走主账, max_actions 兜住病态振荡。"""
    c, n = straight_zone_locked(.35)                 # 冲过 15cm > reverse_step
    step = plan(c, n)[0]
    assert step.jog_distance == pytest.approx(-c.p('reverse_step'))


def test_qualified_endgame_overshoot_latches_instead_of_failing():
    """冲过硬失败跑在 pile 丢失闭锁之前 —— 合格终局的冲过必须豁免, 否则
    本应"闭锁 → 回退修剪"的局面被直接判死。无资格的异常贴近照旧判失败,
    那才是这条检查要拦的东西。"""
    ok = tuned()
    ok.stage = 'approach'
    n = frames(ok, depth=.47, missing=True, count=10)
    ok.progress, ok.qualified_ns = True, ok.stamp
    assert ok.wall[2] < ok.target-ok.p('dock_tolerance'), '前提: 已冲过'
    step = plan(ok, n)[0]
    assert not ok.failure and ok.stage == 'locked', '闭锁接手, 不判死'
    assert step.kind == 'forward' and step.jog_distance == pytest.approx(-.03)
    bad = tuned()                                    # 同样贴近, 但从无合格前进
    bad.stage = 'approach'
    n = frames(bad, depth=.47, missing=True, count=10)
    assert plan(bad, n) is None or bad.failure
    assert bad.failure and 'overshoot' in bad.failure


def test_terminal_run_exempts_the_pile_entirely_it_ends_under_the_body():
    """狗骑跨在桩上充电 (机体下方电极片对准桩上电极片): 到停泊处桩码必然在
    机体下方、必然出画。要求它在终局整段的终点仍可见, 等于要求一个"成功时
    必然不成立"的条件 —— 而桩码贴近时保守包围立方体的投影还会横向炸开,
    连 allow_exit 保留的左右边也会把整段否掉。"""
    c = tuned()
    c.stage = 'approach'
    n = straddle_frames(c, .95)
    c.committed_ns = c.stamp                         # 航向证据来源② (持住对准)
    c.wall, c.pile = (.06, -.2, .95), (-.06, .03, .45)
    run = ActionPlan(kind='forward', jog_distance=.45, continuous=True)
    assert c.wall[2] <= c.steering_stop, '前提: 区内 → allow_exit 不要求对准'
    # 前提: 桩码这条路径确实"横向炸开", 不是只戳下沿。
    worst = min(c.camera.bounds(c.predicted_pair(run, 11/12.)[1], .05)[:2])
    assert worst < -100, f'前提: 左右边已炸开 ({worst:.0f}px)'
    assert c.visible(run)[0], '终局整段对桩码整体免检'
    assert not c.visible(ActionPlan(kind='forward', jog_distance=.45))[0], \
        '非终局的同等长度前进不豁免 (桩码还要用于下一窗口)'
    c.committed_ns = 0                               # 无航向证据 → allow_exit 死
    assert not c.visible(run)[0], '无 allow_exit 时不得盲走整段'


def test_action_watch_deadline_follows_the_continuous_run_length():
    """外层 min(action_timeout_sec=6.0, …) 会把 6.3s 的合法长行程掐死在 6s。
    continuous 时按实际行程放宽, 其余判据 (反向/无响应/直线一致性) 不动。"""
    p = tuned().p
    pose, spd = (0., 0., 0.), .08
    long_run = ActionWatch(ActionPlan(kind='forward', jog_distance=.45, continuous=True),
                           0, pose, .45, spd, p)
    assert long_run.deadline == pytest.approx(1.5*.45/spd+2.)
    assert long_run.deadline > p('action_timeout_sec')
    normal = ActionWatch(ActionPlan(kind='forward', jog_distance=.10),
                         0, pose, .10, spd, p)
    assert normal.deadline == pytest.approx(min(p('action_timeout_sec'),
                                                max(p('response_timeout_sec'),
                                                    3*.10/spd+1.)))
    trim = ActionWatch(ActionPlan(kind='forward', jog_distance=-.03),
                       0, pose, .03, spd, p)
    assert trim.deadline <= p('action_timeout_sec'), '修剪不放宽'


# ── 行进中航向保持 (纯直行区那一步连续直行) ────────────────────────────

HOLD_KW = dict(rate=.12, engage=math.radians(2.), release=math.radians(.7),
               min_engage_ns=int(.15e9), cooldown_ns=int(.3e9),
               budget=math.radians(15.))
DT = int(.05e9)      # 20Hz 控制周期
DEAD = 0.10          # l1w_control min_angular_z


def held(rate=.12, **changes):
    from tagdocking.action_executor import HeadingHold
    return HeadingHold(**{**HOLD_KW, 'rate': rate, **changes})


def jogger(hold, distance=.45, rate=.12):
    """起步一段盲走直行 (= 双码纯直行区那一步), 返回 (executor, 步进函数)。

    步进函数吃"当前 yaw", 按真实速度推进 x, 返回 (done, angular_cmd)。
    """
    from tagdocking.action_executor import ActionExecutor
    ex = ActionExecutor(min_angular_rate=.12)
    ex.start_jog(distance, .08, blind=True, hold=hold)
    ex.set_odom_ref(0., 0., 0.)
    clock = {'ns': int(10e9), 'x': 0.}
    def step(yaw):
        clock['ns'] += DT
        clock['x'] += .08*DT*1e-9
        done = ex.update(clock['x'], 0., yaw, False, None,
                         lambda: 0., lambda d: (0., 0.), 0., .15, clock['ns'])
        return done, ex.angular_cmd
    return ex, step


def test_heading_hold_never_commands_inside_the_chassis_dead_zone():
    """l1w_control 把 |wz| < 0.10 截成 0: 配低了不是"转得慢", 是**根本不转**,
    而日志照打、预算照扣 —— 功能被静默关掉。抬底必须是结构性的, 含 rate=0.0。"""
    for rate in (0.0, .05, .08, .12, .2):
        ex, step = jogger(held(rate=rate))
        done, wz = step(math.radians(3.))      # 左偏 3° → 必接通
        assert not done
        assert abs(wz) >= .12 > DEAD, f'rate={rate} 下发量掉进死区'
        assert wz < 0, f'rate={rate} 符号错: 左偏必须发 CW 才能让 |err| 下降'
        _, wz_r = jogger(held(rate=rate))[1](-math.radians(3.))
        assert wz_r > 0, f'rate={rate} 右偏必须发 CCW'


def test_heading_hold_engages_on_drift_and_releases_before_zero_not_at_zero():
    """瞄 0 断开 + 命令→odom 报告滞后 ⇒ 必过冲换符号 ⇒ 抖振。
    断开点是 release(0.7°) 而非 0, 迟滞带 1.3° 要宽过最短接通粒度 1.03°。"""
    ex, step = jogger(held())
    assert step(math.radians(1.5))[1] == 0, '未达 engage 不接通'
    assert step(math.radians(2.1))[1] < 0, '达 engage 接通'
    for _ in range(3):                          # 熬过 min_engage (3 周期)
        assert step(math.radians(1.4))[1] < 0, '接通期内不因误差回落就松手'
    assert step(math.radians(.9))[1] < 0, '0.9deg 仍在迟滞带内, 保持接通'
    wz = step(math.radians(.6))[1]
    assert wz == 0, '0.6deg <= release 断开'
    assert abs(ex.jog_yaw_error) > 0, '断开时误差必须仍非零 —— 瞄 0 就是在制造过冲'
    assert ex.jog_yaw_error == pytest.approx(math.radians(.6))


def test_heading_hold_cooldown_survives_pessimistic_report_latency():
    """断开后 odom 还在沉降, 此时的读数不能拿来做下一次决策。
    悲观推演: 释放后过冲到反向 3°, 冷却窗内必须一声不吭。"""
    ex, step = jogger(held())
    for yaw in (2.1, 1.4, 1.4, 1.4, .6):        # 接通 → 熬过 min_engage → 断开
        step(math.radians(yaw))
    assert ex.angular_cmd == 0
    engaged = 0
    for i in range(5):                          # 0.30s 冷却 = 6 周期, 前 5 周期在窗内
        wz = step(-math.radians(3.))[1]         # 反向过冲, 幅值远超 engage
        engaged += wz != 0
    assert engaged == 0, f'冷却窗内接通了 {engaged} 次 —— 拿沉降中的读数做了决策'
    assert step(-math.radians(3.))[1] > 0, '冷却到点 (0.30s) 后才允许反向接通'


def test_a_plain_or_blind_jog_is_bit_identical_without_the_hold():
    """保持渗漏到泊出/重试盲腿/单码是最坏的回归 —— 那些路径压根不该有角速度。
    hold=None 时不只是 wz 为 0, 判停时序也必须与改动前逐位相同。"""
    from tagdocking.action_executor import ActionExecutor
    ex, step = jogger(None, distance=.20)
    ticks = 0
    while True:
        done, wz = step(math.radians(8.))       # 大幅漂移也绝不响应
        assert wz == 0, 'hold=None 却发出了角速度'
        ticks += 1
        if done:
            break
        assert ticks < 200
    assert ticks == math.ceil(.20/(.08*.05)), '判停时序被改动'
    # blind 仍跳过视觉早停: 视觉说"已到位"也必须走完整条腿
    ex2 = ActionExecutor(min_angular_rate=.12)
    ex2.start_jog(.20, .08, blind=True)
    ex2.set_odom_ref(0., 0., 0.)
    assert not ex2.update(.01, 0., 0., True, .01, lambda: 0.,
                          lambda d: (0., 0.), .50, .15, int(10e9))


def test_heading_hold_budget_is_a_diagnostic_gate_that_degrades_to_straight():
    """预算不是安全边界 (那由符号规则给), 是"底盘不响应 wz / odom yaw 疯了"
    的闸。耗尽后退化终点必须正好是 wz≡0 = 不做这件事的老行为, 且可上报。"""
    ex, step = jogger(held(budget=math.radians(3.)), distance=3.0)
    for _ in range(40):
        step(math.radians(5.))                  # 持续大漂移, 保持怎么发都追不回
    assert ex.jog_hold_spent, '预算耗尽未置位 —— 现场无从判断功能是否还活着'
    assert ex.jog_hold_used >= math.radians(3.)
    assert ex.angular_cmd == 0
    for _ in range(20):
        assert step(math.radians(5.))[1] == 0, '耗尽后又接通了'
    # 读数故障 (|err| > 20°) 同样只退化到纯直行, 不中止动作
    ex2, step2 = jogger(held())
    assert not step2(math.radians(30.))[0]
    assert step2(math.radians(30.))[1] == 0 and ex2.jog_hold_spent


def test_straight_consistency_veto_tolerates_the_heading_hold_arc():
    """ActionWatch 的直线一致性判据 (progress < target*0.8) 角度等价物是
    arccos(0.8)=36.9°。保持的最坏情形 15° 必须放行, 而判据本身仍要能抓到
    真正的横向漂移 —— 少了后半句就只是在测"什么都不发生"。"""
    p = tuned().p
    run = ActionPlan(kind='forward', jog_distance=.45, continuous=True)
    def drift(deg):
        w = ActionWatch(run, 0, (0., 0., 0.), .45, .08, p)
        d = math.radians(deg)
        return w.check(int(3e9), (.45*math.cos(d), .45*math.sin(d), d), int(3e9))
    assert drift(15.) == '', '15deg 偏航 (预算全用光) 被误判'
    assert drift(30.) == '', '30deg 仍在 36.9deg 判据内'
    assert 'inconsistent' in drift(45.), '45deg 横向漂移必须照抓 —— 判据本身仍有效'
