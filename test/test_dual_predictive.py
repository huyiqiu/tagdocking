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
    ('feedback_fail_windows',1.5),('min_lateral_m',.04),('pile_tag_size',-1),
    ('straight_yaw_tol_deg',.2),('straight_yaw_tol_deg',5.),
    ('straight_start_distance',1.75)])
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


def test_vertical_exit_requires_completed_qualified_forward_and_never_correction():
    c = controller()
    c.stage = 'approach'
    c.wall,c.pile = (0,-.2,1.),(0,.3,.1)
    c.stamp = int(20e9)
    forward = ActionPlan(kind='forward',jog_distance=.02)
    assert not c.visible(forward)[0]
    c.progress,c.qualified_ns = True,c.stamp
    assert c.visible(forward)[0]
    assert not c.visible(ActionPlan(kind='yaw',turn_angle=.01))[0]
    assert not c.visible(ActionPlan(kind='forward',jog_distance=.003,lateral_distance=.003))[0]
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
            return
        assert not step.turn_angle and not step.lateral_distance and 0 < step.jog_distance <= .05
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
    c = controller()
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
