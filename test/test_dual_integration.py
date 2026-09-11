"""Integration of actual method bodies with fake ROS transport; no node startup."""
import ast
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

from tagdocking.state_machine import DockingStateMachine, DockingState
from tagdocking.utils import TagPose
from test_dual_docking import Node, controller, frames, plan

ROOT = Path(__file__).resolve().parents[1]


def method_class(file, name, env):
    """Load production class without ROS import side effects / initialization."""
    tree = ast.parse((ROOT / 'tagdocking' / file).read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == name)
    cls.bases = []
    env.update({'__name__': 'test_transport', 'AprilTagDetectionArray': object,
                'Odometry': object, 'Trigger': NS(Request=lambda: None),
                'String': object, 'Bool': object, 'ActionPlan': object,
                'DockingState': DockingState})
    exec(compile(ast.Module(body=[cls], type_ignores=[]), file, 'exec'), env)
    return env[name]


def clock_node():
    node = Node()
    node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=int(20e9)))
    return node


def test_dual_state_machine_no_single_tag_success_search_or_retry():
    sm = DockingStateMachine(clock_node())
    sm._state = DockingState.FINAL_SERVO
    params = {'dual_enable': True}
    def tick(visible):
        return sm.evaluate(TagPose(.1, 0, 0, int(20e9), 0), visible,
                           0, 0, 0, 0, 0, 0, False, int(20e9), params)
    assert tick(True) == DockingState.FINAL_SERVO
    for _ in range(60):
        tick(False)
    assert sm.state == DockingState.MOTION_FAILED
    sm._state = DockingState.APPROACH
    sm.finish_dual()
    assert sm.state == DockingState.DOCKED
    sm.cancel()
    sm.finish_dual()
    assert sm.state == DockingState.CANCELLED


@pytest.mark.parametrize('outcome', ['ok', 'reject', 'exception', 'timeout', 'disabled'])
def test_passive_acceptance_switch_and_one_shot(outcome):
    cls = method_class('charge_mode.py', 'ChargeMode', {})
    c = cls.__new__(cls)
    node = clock_node()
    node.params['charge.passive'] = outcome != 'disabled'
    c._node = node
    c._phase = c.IDLE
    c._static_ok = False
    c._begin_ns = 0
    c._static_ack_ns = int(3e9)
    c._service_wait_ns = int(1e9)
    c._damp_settle_ns = int(2e9)
    calls = []
    def result():
        if outcome == 'exception':
            raise RuntimeError('transport failed')
        return NS(success=outcome == 'ok')
    future = NS(done=lambda: outcome != 'timeout', result=result, cancel=lambda: None)
    c._cli_passive = NS(service_is_ready=lambda: True,
        call_async=lambda req: calls.append(req) or future)
    c.begin(int(10e9))
    c.begin(int(10e9))
    for sec in (10, 11, 12, 14):
        c.tick(int(sec*1e9))
    assert len(calls) == (0 if outcome == 'disabled' else 1)
    assert c._phase == (c.DONE if outcome in ('ok', 'disabled') else c.FAILED)


def test_explicit_false_launch_always_overrides_yaml():
    source = (ROOT / 'launch' / 'docking.launch.py').read_text()
    assert "'dual.enable': dual_enabled" in source
    assert 'if dual_enabled else []' not in source
    # Never call launch_setup: it contains process-killing side effects.
    ast.parse(source)


def test_pending_expiry_does_not_start_or_qualify():
    cls = method_class('docking_node.py', 'DockingNode', {})
    c = controller()
    n = frames(c)
    seq = plan(c, n)
    node = cls.__new__(cls)
    node._dual = c
    node._pending_seq = seq
    stops = []
    node._adapter = NS(publish_stop=lambda: stops.append(True))
    node._reset_visual_state = lambda: None
    # No posture/executor attributes: reaching either is a test failure.
    assert not node._launch_pending_seq('omni', n+int(1e9))
    assert node._pending_seq is None and stops and not c.progress


def test_stop_precedes_terminal_confirmation():
    cls = method_class('docking_node.py', 'DockingNode', {'math': __import__('math')})
    node = cls.__new__(cls)
    c = controller()
    c.stage = 'locked'
    c.progress = True
    n = frames(c, depth=.5, missing=True)
    node._dual = c
    node._executor = NS(is_active=False, wait_visual_settle=lambda *a: False,
                        cancel=lambda: events.append('cancel'))
    events = []
    node._adapter = NS(publish_stop=lambda: events.append('stop'))
    node._posture = NS(lock_settled=lambda n: True)
    node._frozen = False
    node._pending_seq = None
    node._has_odom = True
    node._planner = None
    node._lookup_camera_offset = lambda: None
    node._sm = NS(finish_dual=lambda: events.append('docked'))
    node._run_stop_and_go(True, None, 'omni', n)
    assert events[-3:] == ['cancel', 'stop', 'docked']


@pytest.mark.parametrize('offset', [0, 30000000, -1000000000])
def test_detection_tf_exact_stamp_and_skew_rejection(offset):
    import math
    error = type('TFError', (Exception,), {})
    env = {'math': math, 'TagPose': TagPose,
           'tf2_ros': NS(LookupException=error, ConnectivityException=error,
                         ExtrapolationException=error),
           'rclpy': NS(time=NS(Time=lambda **kw: kw),
                       duration=NS(Duration=lambda **kw: kw))}
    cls = method_class('docking_node.py', 'DockingNode', env)
    node = cls.__new__(cls)
    node._dual = controller()
    node._dual_info = {int(20e9): node._dual.camera}
    node._dual_pending = []
    node._dual_received_ns = node._dual_window_ns = 0
    node._dual_diag_ns = {}
    node.get_logger = lambda: NS(info=lambda *a, **kw: None)
    node._frozen = False
    node._sm = NS(state=DockingState.APPROACH)
    node._p = lambda name: {'tag.id': 0, 'tag.frame': 'wall',
                            'camera_frame': 'optical'}[name]
    node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=int(20e9)))
    queries, poses = [], []
    def lookup(target, source, when, timeout):
        queries.append((target, source, when, timeout))
        stamp = int(20e9) + offset
        return NS(header=NS(stamp=NS(sec=stamp//1000000000,
                                     nanosec=stamp%1000000000)),
                  transform=NS(translation=NS(x=0., y=0., z=1.5 if source == 'wall' else .6)))
    node._tf_buffer = NS(lookup_transform=lookup)
    node._pose_buffer = NS(add=poses.append, clear=poses.clear)
    msg = NS(header=NS(stamp=NS(sec=20, nanosec=0)),
             detections=[NS(id=0), NS(id=51)])
    node._on_dual_detections(msg)
    assert queries[0][2] == {'nanoseconds': int(20e9)}
    assert queries[0][3] == {'seconds': 0}
    assert bool(poses) == (offset == 0)
    assert node._dual.frames == (1 if offset == 0 else 0)
    node._on_dual_detections(msg)
    assert node._dual.frames <= 1


@pytest.fixture
def intake():
    """Actual callbacks + actual controller/PoseBuffer, only ROS transport faked."""
    import math
    from tagdocking.pose_buffer import PoseBuffer
    error = type('TFError', (Exception,), {})
    env = {'math': math, 'TagPose': TagPose,
           'normalize_angle': lambda angle: angle,
           'tf2_ros': NS(LookupException=error, ConnectivityException=error,
                         ExtrapolationException=error),
           'rclpy': NS(time=NS(Time=lambda **kw: kw),
                       duration=NS(Duration=lambda **kw: kw))}
    cls = method_class('docking_node.py', 'DockingNode', env)
    n = cls.__new__(cls)
    n._dual = controller()
    n._dual_info = {}
    n._dual_pending = []
    n._dual_received_ns = n._dual_window_ns = n._last_detection_ns = 0
    n._dual_diag_ns = {}
    n._frozen = False
    n._det_msg_count = 0
    n._sm = NS(state=DockingState.APPROACH)
    n._p = lambda name: {'tag.id': 0, 'tag.frame': 'wall',
        'camera_frame': 'optical', 'base.type': 'omni',
        'dock_target.lateral_offset': 0., 'dock_target.yaw_offset_deg': 0.,
        'tag.fresh_timeout_sec': 2.0}[name]
    n.now = int(20e9)
    n.get_clock = lambda: NS(now=lambda: NS(nanoseconds=n.now))
    n.logs, n.queries, n.events = [], [], []
    n.get_logger = lambda: NS(info=lambda text: n.logs.append(text))
    n._adapter = NS(publish_stop=lambda: n.events.append('stop'))
    n._executor = NS(cancel=lambda: n.events.append('cancel'), is_active=False)
    n._sm.abort_motion = lambda reason: n.events.append(reason)
    n._pose_buffer = PoseBuffer(max_latency_ns=150_000_000)
    n.available = set()
    def lookup(target, frame, when, timeout):
        stamp = when['nanoseconds']
        assert timeout == {'seconds': 0}
        n.queries.append((frame, stamp))
        if (frame, stamp) not in n.available:
            raise error('not yet available')
        return NS(header=NS(stamp=NS(sec=stamp//10**9, nanosec=stamp%10**9)),
                  transform=NS(translation=NS(x=0., y=0., z=1.5 if frame == 'wall' else .6)))
    n._tf_buffer = NS(lookup_transform=lookup)
    def detect(stamp=None, ids=(0, 51)):
        stamp = n.now if stamp is None else stamp
        n._dual_info[stamp] = n._dual.camera
        n._on_detections(NS(header=NS(stamp=NS(sec=stamp//10**9,
            nanosec=stamp%10**9)), detections=[NS(id=i) for i in ids]))
    def supply(stamp, pile=True):
        n.available.add(('wall', stamp))
        if pile:
            n.available.add((n._dual.pile_frame, stamp))
    n.detect, n.supply = detect, supply
    return n


def test_detection_before_tf_consumed_by_control_before_gather(intake):
    n = intake
    stamp = n.now
    n.detect()
    assert len(n._dual_pending) == 1 and not n._tag_fresh()
    n.now += 200_000_000
    n.supply(stamp)
    class Gathered(Exception):
        pass
    def evaluate(**inputs):
        assert inputs['tag_visible']
        assert inputs['tag_pose'].stamp_ns == stamp
        raise Gathered()
    n._sm.evaluate = evaluate
    n._build_params_dict = lambda: {}
    n._maneuver_active = False
    n._odom_x = n._odom_y = n._odom_yaw = 0.
    with pytest.raises(Gathered):
        n._control_loop()
    assert n._dual.frames == 1 and not n._dual_pending
    n._retry_dual_detections(n.now)
    assert n._dual.frames == 1


def test_continuous_late_tf_fifo_no_starvation(intake):
    n = intake
    stamps = []
    for i in range(25):
        stamp = n.now
        stamps.append(stamp)
        n.detect()
        if i >= 3:
            n.supply(stamps[i-3])
        n._retry_dual_detections(n.now)
        n.now += 50_000_000
    assert n._dual.frames == 22
    assert n._last_detection_ns == stamps[-4]
    assert [stamp for stamp, _ in n._dual_pending] == stamps[-3:]
    assert n._get_latest_pose().stamp_ns == stamps[-4]


@pytest.mark.parametrize('age,valid', [(200_000_000, True), (600_000_000, True),
                                      (600_000_001, False)])
def test_dual_freshness_real_pose_buffer(intake, age, valid):
    n = intake
    stamp = n.now
    n.supply(stamp)
    n.detect()
    n.now += age
    assert n._tag_fresh() == valid
    pose = n._get_latest_pose()
    assert (pose is not None) == valid
    if pose:
        assert pose.stamp_ns == stamp


def test_single_pose_latency_unchanged(intake):
    n = intake
    n._dual.enabled = False
    n._last_detection_ns = n.now
    n._pose_buffer.add(TagPose(1., 0., 0., n.now, 0.))
    n.now += 200_000_000
    assert n._tag_fresh()  # Original independent single-tag freshness rule.
    assert n._get_latest_pose() is None
    assert n._pose_buffer.max_latency_ns == 150_000_000


def test_wait_does_not_invalidate_accepted_pose_and_expiry_does(intake):
    n = intake
    n.supply(n.now)
    n.detect()
    original = n.now
    n.now += 50_000_000
    n.detect()
    assert n._dual.frames == 1
    assert n._get_latest_pose().stamp_ns == original
    n.now += 600_000_001
    n._retry_dual_detections(n.now)
    assert not n._dual_pending and not n._tag_fresh()
    assert n._get_latest_pose() is None and n._dual.frames == 0
    assert any('TF waiting' in line for line in n.logs)
    assert any('expired' in line for line in n.logs)


def test_present_pile_tf_late_not_counted_as_missing(intake):
    n = intake
    stamp = n.now
    n.supply(stamp, pile=False)
    n.detect()
    assert not n._dual.missing_ns and not n._dual.wall_frames
    n.now += 200_000_000
    n.supply(stamp)
    n._retry_dual_detections(n.now)
    assert n._dual.frames == 1 and n._dual.missing_ns == 0
    assert n._get_latest_pose().stamp_ns == stamp


def test_absent_pile_uses_wall_only_without_pile_tf(intake):
    n = intake
    n.supply(n.now, pile=False)
    n.detect(ids=(0,))
    assert n._dual.wall_frames == 1 and n._dual.frames == 0
    assert n._dual.missing_ns == n.now
    assert n.queries == [('wall', n.now)]


def test_missing_wall_clears_pose_and_pending(intake):
    n = intake
    n.supply(n.now)
    n.detect()
    n.now += 50_000_000
    pending_stamp = n.now
    n.detect()
    n.now += 50_000_000
    n.detect(ids=(51,))
    n.supply(pending_stamp)
    n._retry_dual_detections(n.now)
    assert not n._dual_pending and n._pose_buffer.empty
    assert not n._tag_fresh() and n._get_latest_pose() is None


def test_duplicates_out_of_order_future_and_rejected_old_absence(intake):
    n = intake
    stamp = n.now
    n.supply(stamp)
    n.detect()
    queries = len(n.queries)
    n.detect()
    n.detect(stamp-1, ids=())
    n.detect(stamp+1)  # Future does not poison the watermark.
    assert len(n.queries) == queries and n._dual.frames == 1
    assert n._tag_fresh()
    n.now += 1
    n.supply(n.now)
    n.detect()
    assert n._dual.frames == 2
    assert any('rejected' in line for line in n.logs)


@pytest.mark.parametrize('boundary', ['freeze', 'reset', 'cancel', 'settle'])
def test_pending_and_late_packets_cannot_cross_windows(intake, boundary):
    n = intake
    old = n.now
    n.detect()
    n.now += 50_000_000
    if boundary == 'freeze':
        n._frozen = True
        n._retry_dual_detections(n.now)
        n._reset_visual_state()
    elif boundary == 'reset':
        n._reset_maneuver()  # Actual task reset, including DualTagDocking.reset().
    elif boundary == 'cancel':
        n._sm.state = DockingState.CANCELLED
        n._retry_dual_detections(n.now)
        n._reset_maneuver()
        n._sm.state = DockingState.APPROACH
    else:
        n._dual.stopped(n.now)
        n._retry_dual_detections(n.now)
        n.now = n._dual.settle_until_ns
    n.supply(old)
    n.detect(old)
    n._retry_dual_detections(n.now)
    assert n._dual.frames == 0 and n._pose_buffer.empty
    n.now += 1
    n.supply(n.now)
    n.detect()
    assert n._dual.frames == 1


def test_locked_frozen_only_watchdog_and_missing_wall_stops(intake):
    n = intake
    n._dual.stage = 'locked'
    n._frozen = True
    stamp = n.now
    n.detect()
    n.now += 200_000_000
    n.supply(stamp, pile=False)
    n._retry_dual_detections(n.now)
    assert n._dual_live_wall_ns == stamp
    assert n._dual.frames == 0 and n._pose_buffer.empty
    n.detect(ids=())
    assert n.events[:2] == ['stop', 'cancel'] and n._dual_live_wall_ns == 0


def test_bounded_queue_retains_oldest_and_throttles_wait_diagnostics(intake):
    n = intake
    first = n.now
    for _ in range(100):
        n.detect()
        n.now += 1_000_000
    assert len(n._dual_pending) == 64
    assert n._dual_pending[0][0] == first
    assert sum('TF waiting' in line for line in n.logs) == 1
    n.supply(first)
    n._retry_dual_detections(n.now)
    assert n._dual.frames == 1 and n._last_detection_ns == first


def test_expired_head_does_not_block_fresh_tail(intake):
    n = intake
    n.detect()
    n.now += 550_000_000
    tail = n.now
    n.supply(tail)
    n.detect()
    assert not n._dual.frames  # FIFO: tail cannot overtake waiting head.
    n.now += 50_000_001
    n._retry_dual_detections(n.now)
    assert n._dual.frames == 1 and n._last_detection_ns == tail
    assert not n._dual_pending


def test_present_pile_tf_never_arrives_expires_without_loss_qualification(intake):
    n = intake
    n.supply(n.now, pile=False)
    n.detect()
    n.now += 600_000_001
    n._retry_dual_detections(n.now)
    assert not n._dual_pending and not n._dual.missing_ns
    assert n._dual.wall_frames == 0 and n._get_latest_pose() is None


def test_start_cancel_services_clear_queue_and_watchdog(intake):
    n = intake
    n._planner = NS(reset=lambda: None)
    n._charge = NS(reset=lambda: None)
    n._sm.start = lambda: True
    n._sm.cancel = lambda: setattr(n._sm, 'state', DockingState.CANCELLED)
    n._sm.state_name = 'search_tag'
    old = n.now
    n.detect()
    n._dual_live_wall_ns = old
    n.now += 50_000_000
    n._on_cancel_docking(None, NS())
    assert not n._dual_pending and not hasattr(n, '_dual_live_wall_ns')
    n._sm.state = DockingState.SEARCH_TAG
    n.now += 50_000_000
    n._on_start_docking(None, NS())
    n.supply(old)
    n.detect(old)
    assert n._dual.frames == 0 and n._pose_buffer.empty
    n.now += 1
    n.supply(n.now)
    n.detect()
    assert n._dual.frames == 1


def test_dual_fixed_key_log_suppression_and_stage_change():
    cls = method_class('docking_node.py','DockingNode',{})
    n = cls.__new__(cls)
    n._dual,n._dual_diag_ns = controller(),{}
    messages = []
    n.get_logger = lambda: NS(info=messages.append)
    n._dual_log('found','first',int(10e9))
    n._dual_log('found','hidden',int(11e9))
    n._dual_log('found','next',int(12e9))
    n._dual.stage = 'observe'
    n._dual_log('found','stage',int(12.1e9))
    assert messages == ['first suppressed=0','next suppressed=1','stage suppressed=0']


@pytest.mark.parametrize('busy', [True,False])
def test_executor_rejected_start_does_not_commit_qualification(busy):
    import math
    from tagdocking.geometry_planner import ActionPlan
    cls = method_class('docking_node.py','DockingNode',{'math':math})
    n = cls.__new__(cls)
    n._dual = controller()
    now = frames(n._dual)
    n.get_clock = lambda: NS(now=lambda: NS(nanoseconds=now))
    n._odom_stamp_ns = now
    n._odom_x = n._odom_y = n._odom_yaw = 0.
    n._executor = NS(is_active=busy,start_turn=lambda *a,**kw:False)
    n._p = lambda name: .1
    assert not n._launch_step(ActionPlan(kind='yaw',turn_angle=.001),'omni')
    assert n._dual.actions == 0 and not n._dual.qualified_ns and not n._dual._travel_qualification


def test_actual_camera_callback_validates_and_caches_original_stamp():
    from tagdocking.dual_camera import CameraModel
    from test_dual_predictive import info
    cls = method_class('docking_node.py','DockingNode',{'CameraModel':CameraModel})
    n = cls.__new__(cls)
    n._dual,n._dual_info = controller(),{}
    n._p = lambda name: 'raw' if name=='dual.projection_mode' else 'optical'
    errors = []
    n.get_logger = lambda: NS(error=lambda text, **kwargs: errors.append((text, kwargs)))
    msg = info()
    msg.header.stamp = NS(sec=20,nanosec=123)
    n._on_dual_camera_info(msg)
    assert list(n._dual_info) == [20000000123]
    msg.width = 0
    n._on_dual_camera_info(msg)
    assert n._dual.camera is None and 'CameraInfo invalid' in n._dual.failure
    assert errors[-1][1] == {'throttle_duration_sec': 2.0}


def prealign_node(**params):
    """真实 _dual_prealign 方法体 + 假 transport; 不启动节点、不调服务。"""
    from tagdocking.geometry_planner import ActionPlan
    env = {'math': __import__('math')}
    cls = method_class('docking_node.py', 'DockingNode', env)
    env['ActionPlan'] = ActionPlan          # method_class 默认把它桩成 object
    n = cls.__new__(cls)
    n._dual = controller()
    for name, value in params.items():
        n._dual._node.params['dual.' + name] = value
    n._dual_prealigned = False
    n._dual_prealign_steps = 0
    n._dual_prealign_active = False
    n.events = []
    n._adapter = NS(publish_stop=lambda: n.events.append('stop'))
    n.get_logger = lambda: NS(info=lambda *a, **kw: None,
                              warn=lambda *a, **kw: n.events.append('warn'))
    n._p = lambda name: {'stopgo.max_turn_step': .17}[name]
    n._pending_seq = None
    n._launch_pending_seq = lambda bt, now: n.events.append('launch')
    return n


def test_prealign_turns_toward_the_wall_tag_with_a_bounded_step():
    """锁定时的十几度方位误差必须在趴下之前用单码大步收掉。

    符号: bearing = atan2(lat, dist) > 0 = 墙码在车左 → 左转 (turn_angle > 0)。
    """
    n = prealign_node(prealign_step_deg=8.)
    pose = TagPose(dist=1.371, lat=+.5, yaw=0., stamp_ns=int(20e9), normal=0.)
    assert not n._dual_prealign(True, pose, 'omni', int(20e9))
    assert not n._dual_prealigned and n._dual_prealign_active
    assert n.events == ['launch']
    step, = n._pending_seq
    assert step.kind == 'yaw'
    assert step.turn_angle == pytest.approx(__import__('math').radians(8))
    # 粗对准步绝不进双码的动作预算 / 合格状态。
    assert n._dual.actions == 0 and not n._dual.qualified_ns
    # 镜像: 墙码在车右 → 右转。
    m = prealign_node(prealign_step_deg=8.)
    m._dual_prealign(True, TagPose(dist=1.371, lat=-.5, yaw=0.,
                                   stamp_ns=int(20e9), normal=0.), 'omni', int(20e9))
    assert m._pending_seq[0].turn_angle < 0


def test_prealign_step_is_clamped_by_residual_and_by_max_turn_step():
    math_ = __import__('math')
    # 残余小于步长 → 只转残余, 不过冲。
    n = prealign_node(prealign_step_deg=8., prealign_tolerance_deg=5.)
    bearing = math_.radians(6.)
    n._dual_prealign(True, TagPose(dist=1., lat=math_.tan(bearing), yaw=0.,
                                   stamp_ns=int(20e9), normal=0.), 'omni', int(20e9))
    assert n._pending_seq[0].turn_angle == pytest.approx(bearing, rel=1e-6)
    # stopgo.max_turn_step 仍是硬闸 (保证墙码不转出视野)。
    m = prealign_node(prealign_step_deg=14.)
    m._p = lambda name: {'stopgo.max_turn_step': .10}[name]
    m._dual_prealign(True, TagPose(dist=1., lat=1., yaw=0.,
                                   stamp_ns=int(20e9), normal=0.), 'omni', int(20e9))
    assert m._pending_seq[0].turn_angle == pytest.approx(.10)


def test_prealign_hands_off_when_tight_and_warns_but_hands_off_when_exhausted():
    math_ = __import__('math')
    n = prealign_node(prealign_tolerance_deg=5.)
    tight = math_.radians(3.)
    assert n._dual_prealign(True, TagPose(dist=1.371, lat=math_.tan(tight), yaw=0.,
                                          stamp_ns=int(20e9), normal=0.), 'omni', int(20e9))
    assert n._dual_prealigned and n._pending_seq is None and not n.events
    # 预算耗尽: 告警后仍移交双码 —— 收敛/失败判定的责任统一在双码,
    # 两处都判失败会让同一个故障出现两种说法。
    m = prealign_node(prealign_max_steps=3)
    m._dual_prealign_steps = 3
    assert m._dual_prealign(True, TagPose(dist=1.371, lat=.5, yaw=0.,
                                          stamp_ns=int(20e9), normal=0.), 'omni', int(20e9))
    assert m._dual_prealigned and m.events == ['warn'] and m._pending_seq is None


def test_prealign_waits_for_a_pose_and_never_reengages_after_acquire():
    n = prealign_node()
    assert not n._dual_prealign(False, None, 'omni', int(20e9))
    assert n.events == ['stop'] and n._pending_seq is None
    # observe/approach/locked 相位由双码几何或锁定直行掌方向盘, 单码不得插手 ——
    # 这些相位下连 _dual_prealigned 都不该被读 (locked 直行没有粗对准概念)。
    for stage in ('observe', 'approach', 'locked'):
        m = prealign_node()
        m._dual.stage = stage
        del m._dual_prealigned
        assert m._dual_prealign(True, TagPose(dist=.6, lat=.5, yaw=0.,
                                              stamp_ns=int(20e9), normal=0.),
                                'omni', int(20e9))
        assert m._pending_seq is None and not m.events


def test_prealign_flag_is_cleared_at_every_stop_boundary():
    """漏清是静默故障: 之后真正的双码动作会被当成粗对准步 —— 既不查
    pending_valid 也不记 action_started, 双码的预算/合格状态全部作废。
    队列排空一步都没起来 (步长太小被跳过) 也必须走到这个收口。"""
    n = prealign_node()
    n._dual_prealign_active = True
    n._executor = NS(mark_stop_time=lambda ns: None)
    n._posture = NS(on_stop=lambda ns: None)
    n._mark_stopped(int(21e9))
    assert not n._dual_prealign_active
    assert n._dual.settle_until_ns > int(21e9)   # 双码 settle 照常武装
