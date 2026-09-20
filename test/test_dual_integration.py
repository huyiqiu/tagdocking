"""Integration of actual method bodies with fake ROS transport; no node startup."""
import ast
import math
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

from tagdocking.state_machine import DockingStateMachine, DockingState
from tagdocking.state_machine import (
    FAILURE_CODES, CODE_TAG_NOT_FOUND, CODE_VISION_NO_PROGRESS,
    CODE_MOTION_GATED, CODE_MOTION_STALLED, CODE_TIMEOUT, CODE_CANCELLED,
    CODE_UNSPECIFIED)
from tagdocking.utils import TagPose
from tagdocking.action_executor import HeadingHold
from tagdocking.action_executor import ActionPlan as RealActionPlan
from test_dual_docking import Node, controller, frames, plan

ROOT = Path(__file__).resolve().parents[1]


def method_class(file, name, env):
    """Load production class without ROS import side effects / initialization."""
    tree = ast.parse((ROOT / 'tagdocking' / file).read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == name)
    cls.bases = []
    # HeadingHold 出现在 _heading_hold_params 的返回标注里, 而标注在 def 求值
    # 时就要解析 (本仓库没开 from __future__ import annotations), 所以它必须
    # 进 env —— 不是只在调用时才需要。
    env.update({'__name__': 'test_transport', 'AprilTagDetectionArray': object,
                'Odometry': object, 'Trigger': NS(Request=lambda: None),
                'String': object, 'Bool': object, 'ActionPlan': object,
                'HeadingHold': HeadingHold,
                'DockingState': DockingState,
                # 失败码常量: 方法体里的 abort_motion(..., CODE_*) 会在**调用
                # 时**去 env 里找它们, 不进来就是 NameError。
                'CODE_TAG_NOT_FOUND': CODE_TAG_NOT_FOUND,
                'CODE_VISION_NO_PROGRESS': CODE_VISION_NO_PROGRESS,
                'CODE_MOTION_GATED': CODE_MOTION_GATED,
                'CODE_MOTION_STALLED': CODE_MOTION_STALLED})
    exec(compile(ast.Module(body=[cls], type_ignores=[]), file, 'exec'), env)
    return env[name]


def clock_node():
    node = Node()
    node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=int(20e9)))
    return node


def test_dual_state_machine_no_single_tag_success_search_or_retry():
    sm = DockingStateMachine(clock_node())
    sm._state = DockingState.APPROACH
    params = {}
    def tick(visible):
        return sm.evaluate(TagPose(.1, 0, 0, int(20e9), 0), visible,
                           False, int(20e9), params)
    assert tick(True) == DockingState.APPROACH
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


def test_motion_ready_stands_up_before_docking_and_resets_recovery():
    cls = method_class('charge_mode.py', 'ChargeMode', {})
    c = cls.__new__(cls)
    logs, calls, failures = [], [], []
    c._node = NS(get_logger=lambda: NS(info=logs.append))
    c._sm = NS(abort_motion=lambda reason, code: failures.append((reason, code)))
    c._enable = True
    c._phase = c.IDLE
    c._motion_enabled = False
    c._posture_state = 'not_standing'
    c._static_ack_ns = int(3e9)
    c._retries = 1
    c._step_tries = 0
    c._step_future = None
    c._cli_stand = NS(call_async=lambda request: calls.append(request) or NS())

    assert not c.motion_ready(int(10e9), operation='停泊')
    assert c._phase == c.RECOVERING and len(calls) == 1
    assert '停泊请求' in logs[-1]

    c._motion_enabled = True
    assert c.motion_ready(int(11e9), operation='停泊')
    assert c._phase == c.IDLE and c._step_future is None
    assert not failures and '运动模式已恢复' in logs[-1]


def test_motion_ready_docking_timeout_reports_motion_gated():
    cls = method_class('charge_mode.py', 'ChargeMode', {})
    c = cls.__new__(cls)
    failures = []
    c._node = NS(get_logger=lambda: NS(info=lambda text: None))
    c._sm = NS(abort_motion=lambda reason, code: failures.append((reason, code)))
    c._enable = True
    c._phase = c.IDLE
    c._motion_enabled = False
    c._posture_state = 'not_standing'
    c._static_ack_ns = int(3e9)
    c._retries = 0
    c._step_tries = 0
    c._step_future = None
    c._cli_stand = NS(call_async=lambda request: NS())

    assert not c.motion_ready(int(10e9), operation='停泊')
    assert not c.motion_ready(int(13.1e9), operation='停泊')
    assert c._phase == c.FAILED
    assert failures and failures[0][1] == CODE_MOTION_GATED
    assert '停泊前 stand_up 超时' in failures[0][0]


def test_launch_applies_dual_params_unconditionally():
    source = (ROOT / 'launch' / 'docking.launch.py').read_text()
    # dual 恒开: launch 不再有 dual_enable 开关, 也不再写 'dual.enable';
    # 双码参数 (tag 尺寸/ID) 必须无条件覆盖 yaml, 保证 apriltag 与
    # docking_node 两侧的 TF frame 名一致。
    assert 'dual_enable' not in source
    assert "'dual.enable'" not in source
    assert "'dual.wall_tag_size': wall_tag_size" in source
    # Never call launch_setup: it contains process-killing side effects.
    ast.parse(source)


def test_pending_expiry_does_not_start_or_qualify():
    cls = method_class('docking_node.py', 'DockingNode', {})
    c = controller()
    n = frames(c)
    seq = plan(c, n)
    node = cls.__new__(cls)
    node._dual = c
    node._dual_prealign_active = False   # _launch_pending_seq 的粗对准豁免读它
    node._pending_seq = seq
    stops = []
    node._adapter = NS(publish_stop=lambda: stops.append(True))
    node._reset_visual_state = lambda: None
    # No executor attributes: reaching it is a test failure.
    assert not node._launch_pending_seq(n+int(1e9))
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
    node._frozen = False
    node._pending_seq = None
    node._has_odom = True
    node._planner = None
    node._lookup_camera_offset = lambda: None
    node._sm = NS(finish_dual=lambda: events.append('docked'))
    node._run_stop_and_go(True, None, n)
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
    # _reset_maneuver / _launch_pending_seq 会读写的手搭属性 (__init__ 里有,
    # 这里必须补齐, 否则 reset/cancel 族用例 AttributeError):
    n._dual_watch = None
    n._dual_prealign_active = False
    n._dock_motion_pending = False
    n._dual_prealigned = False
    n._dual_prealign_steps = 0
    n._dual_reject_last = None
    n._dual_reject_count = {}
    n._sm = NS(state=DockingState.APPROACH)
    n._p = lambda name: {'tag.id': 0, 'tag.frame': 'wall',
        'camera_frame': 'optical'}[name]
    n.now = int(20e9)
    n.get_clock = lambda: NS(now=lambda: NS(nanoseconds=n.now))
    n.logs, n.queries, n.events = [], [], []
    n.get_logger = lambda: NS(info=lambda text: n.logs.append(text))
    n._adapter = NS(publish_stop=lambda: n.events.append('stop'))
    n._executor = NS(cancel=lambda: n.events.append('cancel'), is_active=False)
    # 桩要收下 code (生产代码每处 abort 都带码了); events 仍只记 reason,
    # 这一族用例咬的是文案可读性, 码由 FAILURE_PATHS 那一节咬。
    n._sm.abort_motion = lambda reason, code='': n.events.append(reason)
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
    assert n._dock_motion_pending
    n.supply(old)
    n.detect(old)
    assert n._dual.frames == 0 and n._pose_buffer.empty
    n.now += 1
    n.supply(n.now)
    n.detect()
    assert n._dual.frames == 1


def test_docking_motion_gate_waits_then_starts_from_fresh_search():
    cls = method_class('docking_node.py', 'DockingNode', {})
    n = cls.__new__(cls)
    n._dock_motion_pending = True
    n._sm = NS(state=DockingState.SEARCH_TAG)
    events, operations = [], []
    ready = iter((False, True))
    n._adapter = NS(publish_stop=lambda: events.append('stop'))
    n._charge = NS(motion_ready=lambda now, operation: (
        operations.append(operation) or next(ready)))
    n._reset_search = lambda: events.append('reset_search')
    n._prev_state = DockingState.IDLE
    n.get_logger = lambda: NS(info=lambda text: events.append(text))

    assert not n._prepare_docking_motion(int(10e9))
    assert n._dock_motion_pending and events == ['stop']
    assert not n._prepare_docking_motion(int(11e9))
    assert not n._dock_motion_pending
    assert n._prev_state == DockingState.SEARCH_TAG
    assert events[1:3] == ['stop', 'reset_search']
    assert operations == ['停泊', '停泊']
    assert n._prepare_docking_motion(int(12e9))


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
    from tagdocking.action_executor import ActionPlan
    cls = method_class('docking_node.py','DockingNode',{'math':math})
    n = cls.__new__(cls)
    n._dual = controller()
    now = frames(n._dual)
    n.get_clock = lambda: NS(now=lambda: NS(nanoseconds=now))
    n._odom_stamp_ns = now
    n._odom_x = n._odom_y = n._odom_yaw = 0.
    n._executor = NS(is_active=busy,start_turn=lambda *a,**kw:False)
    n._p = lambda name: .1
    assert not n._launch_step(ActionPlan(kind='yaw',turn_angle=.001))
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
    from tagdocking.action_executor import ActionPlan
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
    n._launch_pending_seq = lambda now: n.events.append('launch')
    return n


def test_prealign_turns_toward_the_wall_tag_with_a_bounded_step():
    """锁定时的十几度方位误差必须在趴下之前用单码大步收掉。

    符号: bearing = atan2(lat, dist) > 0 = 墙码在车左 → 左转 (turn_angle > 0)。
    """
    n = prealign_node(prealign_step_deg=8.)
    pose = TagPose(dist=1.371, lat=+.5, yaw=0., stamp_ns=int(20e9), normal=0.)
    assert not n._dual_prealign(True, pose, int(20e9))
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
                                   stamp_ns=int(20e9), normal=0.), int(20e9))
    assert m._pending_seq[0].turn_angle < 0


def test_prealign_step_is_clamped_by_residual_and_by_max_turn_step():
    math_ = __import__('math')
    # 残余小于步长 → 只转残余, 不过冲。
    n = prealign_node(prealign_step_deg=8., prealign_tolerance_deg=5.)
    bearing = math_.radians(6.)
    n._dual_prealign(True, TagPose(dist=1., lat=math_.tan(bearing), yaw=0.,
                                   stamp_ns=int(20e9), normal=0.), int(20e9))
    assert n._pending_seq[0].turn_angle == pytest.approx(bearing, rel=1e-6)
    # stopgo.max_turn_step 仍是硬闸 (保证墙码不转出视野)。
    m = prealign_node(prealign_step_deg=14.)
    m._p = lambda name: {'stopgo.max_turn_step': .10}[name]
    m._dual_prealign(True, TagPose(dist=1., lat=1., yaw=0.,
                                   stamp_ns=int(20e9), normal=0.), int(20e9))
    assert m._pending_seq[0].turn_angle == pytest.approx(.10)


def test_prealign_hands_off_when_tight_and_warns_but_hands_off_when_exhausted():
    math_ = __import__('math')
    n = prealign_node(prealign_tolerance_deg=5.)
    tight = math_.radians(3.)
    assert n._dual_prealign(True, TagPose(dist=1.371, lat=math_.tan(tight), yaw=0.,
                                          stamp_ns=int(20e9), normal=0.), int(20e9))
    assert n._dual_prealigned and n._pending_seq is None and not n.events
    # 预算耗尽: 告警后仍移交双码 —— 收敛/失败判定的责任统一在双码,
    # 两处都判失败会让同一个故障出现两种说法。
    m = prealign_node(prealign_max_steps=3)
    m._dual_prealign_steps = 3
    assert m._dual_prealign(True, TagPose(dist=1.371, lat=.5, yaw=0.,
                                          stamp_ns=int(20e9), normal=0.), int(20e9))
    assert m._dual_prealigned and m.events == ['warn'] and m._pending_seq is None


def test_prealign_waits_for_a_pose_and_never_reengages_after_acquire():
    n = prealign_node()
    assert not n._dual_prealign(False, None, int(20e9))
    assert n.events == ['stop'] and n._pending_seq is None
    # observe/approach/locked 相位由双码几何或锁定直行掌方向盘, 单码不得插手 ——
    # 这些相位下连 _dual_prealigned 都不该被读 (locked 直行没有粗对准概念)。
    for stage in ('observe', 'approach', 'locked'):
        m = prealign_node()
        m._dual.stage = stage
        del m._dual_prealigned
        assert m._dual_prealign(True, TagPose(dist=.6, lat=.5, yaw=0.,
                                              stamp_ns=int(20e9), normal=0.),
                                int(20e9))
        assert m._pending_seq is None and not m.events


def test_prealign_flag_is_cleared_at_every_stop_boundary():
    """漏清是静默故障: 之后真正的双码动作会被当成粗对准步 —— 既不查
    pending_valid 也不记 action_started, 双码的预算/合格状态全部作废。
    队列排空一步都没起来 (步长太小被跳过) 也必须走到这个收口。"""
    n = prealign_node()
    n._dual_prealign_active = True
    n._executor = NS(mark_stop_time=lambda ns: None)
    n._mark_stopped(int(21e9))
    assert not n._dual_prealign_active
    assert n._dual.settle_until_ns > int(21e9)   # 双码 settle 照常武装


# ── 行进中航向保持: 节点侧的两道门 ─────────────────────────────────────

HOLD_PARAMS = {'stopgo.heading_hold_enable': True, 'stopgo.heading_hold_rate': .12,
               'stopgo.heading_hold_engage_deg': 2.0,
               'stopgo.heading_hold_release_deg': .7,
               'stopgo.heading_hold_min_engage_sec': .15,
               'stopgo.heading_hold_cooldown_sec': .30,
               'stopgo.heading_hold_budget_deg': 15.0,
               # start_jog 的 rate 抬底基准; budget 守卫要按抬底后的 rate 算门槛。
               'stopgo.min_angular_rate': .12}


def hold_node(**overrides):
    """DockingNode 的真实方法体 + 假参数表, 不启 ROS。"""
    import math as _math
    from tagdocking.dual_feedback import ActionWatch
    cls = method_class('docking_node.py', 'DockingNode',
                       {'math': _math, 'ActionWatch': ActionWatch})
    node = cls.__new__(cls)
    params = dict(HOLD_PARAMS)
    params.update(overrides)
    node._p = lambda name: params[name]
    node.get_logger = Node().get_logger
    return node


def test_jogging_publishes_an_arc_only_while_the_hold_is_engaged():
    """publish_arc 是航向保持接通时的专属通道; 未接通 (常态, 占整程 95%+)
    必须一次都不碰它 —— 底盘侧把 arc 当"走+转都生效"处理, 未接通时混进
    角速度就是在没有闭环依据的地方发转向。接通时发出去的第一个实参必须
    还是那 0.08m/s 前进速度, 不是 0。"""
    node = hold_node()
    calls = []
    node._adapter = NS(publish_jog=lambda v: calls.append(('jog', v)),
                       publish_arc=lambda v, w: calls.append(('arc', v, w)),
                       publish_turn=lambda w: calls.append(('turn', w)),
                       publish_stop=lambda: calls.append(('stop',)))
    node._executor = NS(action_kind='jogging', linear_cmd=.08,
                        angular_cmd=0., lateral_cmd=0.)
    for _ in range(5):
        node._publish_action_cmd()
    assert calls == [('jog', .08)]*5, '未接通却走了 arc —— 会混进无依据的转向'
    node._executor.angular_cmd = -.12
    node._publish_action_cmd()
    assert calls[-1] == ('arc', .08, -.12), '接通时前进速度必须原样带上'
    # turning 分支方向相反: "本该只转、却混进了走"仍然禁止
    node._executor = NS(action_kind='turning', linear_cmd=.08,
                        angular_cmd=.3, lateral_cmd=0.)
    node._publish_action_cmd()
    assert calls[-1] == ('turn', .3), 'turning 仍须纯原地转'


def launched(plan_step, **overrides):
    """跑真实的 _launch_step, 返回传给 start_jog 的 hold 实参。"""
    node = hold_node(**{'stopgo.jog_linear_rate': .08, 'stopgo.jog_odom_scale': 1.,
                        'stopgo.jog_backward_odom_scale': 1.,
                        'stopgo.lateral_rate': .12, 'stopgo.lateral_odom_scale': 1.,
                        **overrides})
    seen = {}
    node._dual = NS(p=lambda k: 0.5,
                    action_started=lambda plan, now: None)
    node._odom_x = node._odom_y = node._odom_yaw = 0.
    node._odom_stamp_ns = int(10e9)
    node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=int(10e9)))
    node._adapter = NS(publish_stop=lambda: None, publish_jog=lambda v: None,
                       publish_arc=lambda v, w: None)
    node._sm = NS(abort_motion=lambda m, code='': None)
    node._dual_prealign_active = False
    node._dual_watch = None
    node._executor = NS(
        is_active=False,
        start_jog=lambda d, r, odom_scale=1., hold=None: (
            seen.update(hold=hold, distance=d), setattr(node._executor, 'is_active', True)),
        set_odom_ref=lambda *a, **kw: None, action_kind='jogging',
        _action_target=.45, angular_cmd=0., linear_cmd=.08, lateral_cmd=0.)
    node._launch_step(plan_step)
    return seen.get('hold', 'not-called')


def test_heading_hold_arms_only_for_the_continuous_zone_run():
    """保持渗漏到别的行程就是在没有闭环依据的地方发角速度。区内一次走完那一步
    之外, 每一种形态都必须拿到 None —— 那正好等于航向保持上线前的行为。"""
    from tagdocking.action_executor import HeadingHold as HH
    zone = RealActionPlan(kind='forward', jog_distance=.45, continuous=True)
    assert isinstance(launched(zone), HH), '区内连续直行没拿到保持'
    # 回退修剪: continuous=False 且是倒走, 两道门各自都该挡住
    assert launched(RealActionPlan(kind='forward', jog_distance=-.03)) is None
    assert launched(RealActionPlan(kind='forward', jog_distance=-.03,
                               continuous=True)) is None, '倒走必须被 jog_distance>0 挡住'
    # 区外 forward_step 逐步走: 那里墙码 bearing 微调活着, 每停都在纠方向
    assert launched(RealActionPlan(kind='forward', jog_distance=.10)) is None
    # 现场一键回滚
    assert launched(zone, **{'stopgo.heading_hold_enable': False}) is None


def test_bad_hysteresis_config_disables_the_hold_instead_of_killing_the_dock():
    """release >= engage 是唯一会让状态机失去意义的配法 (接通即断开)。把锦上
    添花的微调变成整场本可成功的停泊的中止是错的交易 —— warn 后退回纯直行。"""
    zone = RealActionPlan(kind='forward', jog_distance=.45, continuous=True)
    for release in (2.0, 3.0):
        assert launched(zone, **{'stopgo.heading_hold_release_deg': release}) is None


def test_a_budget_below_one_minimum_engagement_is_refused_not_silently_dead():
    """budget < rate×min_engage 是最坏的一种配错, 必须显式关闭而不是假装开着。

    接通门里那条 `used + rate*min_engage < budget` 在 used=0 时就不成立 ——
    一次都接不通, used 恒 0, jog_hold_spent 也永不置上, 于是完成日志打的
    "航向保持已用=0.00deg" 与"漂移没到门槛、本就不需要纠"逐字相同。写这个
    功能时正是先在单测里撞到它 (原用例配 budget=1° 结果继电从未接通), 当时
    只改了测试值、没修代码, 坑留到现在。

    第三个用例是守卫自己的陷阱: 配置 rate 0.02 会被 start_jog 抬到
    min_angular_rate(0.12) —— 拿配置原值算门槛是 0.003deg, 5deg 的预算看着
    绰绰有余, 实际门槛是 1.03deg。守卫必须用抬底后的 rate, 否则它本身就漏。"""
    zone = RealActionPlan(kind='forward', jog_distance=.45, continuous=True)
    # rate .12 × min_engage .15 = .018rad = 1.03deg
    assert launched(zone, **{'stopgo.heading_hold_budget_deg': 1.0}) is None
    assert launched(zone, **{'stopgo.heading_hold_budget_deg': 1.03}) is None, '相等也不行'
    assert launched(zone, **{'stopgo.heading_hold_rate': .02,
                             'stopgo.heading_hold_budget_deg': .5}) is None, \
        '守卫必须按抬底后的 rate 算, 否则自己漏'
    hold = launched(zone, **{'stopgo.heading_hold_budget_deg': 4.0})
    assert hold is not None and hold.budget == pytest.approx(math.radians(4.0))


def test_declared_defaults_match_the_shipped_yaml():
    """七个参数在 declare_parameter 和 yaml 里各写一遍, 漂了就会出现"改了 yaml
    没生效"或"单测按 A 跑、现场按 B 跑"。本仓库已有该病史 (forward_step)。"""
    import ast
    import yaml as _yaml
    src = ast.parse((ROOT / 'tagdocking' / 'docking_node.py').read_text())
    declared = {}
    for n in ast.walk(src):
        if (isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
                and n.func.attr == 'declare_parameter' and len(n.args) == 2
                and isinstance(n.args[0], ast.Constant)
                and 'heading_hold' in str(n.args[0].value)):
            declared[n.args[0].value] = ast.literal_eval(n.args[1])
    assert len(declared) == 7, f'declare 了 {len(declared)} 个, 应为 7'
    shipped = _yaml.safe_load((ROOT / 'config' / 'docking.yaml').read_text())
    shipped = next(iter(shipped.values()))['ros__parameters']
    for name, default in declared.items():
        assert shipped[name] == default, f'{name}: yaml={shipped[name]} declare={default}'


# ── 墙码丢失族: 中止串必须自己答得出"为什么失败" ────────────────────
# 现场连续三次停泊失败是同一个物理现象(检测器某帧不含墙码), 操作员却看到三条
# 互不相干的英文串, 其中一条还在说谎。以下用例钉住"可读"本身, 全部只断言文案
# 与归因, 不含任何行为断言 —— 行为一字未改。

def _locked_wall_loss(frozen=True, active=True, ids=(51,)):
    """把节点开到 locked 段再丢墙码, 返回 (节点, abort 理由)。"""
    n = intake.__wrapped__()
    n._dual.stage = 'locked'
    n._frozen, n._executor.is_active = frozen, active
    stamp = n.now
    n.detect()
    n.supply(stamp)
    n._retry_dual_detections(n.now)         # 先拿到一帧可用墙码当证据
    n.now += 200_000_000
    n.detect(ids=ids)
    return n, n.events[-1]


def test_wall_loss_abort_says_wall_tag_lost_in_plain_chinese():
    """开头四个字就得是"墙码丢失", 且不再出现操作员看不懂的英文 jargon。"""
    _, reason = _locked_wall_loss()
    assert reason.startswith('墙码丢失')
    for jargon in ('wall detection lost', 'wall stream stale',
                   'wall stream lost', 'no search/reverse'):
        assert jargon not in reason


def test_wall_loss_reason_reports_what_the_detector_actually_saw():
    """{51} 与空集是两个完全不同的根因, 必须给出不同的排查指向。

    桩码还在 → 相机与检测流没断, 是墙码这一个没解出来;
    整帧零检测 → 查检测链路/图像流, 与墙码无关。旧串对两者一字不差。
    """
    _, pile_only = _locked_wall_loss(ids=(51,))
    _, nothing = _locked_wall_loss(ids=())
    assert '检测器可见={51}' in pile_only and '查墙码本身' in pile_only
    assert '整帧一个码都没有' in nothing and '查检测链路/图像流' in nothing
    assert pile_only != nothing


def test_wall_loss_reason_distinguishes_mid_move_from_after_stop():
    """现场失败 A(盲走 1.00s 时被杀) 与 C(机动结束后 1.1ms) 旧串逐字相同,
    我只能靠手工对时间戳把它们分开。三种相位必须各自可读。

    这同时钉住"取证早于 _executor.cancel()": cancel() 把 is_active 翻成
    False, 取晚一行, 三种相位就全都读成"已停稳"。
    """
    _, moving = _locked_wall_loss(frozen=True, active=True)
    _, unsettled = _locked_wall_loss(frozen=True, active=False)
    _, stopped = _locked_wall_loss(frozen=False, active=False)
    assert '丢失时=盲走执行中' in moving
    assert '丢失时=冻结未收口' in unsettled
    assert '丢失时=已停稳' in stopped
    assert len({moving, unsettled, stopped}) == 3


def test_wall_loss_evidence_survives_the_invalidation_that_precedes_the_abort():
    """最关键的一条: 抹除发生在中止之前。

    _invalidate_dual_pose / observe(..., None) 都会 reset_filter(), 把
    wall/stamp/帧数清零。取证若退回原位, 串里就是"从未取得"与空余量 ——
    看着有证据其实全是空值, 最坏的失效形态。
    """
    n, reason = _locked_wall_loss(frozen=False, active=False)
    assert n._dual.wall is None and n._dual.stamp == 0   # 证据确实已被抹掉
    assert '上次可用墙码深度=1.500m' in reason            # 但串里留住了
    assert 'wall_frames=1' in reason
    assert 'required=' in reason and '<无最后位姿>' not in reason


def test_stale_locked_watchdog_reason_names_fresh_sec():
    n = intake.__wrapped__()
    n._dual.stage = 'locked'
    stamp = n.now
    n.detect()
    n.supply(stamp)
    n._retry_dual_detections(n.now)
    n.now += 900_000_000
    reason = n._wall_stale_reason(n.now, n._motion_phase())
    assert reason.startswith('墙码丢失')
    assert 'dual.fresh_sec' in reason and 'ms' in reason


def _outer_budget_reason(**extra):
    sm = DockingStateMachine(clock_node())
    sm._tag_lost_count = 51
    params = {'tag': {'tag_loss_timeout_sec': 2.5}}
    params.update(extra)
    return sm._wall_loss_reason(params)


def test_outer_wall_loss_blames_the_settle_window_when_it_is_to_blame():
    """旧串 'dual wall stream lost' 在说谎: 现场失败 B 里墙码一直看得见,
    2.5s 预算有 1.6s 花在 dual.settle_sec 停稳窗的按设计拒收上。操作员照串
    去查相机, 方向完全错。新串必须把这笔账算出来并点名 dual.settle_sec。
    """
    now = int(100e9)
    reason = _outer_budget_reason(
        dual_now_ns=now, dual_settle_until_ns=now - int(0.93e9),
        dual_settle_sec=1.5, dual_wall_loss='检测器最近给出墙码=60ms 前')
    assert reason.startswith('墙码丢失')
    assert 'dual.settle_sec' in reason and '真正的丢码宽限只有 0.93s' in reason
    assert '不退回 SEARCH_TAG' in reason      # 双码路径不搜索不重试, 文档曾写错
    # 停稳窗早已过期时不得再拿它当借口
    clean = _outer_budget_reason(dual_now_ns=now,
                                 dual_settle_until_ns=now - int(9e9),
                                 dual_settle_sec=1.5)
    assert '停稳静止窗' not in clean


def test_outer_wall_loss_resolves_lazy_evidence():
    """取证按闭包下发(20Hz 不白算), 串里必须是取证结果而不是 <function ...>。"""
    reason = _outer_budget_reason(dual_wall_loss=lambda: '检测器本轮从未给出墙码')
    assert '检测器本轮从未给出墙码' in reason and 'function' not in reason


def test_outer_wall_loss_stays_readable_without_any_node_evidence():
    """假 params(单测/无取证接口)下仍须给出可读中文串: 不 KeyError、不空串。"""
    reason = _outer_budget_reason()
    assert reason.startswith('墙码丢失') and '证据不可得' in reason
    bare = DockingStateMachine(clock_node())._wall_loss_reason({})
    assert bare.startswith('墙码丢失')


def test_abort_reason_is_kept_for_the_action_result():
    """理由过去只活在一行日志里, action 客户端只拿到 'motion_failed'。"""
    sm = DockingStateMachine(clock_node())
    assert sm.failure_reason == ''
    sm.abort_motion('墙码丢失 — 测试')
    assert sm.state == DockingState.MOTION_FAILED
    assert sm.failure_reason == '墙码丢失 — 测试'
    sm.reset()
    assert sm.failure_reason == ''


# ── 失败留档: 每条通往错误终态的路都要有 reason + code ───────────────
# 起因是要把停泊接口交给上层应用。改之前"状态有、原因基本没有":
# _abort_reason 只在 abort_motion() 里赋值 —— TIMEOUT 全族 / CANCELLED /
# 运动卡死 / 倒车超时 / fail() 路径共 8 处只留一行日志, 上层拿到的是个
# 光秃秃的 'motion_failed'。以下用例钉住"每条路都记账"与"码是 6 族之一"。


def logging_node():
    """带日志捕获的假节点 —— test_dual_docking.Node 的 logger 吞掉一切。"""
    node = clock_node()
    node.logged = {'info': [], 'warn': [], 'error': []}
    node.get_logger = lambda: NS(
        info=lambda m, **kw: node.logged['info'].append(m),
        warn=lambda m, **kw: node.logged['warn'].append(m),
        error=lambda m, **kw: node.logged['error'].append(m))
    return node


def _sm_at(state, **attrs):
    """把状态机摆到 state 上, 时间起点 = 节点时钟 (20s)。

    必须先 start() 再直接写 _state: state_elapsed_ns 把 _state_start_ns==0
    当"未开始"的哨兵返回 0, 光设 _state 的话所有超时判据永不触发 (这一脚
    在写这些用例时就真踩过一次)。
    """
    sm = DockingStateMachine(logging_node())
    sm.start()
    sm._state = state
    for k, v in attrs.items():
        setattr(sm, k, v)
    return sm


def _tick(sm, now_s, params, visible=False, pose=None, stalled=False):
    sm.evaluate(pose, visible, stalled, int(now_s * 1e9), params)
    return sm


def _overall_timeout():
    return _tick(_sm_at(DockingState.SEARCH_TAG), 200, {'timeout_sec': 120.0})


def _motion_stalled():
    # APPROACH 由双码全权驱动 (evaluate 无条件早退), motion_stalled 判据只
    # 够得着其余活动态 —— 用 UNDOCKING 钉这条路径。
    return _tick(_sm_at(DockingState.UNDOCKING), 21, {}, stalled=True)


def _search_timeout():
    return _tick(_sm_at(DockingState.SEARCH_TAG), 100,
                 {'timeout_sec': 9e9, 'search': {'timeout_sec': 60.0}})


def _undock_timeout(node_code):
    sm = DockingStateMachine(logging_node())
    sm._node._undock_note = '测试: 卡在这一步'
    sm._node._undock_code = node_code
    sm.start_undock()
    return _tick(sm, 100, {'undock': {'timeout_sec': 30.0}})


def _dual_wall_loss():
    sm = _sm_at(DockingState.APPROACH, _tag_lost_count=200)
    return _tick(sm, 21, {'timeout_sec': 9e9,
                          'tag': {'tag_loss_timeout_sec': 2.5}})


def _cancelled():
    sm = _sm_at(DockingState.APPROACH)
    sm.cancel()
    return sm


# 每一行 = 一条通往错误终态的路。加新的 abort 点就往这里加一行 ——
# 漏加也不会静默: _transition_to 的守卫会把它报成 unspecified, 而
# test_unaccounted_failure_path_warns_and_marks_itself 咬着那个码。
# (v1.0 删掉了单码重试链: 倒车超时/末端精调超时×2/重试耗尽/接近超时
# 五条路随之消失, 双码失败全部经 abort_motion 直落终态。)
FAILURE_PATHS = [
    ('整体超时', _overall_timeout, CODE_TIMEOUT),
    ('运动卡死', _motion_stalled, CODE_MOTION_STALLED),
    ('搜索超时', _search_timeout, CODE_TAG_NOT_FOUND),
    ('泊出超时-门控', lambda: _undock_timeout(CODE_MOTION_GATED),
     CODE_MOTION_GATED),
    ('泊出超时-节点没给码', lambda: _undock_timeout(''), CODE_MOTION_STALLED),
    ('双码外层丢码', _dual_wall_loss, CODE_TAG_NOT_FOUND),
    ('用户取消', _cancelled, CODE_CANCELLED),
]


@pytest.mark.parametrize('name,drive,expect', FAILURE_PATHS,
                         ids=[p[0] for p in FAILURE_PATHS])
def test_every_error_terminal_carries_a_reason_and_a_code(name, drive, expect):
    """本次改动的主断言: 没有哪条失败路径能只给上层一个状态名。"""
    sm = drive()
    assert sm.is_error, f'{name}: 没走到错误终态, 状态={sm.state_name}'
    assert sm.failure_code == expect, f'{name}: 码={sm.failure_code!r}'
    assert sm.failure_code in FAILURE_CODES, f'{name}: 码不在 6 族里'
    assert sm.failure_reason, f'{name}: 原因是空串'
    # 守卫兜底码不是对外承诺的一族 —— 出现它就说明这条路漏了记账。
    assert sm.failure_code != CODE_UNSPECIFIED, f'{name}: 漏记账'
    assert '未记账' not in ''.join(sm._node.logged['warn']), f'{name}: 触发了守卫'


def test_unaccounted_failure_path_warns_and_marks_itself():
    """漏记账的失败路径必须自己喊出来, 而不是静默给上层一个空原因。"""
    sm = DockingStateMachine(logging_node())
    sm.start()
    sm._transition_to(DockingState.MOTION_FAILED)   # 绕开所有记账入口
    assert sm.failure_code == CODE_UNSPECIFIED
    assert sm.failure_reason         # 兜底串, 不许空
    warns = ''.join(sm._node.logged['warn'])
    assert '未记账的失败路径' in warns and 'MOTION_FAILED' in warns


def test_failure_ledger_is_cleared_on_each_new_run_and_seq_advances():
    """改之前只有 reset() 清账, 而包里没有任何节点调用 reset() ——
    上一次的失败原因会一路活到下一次, 被当成这一次的原因报给上层。"""
    sm = DockingStateMachine(logging_node())
    first = sm.run_seq
    sm.start()
    sm.abort_motion('第一轮: 墙码丢失', CODE_TAG_NOT_FOUND)
    assert (sm.failure_code, sm.run_seq) == (CODE_TAG_NOT_FOUND, first + 1)

    sm.start()                       # 新一轮停泊
    assert sm.failure_reason == '' and sm.failure_code == ''
    assert sm.run_seq == first + 2

    sm.abort_motion('第二轮', CODE_MOTION_GATED)
    sm.start_undock()                # 泊出同样算新一轮
    assert sm.failure_reason == '' and sm.failure_code == ''
    assert sm.run_seq == first + 3


def test_publish_outcome_emits_code_and_reason_once_per_terminal():
    """~/outcome 是给上层的出口: 终态那一沿发一次, 成功也发, 带轮次号。"""
    published = []
    env = {'json': __import__('json'), 'time': __import__('time'),
           'math': math}
    cls = method_class('docking_node.py', 'DockingNode', env)
    # method_class 自己把 String 设成 object 占位 (多数用例只需要它能被
    # 引用), 这里真要构造消息, 所以在它之后再覆盖回来。
    env['String'] = lambda: NS(data='')

    sm = DockingStateMachine(logging_node())
    # 假节点的时钟拨到 66s: 状态机侧 logging_node 的时钟固定在 20s (start
    # 记 t0), 两者相减就是可断言的整轮耗时 46.0s。
    logs = {'info': [], 'error': []}
    node = NS(_sm=sm, _prev_state=DockingState.IDLE,
              _outcome_pub=NS(publish=lambda m: published.append(m.data)),
              get_clock=lambda: NS(now=lambda: NS(nanoseconds=int(66e9))),
              get_logger=lambda: NS(info=lambda m, **kw: logs['info'].append(m),
                                    error=lambda m, **kw: logs['error'].append(m)))

    sm.start()
    sm.abort_motion('测试: 视觉修正振荡', CODE_VISION_NO_PROGRESS)
    cls._publish_outcome(node, sm.state)
    fail = __import__('json').loads(published[-1])
    assert fail['ok'] is False
    assert fail['state'] == 'motion_failed'
    assert fail['code'] == CODE_VISION_NO_PROGRESS
    assert fail['reason'] == '测试: 视觉修正振荡'
    assert fail['op'] == 'dock' and fail['seq'] == sm.run_seq
    assert isinstance(fail['stamp'], float)
    assert fail['elapsed_sec'] == 46.0, '整轮耗时 (start→终态) 要进载荷'
    assert '耗时=46.0s' in logs['error'][-1], '失败日志要带耗时'

    # 成功也发, 且 code/reason 必须是空的 (不能带上一轮的残留)。
    sm.start()
    sm._state = DockingState.APPROACH
    sm.finish_dual()
    cls._publish_outcome(node, sm.state)
    ok = __import__('json').loads(published[-1])
    assert (ok['ok'], ok['state'], ok['code'], ok['reason']) == (
        True, 'docked', '', '')
    assert ok['seq'] == fail['seq'] + 1
    assert isinstance(ok['elapsed_sec'], float)
    assert any('耗时' in m for m in logs['info']), \
        '成功原来没有结果日志, 现场问"这次停了多久"只能翻系统日志'

    # 泊出的结果要能和停泊分开 —— 同一个 motion_failed 上层处置不同。
    sm.start_undock()
    sm.finish_undock()
    cls._publish_outcome(node, sm.state)
    assert __import__('json').loads(published[-1])['op'] == 'undock'


def test_action_result_carries_the_code_after_the_state_name():
    """action 的调用方也要能按码分支, 而状态名前缀不能被破坏 (有 startswith
    消费者)。"""
    src = (ROOT / 'tagdocking' / 'docking_node.py').read_text()
    assert "f'{state}: {tail}' if tail else state" in src
    assert "f'[{code}]' if code else ''" in src


def test_outcome_is_forwarded_and_exposed_verbatim():
    """web_console / stack_supervisor 的转发只做源码级断言。

    这两个模块目前零测试, 且模块作用域就 import rclpy/cv2/fastapi/uvicorn,
    纯离线环境里根本 import 不进来 —— 为 3 行不透明转发去建该包的首个真
    单测, 成本远大于收益。**这不等于它们被真覆盖了**, 上机清单里有对应的
    人工验证项 (收栈后从 /docking_supervisor/status 读 last_outcome)。
    照本文件既有的 AST/源码断言先例。
    """
    sup = (ROOT / 'tagdocking' / 'stack_supervisor.py').read_text()
    assert "OUTCOME_TOPIC = '/docking_node/outcome'" in sup
    assert "'last_outcome': self._last_outcome" in sup
    # 转发必须是不透明的: supervisor 一行停泊逻辑都不碰 (模块头硬约束),
    # 所以它不许 import 状态机/码常量, 也不许按码分支。
    # (按"整词"断言而不是子串: 它的注释里合法地提到 state_machine.py 的行号。)
    assert 'from .state_machine' not in sup and 'import state_machine' not in sup
    assert 'CODE_' not in sup.replace('# ', '')
    assert "_last_outcome['code']" not in sup and '.get(\'code\')' not in sup

    web = (ROOT / 'tagdocking' / 'web_console.py').read_text()
    assert "OUTCOME_TOPIC = '/docking_node/outcome'" in web
    assert "'outcome': self._outcome" in web
    assert 'DurabilityPolicy.TRANSIENT_LOCAL' in web   # 必须匹配锁存 QoS

    page = (ROOT / 'tagdocking' / 'static' / 'index.html').read_text()
    # 显示门槛按 state 对齐 (不按年龄 —— 锁存边沿值的年龄一直涨),
    # 失败沿按 seq 去重 (render() 是 5Hz)。
    assert 'outcome.state === name' in page
    assert 'lastOutcomeSeq' in page
