"""No ROS nodes, launch processes or robot service calls in these tests."""
import math
from types import SimpleNamespace

import pytest

from tagdocking.dual_docking import DEFAULTS, DualTagDocking
from tagdocking.dual_camera import CameraModel


class Node:
    def __init__(self, **overrides):
        self.params = {'dual.'+k: v for k, v in DEFAULTS.items()}
        self.params.update({'dual.enable': True, 'dual.pile_tag_id': 51,
                            'tag.id': 0, 'tag.family': '36h11',
                            'base.type': 'omni', 'dual.dock_distance': .5,
                            'dual.crouch_enable': False,
                            'dual.straight_start_distance': 1.70,
                            'dual.align_tolerance_deg': 3., 'dual.align_hold_sec': .5,
                            'posture.static_settle_sec': 1.2,
                            'dock_target.distance': .55, 'dual.wall_tag_size': .15,
                            'dual.pile_tag_size': .05})
        self.params.update(overrides)

    def _p(self, name):
        return self.params[name]

    def get_logger(self):
        return SimpleNamespace(info=lambda *a, **kw: None,
                               warn=lambda *a, **kw: None,
                               error=lambda *a, **kw: None)


def controller():
    c = DualTagDocking(Node())
    c.set_extrinsics((.2, .03, .4), (-.5, .5, -.5, .5))
    c.camera = CameraModel(1600, 1296, 400., 400., 800., 648.)
    return c


def frames(c, depth=1.8, x=0., pile_x=None, start=10., missing=False, count=4):
    for i in range(count):
        stamp = int((start+i*.2)*1e9)
        pile = None if missing else (x if pile_x is None else pile_x, .3, depth-.9)
        c.observe(stamp, stamp, (x, -.2, depth), pile, pile_missing=missing)
    return stamp


def plan(c, now):
    return c.plan_dual(None, 'omni', None, now)


def test_observation_then_forward_and_recorrect():
    c = controller()
    n = frames(c)
    step = plan(c, n)[0]
    assert c.stage == 'approach' and step.jog_distance == .05
    c.action_started(step, n)
    assert not c.progress
    c.action_completed()
    assert c.progress
    c.stopped(n)
    assert not c.observe(n+int(1.49e9), n+int(1.49e9), (0, 0, 1.75), (0, 0, .85))
    n = frames(c, depth=1.75, x=.07, start=13.)
    step = plan(c, n)[0]
    assert step.turn_angle or step.lateral_distance
    assert c.pending_metrics["after"][-1] < c.pending_metrics["before"][-1]
    assert c.stage == 'approach'


def test_duplicate_stale_and_missing_reset_stability():
    c = controller()
    n = int(10e9)
    for _ in range(10):
        c.observe(n, n, (0, 0, 1.5), (0, 0, .6))
    assert c.frames == 1 and plan(c, n) is None
    assert not c.observe(n+1, n+int(2e9), (0, 0, 1.5), (0, 0, .6))
    assert c.frames == 0
    n = frames(c, start=13)
    c.observe(n+int(.2e9), n+int(.2e9), (0, 0, 1.5), None, True)
    assert c.frames == 0


def test_wall_only_bounded_reverse():
    # 预算取小值: 本用例测的是"墙码单码后退有界、耗尽即判失败"这个机制, 不是
    # 生产配置里那个数。写死 6 次在预算从 0.30m/8 次调到 0.80m/16 次之后就静默
    # 失效了 (而 16 次 × 4s 还会先撞上 acquire_timeout_sec, 失败串也就不再是
    # 预算)。
    c = DualTagDocking(Node(**{'dual.reverse_count': 4., 'dual.reverse_limit': .20}))
    c.set_extrinsics((.2, .03, .4), (-.5, .5, -.5, .5))
    c.camera = CameraModel(1600, 1296, 400., 400., 800., 648.)
    for i in range(4):
        n = frames(c, start=10+i*4, missing=True, count=10)
        step = plan(c, n)[0]
        assert step.jog_distance == -.05
        c.action_started(step, n)
        c.stopped(n)
    n = frames(c, start=26, missing=True, count=10)
    assert plan(c, n) is None
    assert 'budget' in c.failure


def test_each_tag_not_average_and_yaw_cap():
    c = controller()
    n = frames(c, x=.12, pile_x=-.12)
    step = plan(c, n)[0]
    assert step.turn_angle or step.lateral_distance
    # z=1.8 在 near(1.70) 之上 → 远场粗档 cap; 断言对 cap 而非写死 3°
    assert abs(step.turn_angle) <= c.yaw_cap
    assert not c.progress


def test_near_unqualified_and_far_loss_never_latch():
    c = controller()
    c.stage = 'approach'
    n = frames(c, depth=.95, missing=True, count=10)
    assert plan(c, n) is None and c.stage != 'locked'
    c.progress = True
    c.qualified_ns = n
    # "远" = 直行包络 (straight_envelope = obs+tol = 1.90) 之外, 不再是 near
    # (1.70) 之外: 1.70~1.90 是直行提交区, 桩码在那里合法离场并须闭锁。
    n = frames(c, depth=2.2, missing=True, start=12, count=10)
    assert plan(c, n) is None and c.stage != 'locked'


def test_latch_reappearance_finish_and_shrinking_jog():
    c = controller()
    c.stage = 'approach'
    c.progress = True
    c.qualified_ns = int(10e9)
    n = frames(c, depth=.95, missing=True, count=10)
    step = plan(c, n)[0]
    assert c.stage == 'locked' and step.jog_distance > 0
    c.stopped(n)
    # x=0: locked 下若墙码 bearing 超容差, 新的 bearing 微调会取代 _advance,
    # 本用例测的是桩码重现为 locked 所忽略 + 终点收缩步, 需保持正对。
    n = frames(c, depth=.531, x=0., pile_x=-.2, start=14)
    step = plan(c, n)[0]
    assert step.turn_angle == step.lateral_distance == 0
    assert step.jog_distance == pytest.approx(.031)
    c.stopped(n)
    n = frames(c, depth=.5, missing=True, start=17)
    assert plan(c, n)[0].kind == 'done' and c.complete


def test_locked_forward_survives_pending_revalidation():
    # locked 下 observe() 强制 pile=None, "仍然对准"对无桩码计划不可检验 --
    # 修复前每一步 locked 前进都被 pending_valid 否掉再原样重规划, 一步不动的活锁。
    c = controller()
    c.stage = 'approach'
    c.progress = True
    c.qualified_ns = int(10e9)
    n = frames(c, depth=.95, missing=True, count=10)
    step = plan(c, n)[0]
    assert c.stage == 'locked' and c.pile is None
    assert step.jog_distance > 0 and c.pending_aligned
    assert c.pending_valid(n)


def test_overshoot_and_initial_target_not_success():
    c = controller()
    n = frames(c, depth=.47, missing=True)
    assert plan(c, n) is None and c.failure and not c.complete
    c = controller()
    n = frames(c, depth=.5, missing=True)
    assert plan(c, n) is None and not c.complete


def test_calibration_missing_bad_mount_and_pending_expiry():
    c = DualTagDocking(Node())
    n = frames(c)
    assert plan(c, n) is None and c.failure
    with pytest.raises(ValueError):
        c.set_extrinsics((0, 0, 0), (0, 0, 0, 1))
    c = controller()
    n = frames(c)
    plan(c, n)
    assert c.pending_valid(n)
    assert not c.pending_valid(n+int(1e9))
    assert not c.progress


def test_height_difference_with_pitch():
    c = controller()
    pitch = .2
    # R_base_y(pitch) * canonical optical rotation; convert points by R^T.
    cp, sp = math.cos(pitch), math.sin(pitch)
    c.r = ((0., sp, cp), (-1., 0., 0.), (0., -cp, sp))
    def optical(x, z):
        return (0., sp*x-cp*z, cp*x+sp*z)
    w, p = optical(1.9, .4), optical(.7, -.2)
    for i in range(4):
        n = int((10+i*.2)*1e9)
        c.observe(n, n, w, p)
    step = plan(c, n)[0]
    assert step.turn_angle == 0 and step.lateral_distance == 0
    assert step.jog_distance > 0


def test_pending_forward_without_completion_cannot_lock():
    c = controller()
    n = frames(c)
    step = plan(c, n)[0]
    c.action_started(step, n)
    assert not c.progress
    c.stopped(n)
    n = frames(c, depth=.95, missing=True, start=13, count=10)
    assert plan(c, n) is None and c.stage == 'approach'


def test_new_misaligned_pair_revokes_qualification():
    c = controller()
    n = frames(c)
    step = plan(c, n)[0]
    c.action_started(step, n)
    c.action_completed()
    assert c.qualified_ns
    c.stopped(n)
    frames(c, depth=1.45, x=.1, start=13)
    assert c.qualified_ns == 0


def test_single_tag_target_unchanged():
    c = DualTagDocking(Node(**{'dual.enable': False}))
    assert c.effective_dock_distance() == .55
