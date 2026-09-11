#!/usr/bin/env python3
"""双码姿态切换 + 状态机全分支 mock 验证 (纯离线, 不依赖 ROS 运行时数据流)。

场景 (对照用户 5 步流程):
  [T1] acquire 远距桩码不可见 → 按墙码距离分流: 匍匐前进逼近 (非倒车死路)
  [T2] acquire 太近桩码出视野 → 后退找回 (reverse 预算 0.6m/12 次)
  [T3] 双码对准 + 桩码 z=0.30m → locked + request_stand (30cm 主动锁定)
  [T4] 对准 0.8m → observe→approach 全程匍匐前进 (不锁) → 桩码 0.30 锁定
  [T5] locked + 站立后桩码必然丢失 → 墙码纯直行累积航向资格 → 0.50m complete
       (验证 locked forward aligned=True 修复: 否则终点误判 no qualified approach)
  [T6] locked 墙码 z 跳近豁免 overshoot (姿态切换跳变); 非 locked 命中 fail
  [T7] DualPosture: 服务缺失宽限→fail / 趴下成功 / 桥锁 cmd_vel→fail /
       服务拒绝重试耗尽→fail / 站立 motion_enabled 确认 / 站立超时重试耗尽→fail
  [T8] ActionWatch: 正常完成 / 反方向 veto / 无里程计响应

世界模型: 狗位姿 (x, y, yaw) 于 odom; 墙码 id=0 (15cm) 贴桩后墙, 桩码 id=51
(5cm) 贴桩底座正面, 桩-墙距 0.70m; 两码同高 0.30m。相机正装 base (0.15,0,0.30),
光学轴 = 狗朝向。桩码 5cm 可见包络 (光学 z): [0.22, 2.2] m。
运行: source /opt/ros/humble/setup.bash && python3 test_dual_posture_mock.py
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import yaml

from tagdocking.dual_docking import DEFAULTS, DualTagDocking
from tagdocking.dual_posture import DualPosture
from tagdocking.dual_feedback import ActionWatch
from tagdocking.geometry_planner import ActionPlan

ROOT = os.path.dirname(os.path.abspath(__file__))
STEP = 0.1                      # s — observation tick
JOG_SPEED = 0.08                # m/s — mirrors stopgo.jog_linear_rate
WALL_BASELINE = 0.70            # m — pile-to-wall distance
CAM = (0.15, 0.0, 0.30)         # camera in base frame (正装, 光学轴 = 狗朝向)
PILE_VISIBLE = (0.22, 2.2)      # m — 5cm pile tag optical-z visibility envelope

# 光学→base: 光学 z_f→base x, x_right→base -y, y_down→base -z (完整刚体变换,
# 绕过 set_extrinsics 校验直接注入 —— 与节点 TF 标定等价的正装标定)。
R_OPT = ((0.0, 0.0, 1.0), (-1.0, 0.0, 0.0), (0.0, -1.0, 0.0))
T_OPT = CAM

RESULTS = []


def check(name, cond, detail=''):
    RESULTS.append((name, bool(cond)))
    print(('PASS' if cond else 'FAIL') + f' [{name}] {detail}')


# ── Fake ROS surface ────────────────────────────────────────────────

class FakeLogger:
    def __init__(self):
        self.lines = []
    def info(self, msg, **kw):
        self.lines.append(('INFO', str(msg)))
    def warn(self, msg, **kw):
        self.lines.append(('WARN', str(msg)))
    def error(self, msg, **kw):
        self.lines.append(('ERROR', str(msg)))


class TriggerResult:
    def __init__(self, success):
        self.success = success


class FakeFuture:
    def __init__(self, result=None):
        self._result = result
    def done(self):
        return True
    def result(self):
        return self._result


class FakeClient:
    def __init__(self, ready=True, result=True):
        self._ready = ready
        self._result = result
        self.calls = 0
    def service_is_ready(self):
        return self._ready
    def call_async(self, req):
        self.calls += 1
        return FakeFuture(TriggerResult(self._result))


class MotionMsg:
    def __init__(self, data):
        self.data = data


class FakeNode:
    def __init__(self, params, clients=None):
        self._params = params
        self.logger = FakeLogger()
        self._clients = clients or {}
        self.subs = []
    def _p(self, name):
        return self._params[name]
    def get_logger(self):
        return self.logger
    def create_client(self, srv, name):
        return self._clients[name]
    def create_subscription(self, msg, topic, cb, qos):
        self.subs.append((topic, cb))
        return cb


def load_params(profile=None):
    with open(os.path.join(ROOT, 'config', 'docking.yaml')) as f:
        doc = yaml.safe_load(f)
    params = dict(doc['docking_node']['ros__parameters'])
    params['dual.enable'] = True
    # yaml 缺省的 dual.* 回填 DEFAULTS (与节点 declare_parameter 语义一致)
    for k, v in DEFAULTS.items():
        params.setdefault('dual.' + k, v)
    params.update(profile or {})
    return params


# 匍匐 profile: 历史 T1-T8 场景按此调参 (obs 1.5 / near 1.0 / 30cm 主动锁定)。
# pile_lock_distance 两 profile 同为 0.30, 站立下该触发被 crouch 门关掉。
CROUCH_PROFILE = {'dual.crouch_enable': True,
                  'dual.observation_distance': 1.5,
                  'dual.straight_start_distance': 1.0}
# 站立 profile: 全程站立, locked 只能由 approach 桩码丢失闭锁进入,
# 直行阶段带墙码 bearing 微调 (straight_yaw_tol_deg)。
STANDING_PROFILE = {'dual.crouch_enable': False,
                    'dual.observation_distance': 1.8,
                    'dual.straight_start_distance': 1.70}


def set_motion(node, value):
    for topic, cb in node.subs:
        if topic.endswith('motion_enabled'):
            cb(MotionMsg(value))
            return
    raise AssertionError('motion_enabled subscription missing')


# ── World / mock camera ─────────────────────────────────────────────

def optical(pose, P):
    """世界点 P → 相机光学 (x_right, y_down, z_forward); 相机正装于 base CAM。

    约定必须与 predict() 一致, 否则闭环自己打自己: REP-103 yaw+ = 左转,
    正装相机 optical x = 右 —— 左转使正前目标向画面右侧移动
    (d(x_opt)/d(yaw) > 0)。符号写反时 (x_opt = -s·dx + c·dy) 纠偏转向会把
    bearing 翻倍而不是归零: T9 是第一个走转向的世界模型用例, 修前正是
    以 'visual correction no progress / oscillation' 暴露的。
    """
    px, py, yaw = pose
    cx = px + CAM[0] * math.cos(yaw)
    cy = py + CAM[0] * math.sin(yaw)
    dx, dy, dz = P[0] - cx, P[1] - cy, P[2] - CAM[2]
    c, s = math.cos(yaw), math.sin(yaw)
    return (s * dx - c * dy, -dz, c * dx + s * dy)


class FakeCamera:
    """bounds 恒定充裕 → visible() 恒 True (可见性由世界包络独立控制)。"""
    def bounds(self, pt, size):
        return (50.0, 50.0, 50.0, 50.0)


class World:
    def __init__(self, pile_x):
        self.pile = (pile_x, 0.0, 0.30)
        self.wall = (pile_x + WALL_BASELINE, 0.0, 0.30)

    def observe_tick(self, d, pose, t, pile_on=True):
        wall = optical(pose, self.wall)
        pile = optical(pose, self.pile)
        seen = pile_on and PILE_VISIBLE[0] <= pile[2] <= PILE_VISIBLE[1]
        return d.observe(int(t), int(t), wall, pile if seen else None,
                         pile_missing=not seen)


def apply_plan(plan, pose):
    px, py, yaw = pose
    if plan.turn_angle:
        return (px, py, yaw + plan.turn_angle)
    if plan.lateral_distance:
        s, c = math.sin(yaw), math.cos(yaw)
        return (px - s * plan.lateral_distance, py + c * plan.lateral_distance, yaw)
    c, s = math.cos(yaw), math.sin(yaw)
    return (px + c * plan.jog_distance, py + s * plan.jog_distance, yaw)


def make_dual(params, pile_x):
    node = FakeNode(params)
    d = DualTagDocking(node)
    d.r, d.t = R_OPT, T_OPT
    d.camera = FakeCamera()
    return d, node, World(pile_x)


def run(d, world, pose, t, until=None, max_ticks=1200, pile_on=True):
    """闭环走停: 观测 → plan_dual → 执行动作 (started/completed/stopped 节奏)。"""
    d.stopped(int(t))
    t += int(1.6e9)
    actions, complete = [], False
    for _ in range(max_ticks):
        world.observe_tick(d, pose, t, pile_on)
        seq = d.plan_dual(None, 'omni', None, int(t))
        if d.failure:
            break
        if seq and seq[0].kind == 'done':
            complete = True
            actions.append(seq[0])
            break
        if d.complete:
            complete = True
            break
        if seq:
            plan = seq[0]
            d.action_started(plan, int(t))
            amount = plan.turn_angle or plan.lateral_distance or plan.jog_distance
            t += int(max(0.5, abs(amount) / JOG_SPEED) * 1e9)
            pose = apply_plan(plan, pose)
            d.action_completed()
            d.stopped(int(t))
            t += int(1.6e9)
            actions.append(plan)
            if until and until(d):
                break
            continue
        t += int(STEP * 1e9)
        if until and until(d):
            break
    return pose, t, actions, complete


# ── T1: acquire 远距 → 前进逼近 ─────────────────────────────────────

def t1_acquire_forward(params):
    # pile 光学 z = 2.5 - x > 2.2 不可见; wall z = 3.2 - x
    d, node, world = make_dual(params, 2.65)
    pose, t, actions, _ = run(d, world, (0.0, 0.0, 0.0), 0,
                              until=lambda d: d.stage != 'acquire')
    fwd = [a for a in actions if a.jog_distance > 0]
    rev = [a for a in actions if a.jog_distance < 0]
    check('T1 acquire-太远走前进而非倒车',
          len(fwd) >= 5 and not rev and d.stage != 'acquire' and not d.failure,
          f'forward={len(fwd)} reverse={len(rev)} stage={d.stage} '
          f'failure={d.failure!r}')
    return d, world, pose, t, actions


# ── T2: acquire 太近 → 后退找回 ─────────────────────────────────────

def t2_acquire_reverse(params):
    # pile 光学 z = 0.18 < 0.22 出视野; wall z = 0.88
    d, node, world = make_dual(params, 0.33)
    pose, t, actions, _ = run(d, world, (0.0, 0.0, 0.0), 0,
                              until=lambda d: d.stage != 'acquire')
    rev = [a for a in actions if a.jog_distance < 0]
    check('T2 acquire-太近后退找回',
          len(rev) >= 1 and d.stage != 'acquire' and not d.failure,
          f'reverse={len(rev)} total={d.reverse_total:.2f}m '
          f'stage={d.stage} failure={d.failure!r}')


# ── T3: 对准 + 桩码 0.30 → 锁定 + request_stand ─────────────────────

def t3_lock_at_30cm(params):
    # 正对桩, 从站位 (wall z=1.5 = obs, pile z=0.80) approach 前进到
    # pile z=0.30 → locked (30cm 主动锁定, 匍匐 profile)。
    # 原断言 "actions==0" 已失效: 站位守卫 (obs 1.5, 守卫下沿 1.4) 会把
    # 从 wall z=1.00 起步的场景先拖回观察窗 —— 守卫是对的, 场景改从窗内起步。
    d, node, world = make_dual(params, 0.95)
    pose, t, actions, _ = run(d, world, (0.0, 0.0, 0.0), 0,
                              until=lambda d: d.stage == 'locked')
    fwd = [a for a in actions if a.jog_distance > 0]
    check('T3 对准+30cm主动锁定+request_stand',
          d.stage == 'locked' and d.request_stand and len(actions) == len(fwd)
          and not d.failure and not d.complete,
          f'stage={d.stage} request_stand={d.request_stand} '
          f'actions={len(actions)} failure={d.failure!r}')
    return d, world, pose, t


# ── T4: 对准 0.8m → approach 匍匐前进 → 0.30 锁定 ───────────────────

def t4_approach_then_lock(params):
    # 正对桩: pile z = 0.80 (PILE_X = 0.95), wall z = 1.50 (观察点)
    d, node, world = make_dual(params, 0.95)
    pose, t, actions, _ = run(d, world, (0.0, 0.0, 0.0), 0,
                              until=lambda d: d.request_stand)
    fwd = [a for a in actions if a.jog_distance > 0]
    bad = [a for a in actions if a.jog_distance < 0 or a.turn_angle
           or a.lateral_distance]
    check('T4 匍匐approach前进→0.30锁定 (步骤4→5)',
          d.stage == 'locked' and d.request_stand and len(fwd) >= 8
          and not bad and not d.failure,
          f'stage={d.stage} forward={len(fwd)} bad={len(bad)} '
          f'pile_z={optical(pose, world.pile)[2]:.3f} failure={d.failure!r}')


# ── T5: locked → 站立 → 墙码纯直行 → complete ───────────────────────

def t5_locked_straight_complete(params):
    d, world, pose, t = t3_lock_at_30cm(params)
    # 节点职责: ensure_stand 确认后清 request_stand + 重置观测窗 (T7 已验 DualPosture)
    d.request_stand = False
    pose, t, actions, complete = run(d, world, pose, t, pile_on=False)
    fwd = [a for a in actions if a.jog_distance > 0]
    wall_z = optical(pose, world.wall)[2]
    check('T5 locked直行→0.50 complete + 航向资格',
          complete and d.progress and d.qualified_ns > 0 and len(fwd) >= 9
          and not d.failure and abs(wall_z - 0.50) <= 0.02,
          f'complete={complete} progress={d.progress} forward={len(fwd)} '
          f'wall_z={wall_z:.3f} failure={d.failure!r}')


# ── T6: locked overshoot 豁免 vs 非 locked fail ─────────────────────

def t6_overshoot(params):
    # 6a: locked 且墙码 z=0.45 (< target-0.02) → 豁免, 出后退修正动作
    # 6b: approach 阶段同情况 → 命中 overshoot fail (locked 豁免不外溢)
    # 相机距墙 0.45m 在世界模型里几何不可达: 桩码贴桩底座在墙前 0.70m, 会落到
    # 相机身后, observe 整帧拒绝 —— 旧场景表达式 (0.60+0.45-0.15) 实际产出
    # wall z=1.45, 测的根本不是 overshoot。改为直接注入滤波状态 (与 pytest
    # 单测同款); locked 下 observe 本就强制 pile=None, 注入合法。
    wall = (0., -.2, 0.45)
    d, node, world = make_dual(params, 0.45 + CAM[0])
    d.stage = 'locked'
    d.stopped(int(2e9))
    t = int(3.6e9)
    for i in range(4):
        stamp = t + i*int(2e8)
        d.observe(stamp, stamp, wall, None, pile_missing=True)
    seq = d.plan_dual(None, 'omni', None, t + int(6e8))
    check('T6a locked 墙码z跳近豁免overshoot',
          not d.failure and seq and seq[0].jog_distance < 0,
          f'failure={d.failure!r} jog={seq[0].jog_distance if seq else None}')
    d2, _, _ = make_dual(params, 0.45 + CAM[0])
    d2.stage = 'approach'
    d2.stopped(int(6e9))
    t2 = int(7.6e9)
    for i in range(4):
        stamp = t2 + i*int(2e8)
        d2.observe(stamp, stamp, wall, None, pile_missing=True)
    seq2 = d2.plan_dual(None, 'omni', None, t2 + int(6e8))
    check('T6b 非locked命中overshoot fail',
          seq2 is None and 'overshoot' in d2.failure,
          f'failure={d2.failure!r}')


# ── T7: DualPosture 六态分支 ────────────────────────────────────────

def make_posture(params, ready=True, result=True):
    node = FakeNode(params, clients={
        '/l1w_control/lie_down': FakeClient(ready, result),
        '/l1w_control/stand_up': FakeClient(ready, result),
    })
    return DualPosture(node), node, node._clients['/l1w_control/lie_down'], \
        node._clients['/l1w_control/stand_up']


def t7_dual_posture(params):
    # F1: 服务缺失 → 1s 宽限 → FAILED
    dp, node, lie, stand = make_posture(params, ready=False)
    t = 0
    ok = dp.ensure_crouch(t)
    t += int(0.5e9)
    ok2 = dp.ensure_crouch(t)
    t += int(0.6e9)
    dp.ensure_crouch(t)
    check('T7-F1 趴下服务缺失→宽限后fail',
          not ok and not ok2 and dp._phase == DualPosture.FAILED
          and '服务不可用' in dp.failure and lie.calls == 0,
          f'failure={dp.failure!r} calls={lie.calls}')

    # F2a: lie_down 成功 + settle 3s + motion_enabled=True → CROUCHED
    dp, node, lie, stand = make_posture(params)
    t = 0
    dp.ensure_crouch(t)
    t += int(0.05e9)
    mid = dp.ensure_crouch(t)
    set_motion(node, True)
    t += int(3.0e9)
    done = dp.ensure_crouch(t)
    check('T7-F2a 趴下响应+settle→匍匐就绪',
          not mid and done and dp._phase == DualPosture.CROUCHED
          and lie.calls == 1,
          f'mid={mid} done={done} phase={dp._phase}')

    # F2b: settle 后 motion_enabled=False → 桥锁 cmd_vel → FAILED
    dp, node, lie, stand = make_posture(params)
    t = 0
    dp.ensure_crouch(t)
    t += int(0.05e9)
    dp.ensure_crouch(t)
    set_motion(node, False)
    t += int(3.0e9)
    dp.ensure_crouch(t)
    check('T7-F2b 桥锁cmd_vel→明确fail',
          dp._phase == DualPosture.FAILED and 'cmd_vel' in dp.failure,
          f'failure={dp.failure!r}')

    # F3: lie_down 拒绝 → 重试 2 次耗尽 → FAILED
    dp, node, lie, stand = make_posture(params, result=False)
    t = 0
    dp.ensure_crouch(t)
    for _ in range(6):
        t += int(0.1e9)
        dp.ensure_crouch(t)
    check('T7-F3 服务拒绝重试耗尽→fail',
          dp._phase == DualPosture.FAILED and '重试耗尽' in dp.failure
          and lie.calls == 3,
          f'failure={dp.failure!r} calls={lie.calls}')

    # F4: CROUCHED → stand_up → motion_enabled=True → STANDED
    dp, node, lie, stand = make_posture(params)
    t = 0
    dp.ensure_crouch(t)
    t += int(0.05e9)
    dp.ensure_crouch(t)
    set_motion(node, True)
    t += int(3.0e9)
    assert dp.ensure_crouch(t) is True, 'F4 前置: 匍匐就绪'
    t += int(0.1e9)
    mid = dp.ensure_stand(t)
    t += int(0.1e9)
    done = dp.ensure_stand(t)
    check('T7-F4 stand_up+motion确认→站立就绪',
          not mid and done and dp._phase == DualPosture.STANDED
          and stand.calls == 1,
          f'mid={mid} done={done} phase={dp._phase}')

    # F4b: stand_up 受理但 motion_enabled 永不变 True → 重试耗尽 → FAILED
    dp, node, lie, stand = make_posture(params)
    t = 0
    dp.ensure_crouch(t)
    t += int(0.05e9)
    dp.ensure_crouch(t)
    set_motion(node, True)
    t += int(3.0e9)
    assert dp.ensure_crouch(t) is True, 'F4b 前置: 匍匐就绪'
    # 重新构造: motion 无回传 (None 语义) 场景复测站立确认路径
    dp, node, lie, stand = make_posture(params)
    t = 0
    dp.ensure_crouch(t)
    t += int(3.05e9)          # 无 motion 回传 → 放行 + warn (None 语义)
    assert dp.ensure_crouch(t) is True, 'F4b 前置: 无回传放行'
    t += int(0.1e9)
    dp.ensure_stand(t)
    for _ in range(80):
        t += int(0.1e9)
        dp.ensure_stand(t)
        if dp._phase == DualPosture.FAILED:
            break
    check('T7-F4b 站立确认超时重试耗尽→fail',
          dp._phase == DualPosture.FAILED and '超时' in dp.failure
          and stand.calls == 3,
          f'failure={dp.failure!r} calls={stand.calls}')


# ── T8: ActionWatch ─────────────────────────────────────────────────

def t8_action_watch(d):
    plan = ActionPlan(kind='forward', jog_distance=0.05)
    w = ActionWatch(plan, 0, (0.0, 0.0, 0.0), 0.05, JOG_SPEED, d.p)
    ok_running = w.check(int(0.5e9), (0.04, 0.0, 0.0), int(0.5e9)) == ''
    ok_done = w.check(int(1.0e9), (0.05, 0.0, 0.0), int(1.0e9)) == ''
    w2 = ActionWatch(plan, 0, (0.0, 0.0, 0.0), 0.05, JOG_SPEED, d.p)
    # 反向判据有起步宽限 (action_startup_sec) 且门槛为 max(3×噪声, 半程) ——
    # 旧探针 (0.5s, -0.02) 落在宽限期内且低于门槛, 探不到判据。改取宽限之后
    # + 整步反向 (-target), 与 pytest 的 test_reverse_threshold 同款。
    late = int((d.p('action_startup_sec') + .1) * 1e9)
    opposite = w2.check(late, (-0.05, 0.0, 0.0), late)
    w3 = ActionWatch(plan, 0, (0.0, 0.0, 0.0), 0.05, JOG_SPEED, d.p)
    silent = w3.check(int(2.5e9), (0.0, 0.0, 0.0), int(2.5e9))
    check('T8 ActionWatch 完成/反向/无响应',
          ok_running and ok_done and 'opposite' in opposite
          and 'no odometry response' in silent,
          f'run={ok_running!r} done={ok_done!r} '
          f'opp={opposite!r} silent={silent!r}')


# ── T9: 站立 profile 全程 ───────────────────────────────────────────

def t9_standing_end_to_end(params):
    """T9 站立 profile 全程 (dual.crouch_enable: false, obs 1.8 / near 1.70):
    - 匍匐从不触发: d.crouch False (节点据此跳过 ensure_crouch), 且全程
      request_stand False (节点不调 ensure_stand → 无 lie_down/stand_up 服务);
    - locked 只能由 approach 桩码丢失闭锁进入 (30cm 主动锁定被 crouch 门关掉);
    - 闭锁那一刻注入 2° 航向扰动 (> straight_yaw_tol_deg 1.5°): locked
      bearing 微调必须至少触发一次。扰动不能放在起步 —— 双码纠偏的 theta
      门是 min(对准门, 1°), 桩码可见的全程都会把航向误差修掉 (这本就是
      设计); locked 微调的职责窗口恰是桩码丢失之后, 对应现场最后一程
      盲走中的航向漂移;
    - 0.50m 完成。
    """
    d, node, world = make_dual(params, 1.25)   # wall z=1.8 = 站位, pile z=1.10
    pose = (0.0, 0.0, 0.0)
    d.stopped(0)
    t = int(1.6e9)
    actions, complete = [], False
    locked_seen = locked_yaws = disturbed = 0
    for _ in range(900):
        stage_before = d.stage
        world.observe_tick(d, pose, t)
        seq = d.plan_dual(None, 'omni', None, t)
        if d.failure:
            break
        assert not d.request_stand, '站立 profile 下 request_stand 必须恒 False'
        if seq and seq[0].kind == 'done':
            complete = True
            break
        if seq:
            plan = seq[0]
            if d.stage == 'locked':
                locked_seen += 1
                if plan.turn_angle:
                    locked_yaws += 1
            d.action_started(plan, t)
            pose = apply_plan(plan, pose)
            if stage_before == 'approach' and d.stage == 'locked' and not disturbed:
                disturbed = 1
                pose = (pose[0], pose[1], pose[2] + math.radians(2.))
            d.action_completed()
            d.stopped(t)
            amount = plan.turn_angle or plan.lateral_distance or plan.jog_distance
            t += int(max(0.5, abs(amount)/JOG_SPEED)*1e9) + int(1.6e9)
            actions.append(plan)
            continue
        t += int(STEP*1e9)
    wall_z = optical(pose, world.wall)[2]
    check('T9 站立profile全程→complete+bearing微调',
          complete and not d.failure and not d.crouch and disturbed
          and locked_seen > 0 and locked_yaws >= 1
          and abs(wall_z-0.50) <= 0.02,
          f'complete={complete} failure={d.failure!r} crouch={d.crouch} '
          f'locked_yaws={locked_yaws} locked_plans={locked_seen} '
          f'actions={len(actions)} wall_z={wall_z:.3f} stage={d.stage}')


def main():
    crouch = load_params(CROUCH_PROFILE)
    standing = load_params(STANDING_PROFILE)
    # T1/T2/T6/T7/T8 与 profile 无关的场景走匍匐参数 (历史场景按匍匐调);
    # T3/T4/T5 匍匐全链 (含 T3 内嵌的 30cm 主动锁定); T9 站立 profile 全程。
    d0, _, _, _, _ = t1_acquire_forward(crouch)
    t2_acquire_reverse(crouch)
    t4_approach_then_lock(crouch)
    t5_locked_straight_complete(crouch)
    t6_overshoot(crouch)
    t7_dual_posture(crouch)
    t8_action_watch(d0)
    t9_standing_end_to_end(standing)

    failed = [n for n, ok in RESULTS if not ok]
    total = len(RESULTS)
    print(f'\n{"=" * 60}')
    print(f'{"PASS" if not failed else "FAIL"}: {total - len(failed)}/{total} '
          f'双码姿态切换 + 状态机全分支验证')
    if failed:
        print('FAILED: ' + ', '.join(failed))
        sys.exit(1)


if __name__ == '__main__':
    main()
