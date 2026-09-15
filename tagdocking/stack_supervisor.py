#!/usr/bin/env python3
"""停泊栈按需启停 —— 用时才起, 用完就收。

    ros2 run tagdocking docking_supervisor

**动机**: 停泊栈三个进程 (rtsp_camera / apriltag_node / docking_node) 常驻着,
而停泊是个偶发的、人触发的动作, 绝大多数时间它们在空转。空转并不比工作省:
`rtsp_camera._pump` (rtsp_camera.py:527-543) 的 `max_fps` 限的是**发布**不是
**解码**, `cap.read()` 一直在跑; apriltag 对每一帧都做检测, 且
`detector.decimate: 1.0` (docking.launch.py) 刻意关掉了它自己的降采样。
也就是说 IDLE 态的开销结构上约等于停泊态的开销。

**分层**: 本节点只管**进程生命周期**, 一行停泊逻辑都不碰。停泊逻辑全在
docking_node 里, supervisor 对它只做两件事: 起它、以及把 dock/undock/cancel
原样转发给它。栈起没起、要不要收, 是 supervisor 的事; 怎么停泊, 是 docking_node
的事。这条界线是这次改动的硬约束。

**为什么是独立节点而不是塞进 web**: 两个理由。一是 ROS 入口要在栈没起时也能用
(外部平台直接调 ROS 服务, 不该被迫走 HTTP); 二是 web 是 `Restart=always` 的,
web 一重启就带走整个停泊栈是不可接受的 —— 停泊可能正进行到一半。

**为什么是子进程而不是 systemctl**: 父进程启停自己的子进程不需要任何权限。
走 systemctl 要经 D-Bus → systemd → polkit, 而本机
`org.freedesktop.systemd1.manage-units` 是 `auth_admin` (要管理员密码),
supervisor 以 nvidia 跑且无 tty。放开就得新写一条 root 拥有的 polkit 规则 ——
仓库外、刷机即丢, 且是本项目引入的第一个提权机制。仓库里已经用子进程解过一遍
同样的问题 (robot_web_console/providers/odin_mapping.py:645-693, 动机一模一样:
"SLAM mapping and Nav2 compete for CPU on the robot")。
"""

from __future__ import annotations

import argparse
import contextlib
import json
import os
import signal
import subprocess
import threading
import time
from typing import Optional

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time

from sensor_msgs.msg import CameraInfo
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

import tf2_ros

from tagdocking.launch_processes import (
    DOCKING_LAUNCH_CMD, launch_tree, process_snapshot, stop_launch_tree)


START_SH = '/home/nvidia/whale-nav/src/tagdocking/scripts/start_docking.sh'
STACK_LOG = '/tmp/tagdocking_stack.log'

SRV_DOCK = '/docking_node/start_docking'
SRV_UNDOCK = '/docking_node/start_undock'
SRV_CANCEL = '/docking_node/cancel_docking'

STATE_TOPIC = '/docking_node/state'
CAMERA_INFO_TOPIC = '/camera_sync/camera_info'
DETECTIONS_TOPIC = '/detections'

# 状态族 —— 与 state_machine.py:44-62 一一对应。改那边记得同步这里。
# 注意 retrying 是**活动态**不是终态 (节点驱动的盲退重锁, max_retries=2),
# idle 既不在活动态里也不在终态里。
ACTIVE_STATES = frozenset({
    'search_tag', 'align', 'approach', 'final_servo', 'retrying', 'undocking'})
TERMINAL_STATES = frozenset({
    'docked', 'undocked', 'tag_lost', 'timeout', 'motion_failed', 'cancelled'})

# 就绪门整体预算。实测冷启动 5~8 秒 (RTSP 连接 ~2s + 管线 0.1~0.5s + DDS 发现)。
READY_TIMEOUT_SEC = 20.0
# 五项信号到齐后再多等一下。信号证明的是"各自的发现完成了", 这一秒是留给
# docking_node 那侧收 /tf_static 的余量 —— 见下面 _wait_ready 的长注释。
READY_SETTLE_SEC = 1.0
# 终态后延迟多久收栈。留这段是为了"泊入完接着泊出""失败后立刻重试"不用再付一次
# 冷启动。
IDLE_STOP_DELAY_SEC = 30.0
# 转发 start_docking 后, 多久没见到活动态就认为没真的动起来。
ACTIVE_WAIT_SEC = 10.0
# state 断流多久算栈掉了。docking_node 每个 tick 都发 (20Hz), 3 秒是 60 个 tick。
STATE_STALE_SEC = 3.0
# 新建就绪探针后留给数据送达的窗口 (只 ready_check 诊断用)。最慢的一项是
# latched /tf_static: 它是异步送达的, 实测 ~100ms。camera_info / odom /
# detections 都 ≥30Hz, 远快于它。1.5s 是十几倍余量 —— 这条路径不赶时间,
# 宁可慢也不要假阴性 (对着健康的栈报"未就绪"会把人往错方向带)。
TF_PROBE_SETTLE_SEC = 1.5
# /proc 全扫的最小间隔。这是本节点里最贵的一件事 —— 实测 485 个进程 62ms/次,
# 按定时器的 0.5s 跳就是一个核的 12%。在一个"为省 CPU 而做"的节点里花 12% 去看
# 有没有别人起的栈, 那是把省下来的又烧回去。所以扫描单独限速: 接管和"栈是不是
# 死了"这两项容忍 5 秒的发现延迟 (1.2% 一个核)。自己起的栈用 proc.poll() 判死,
# 那个是免费的, 不受这条限速影响。
PROC_SCAN_SEC = 5.0
# video 占用的租约时长。持有方 (web) 每 5 秒续一次, 这里给 15 秒 = 容得下两次
# 丢失的续租再过期。设成刚好 5 秒会让一次 GC 停顿或一拍网络抖动就把正在看的
# 视频掐掉。见 _srv_video_hold 里为什么是租约而不是开关。
VIDEO_LEASE_TTL_SEC = 15.0


class StackSupervisor(Node):

    def __init__(self, args):
        super().__init__('docking_supervisor')

        self._cbg = ReentrantCallbackGroup()
        self._lock = threading.RLock()

        # ── 进程状态 ──
        self._proc: Optional[subprocess.Popen] = None
        self._owned_pid: Optional[int] = None
        self._stack = 'down'          # down | starting | ready | stopping
        self._mode = 'none'           # none | camera | full
        self._detail = ''

        # ── 占用引用计数 ──
        # 有 docking 占用 → 要满栈; 只有 video 占用 → 相机就够; 都没有 → 收栈。
        # 把"视频只拉相机"和"终态后延迟收栈"统一成同一件事, 不用两套计时逻辑。
        self._holds: set = set()
        self._release_at: Optional[float] = None   # docking 占用的到期时刻
        # video 占用的租约到期时刻, 见 _srv_video_hold。
        self._video_lease_until: Optional[float] = None
        # 边沿触发用: 转发成功后置 True, 见到一次活动态才清掉。见 _evaluate。
        self._awaiting_active = False
        self._forward_mono = 0.0
        # 起栈/收栈进行中。**必须有**: 定时器回调挂在 ReentrantCallbackGroup 上、
        # executor 是多线程的, 而 _teardown 会阻塞好几秒 (等进程逐级退出)。
        # 没有这个标志时, 后面的 tick 会看到 stack='stopping' (≠'down') 就再叫
        # 一次 _teardown —— 实测 0.5 秒一次连叫三遍, 几个线程对着同一批 pid
        # 抢着发信号。
        self._busy = False
        # 上一次 /proc 全扫的时刻, 见 PROC_SCAN_SEC。
        self._last_proc_scan = 0.0
        # 就绪探针的节点名序号, 见 _make_probe。
        self._tf_probe_seq = 0

        # ── 就绪信号 (订阅回调里置位, 门里轮询) ──
        self._state: Optional[str] = None
        self._state_mono = 0.0
        self._cam_info_seen = False
        self._odom_seen = False
        self._detections_seen = False

        self._ready_timeout = args.ready_timeout
        self._idle_delay = args.idle_delay
        self._odom_topic = args.odom_topic
        # 就绪门要查的那对 frame。默认值与 docking.launch.py 的 base_frame /
        # camera_frame 默认值一致 —— 换机器改了那边, 这里也要跟着改, 否则门会
        # 永远等不到一个不存在的 frame。
        self._base_frame = self.declare_parameter(
            'base_frame', 'base_link').value
        self._camera_frame = self.declare_parameter(
            'camera_frame', 'camera_color_optical_frame').value

        # 只有 state 是常驻订阅: 20Hz 的 String, 便宜, 而且必须一直听 —— 终态
        # 判定和"栈是不是断流了"都靠它。
        # camera_info / odom / detections **不常驻**, 它们跟 TF 监听一样只在
        # 就绪门开着的那几秒里存在, 见 _make_probe。
        self.create_subscription(
            String, STATE_TOPIC, self._on_state, 10, callback_group=self._cbg)

        self._cli = {
            'dock': self.create_client(Trigger, SRV_DOCK,
                                       callback_group=self._cbg),
            'undock': self.create_client(Trigger, SRV_UNDOCK,
                                         callback_group=self._cbg),
            'cancel': self.create_client(Trigger, SRV_CANCEL,
                                         callback_group=self._cbg),
        }

        self.create_service(Trigger, '~/dock', self._srv_dock,
                            callback_group=self._cbg)
        self.create_service(Trigger, '~/undock', self._srv_undock,
                            callback_group=self._cbg)
        self.create_service(Trigger, '~/cancel', self._srv_cancel,
                            callback_group=self._cbg)
        self.create_service(SetBool, '~/video_hold', self._srv_video_hold,
                            callback_group=self._cbg)
        self.create_service(Trigger, '~/stack_down', self._srv_stack_down,
                            callback_group=self._cbg)
        self.create_service(Trigger, '~/ready_check', self._srv_ready_check,
                            callback_group=self._cbg)

        self._status_pub = self.create_publisher(String, '~/status', 10)
        self.create_timer(0.5, self._tick, callback_group=self._cbg)

        self._adopt_existing()
        self.get_logger().info(
            f'docking_supervisor 已就绪 (按需启停; 空闲 {self._idle_delay:.0f}s 后收栈)')

    # ── 检测话题订阅 ────────────────────────────────────────────────

    def _make_detections_sub(self, node):
        """在探针节点上订阅 /detections, 作为"apriltag ↔ 相机已经接上"的证据。

        前提已核实: `AprilTagNode.cpp:248` 的 `pub_detections->publish(...)` 在
        逐个检测的循环 (:234) **之外**, 所以**每帧都发, 看不到码时发的是空数组**。
        因此这个信号与"当前有没有码在视野里"无关 —— 否则停泊开始前 (正常就是
        没码的状态) 这道门会永远等不到。

        apriltag_msgs 是 apriltag_ros 带来的, 不是本包的 <depend>。拿不到就
        降级 (那一项直接算通过) 而不是让整个 supervisor 起不来 —— 少一项信号
        比没有 supervisor 好。
        """
        try:
            from apriltag_msgs.msg import AprilTagDetectionArray
        except ImportError:
            self.get_logger().warn(
                'apriltag_msgs 不可用, 就绪门跳过 /detections 一项')
            self._detections_seen = True
            return None
        return node.create_subscription(
            AprilTagDetectionArray, DETECTIONS_TOPIC,
            self._on_detections, 1)

    # ── 订阅回调 (只置位, 不做决策) ─────────────────────────────────

    def _on_state(self, msg: String):
        self._state = msg.data
        self._state_mono = time.monotonic()

    def _on_cam_info(self, _msg):
        self._cam_info_seen = True

    def _on_odom(self, _msg):
        self._odom_seen = True

    def _on_detections(self, _msg):
        self._detections_seen = True

    # ── 接管 ────────────────────────────────────────────────────────

    def _adopt_existing(self):
        """启动时若已有一棵 tagdocking launch 树, 接管而不是重起。

        supervisor 崩了或被 systemd 重启, `owned_pid` 就丢了。手工
        `ros2 launch tagdocking docking.launch.py` 起的那棵也一样。两种情况下都
        接管 (owned_pid=None, 靠 argv 匹配认): 不重启它, 但照常能停它、照常挂
        空闲定时器。这也是手工起栈仍然安全的原因。
        """
        tree = launch_tree(process_snapshot(), DOCKING_LAUNCH_CMD)
        if not tree:
            return False
        with self._lock:
            self._owned_pid = None
            self._stack = 'ready'
            # 接管时认不出对方当初是 camera 还是 all 起的 —— argv 里有 nodes:=,
            # 但手工起的那条没有。一律当满栈: 猜低了会在有人点停泊时白白重起
            # 一次, 猜高了只是少省一点 CPU。
            self._mode = 'full'
            self._detail = f'接管既有栈 ({len(tree)} 个进程)'
            self._holds.add('adopted')
            self._release_at = time.monotonic() + self._idle_delay
        self.get_logger().info(
            f'接管既有停泊栈: {sorted(tree)} —— 不重启, 空闲后照常收')
        return True

    # ── 起栈 / 收栈 ─────────────────────────────────────────────────

    @contextlib.contextmanager
    def _busy_guard(self, what: str):
        """占住"正在动进程"这个标志; 拿不到就抛。

        用 contextmanager 而不是在每条路径上手动置位/复位: 复位必须在异常路径上
        也发生, 否则一次失败的起栈就把 supervisor 永久卡在 busy 上, 之后既不起
        也不收, 而且从外面看它还是一副正常样子。
        """
        with self._lock:
            if self._busy:
                raise RuntimeError(f'另一个操作正在进行, {what} 被跳过')
            self._busy = True
        try:
            yield
        finally:
            with self._lock:
                self._busy = False

    def _spawn(self, mode: str):
        """起一棵栈。调用者必须已经确认当前没有栈 (或已收掉)。"""
        # 起之前无条件清一次场。幂等 (没东西就是空转), 但能收掉两类东西:
        # 上一棵没收干净的树, 以及 launch 根先死、被 init 收养的游兵。后者单靠
        # 进程树找不到, 而它还在发 /camera_sync/* 和第二份静态 TF —— 下次起栈
        # 就是两个实例对着同一个 TF 帧双重广播, 量测在两组矛盾值之间跳。
        try:
            stop_launch_tree(DOCKING_LAUNCH_CMD)
        except RuntimeError as exc:
            self.get_logger().warn(f'启动前清场未净: {exc}')

        # 复用 start_docking.sh 而不是自己拼 ROS 环境: 那套 source 顺序和
        # `set +u` 的坑已经在脚本里解过一遍了, 抄第二遍就是第二个真相来源。
        # 脚本的 docking 模式末尾是 `exec ros2 launch … "$@"`, 额外参数原样透传,
        # 所以 nodes:=camera 不用改脚本。两层 exec 让进程树里不留 wrapper ——
        # Popen 拿到的 pid 就是 `ros2 launch` 自己。
        cmd = f'exec {START_SH} docking nodes:={mode}'
        logfile = open(STACK_LOG, 'ab')
        try:
            proc = subprocess.Popen(
                ['/bin/bash', '-c', cmd],
                stdout=logfile, stderr=subprocess.STDOUT,
                start_new_session=True)
        finally:
            logfile.close()
        with self._lock:
            self._proc = proc
            self._owned_pid = proc.pid
            self._stack = 'starting'
            self._mode = mode
            self._detail = f'已拉起 (pid {proc.pid}), 等待就绪'
        self.get_logger().info(
            f'起栈: mode={mode} pid={proc.pid} 日志={STACK_LOG}')

    def _teardown(self):
        """收栈。活动态时先取消, 再逐级收进程树。

        调用者应当先拿 _busy_guard —— 唯一的例外是 shutdown(), 它有意绕过。
        """
        with self._lock:
            if self._stack == 'down':
                return
            self._stack = 'stopping'
            owned = self._owned_pid
            state = self._state

        # 正在动就先让它自己停下来: cancel_docking (docking_node.py:2423-2431)
        # 同步调 publish_stop()。比直接发信号温和, 且走的是节点自己的停车路径。
        if state in ACTIVE_STATES:
            self.get_logger().info(f'收栈前先取消 (当前 {state})')
            self._call_downstream('cancel', timeout=3.0)
            deadline = time.monotonic() + 2.0
            while time.monotonic() < deadline:
                if self._state not in ACTIVE_STATES:
                    break
                time.sleep(0.1)

        try:
            stop_launch_tree(DOCKING_LAUNCH_CMD, owned_pid=owned)
            detail = ''
        except RuntimeError as exc:
            detail = str(exc)
            self.get_logger().error(f'收栈未净: {exc}')

        with self._lock:
            self._proc = None
            self._owned_pid = None
            self._stack = 'down'
            self._mode = 'none'
            self._release_at = None
            self._detail = detail
            self._reset_ready_signals()
        self.get_logger().info('栈已收')

    def _reset_ready_signals(self):
        """收栈时清掉全部就绪信号。

        后三项探针每次建立时也会自己清一遍 (见 _make_probe) —— 那一遍才是判据
        正确性所依赖的。这里再清一次是为了 status 里显示出来的东西别停留在
        "栈都收了还一片绿"。
        """
        self._state = None
        self._state_mono = 0.0
        self._cam_info_seen = False
        self._odom_seen = False
        self._detections_seen = False

    # ── 就绪门 ──────────────────────────────────────────────────────

    def _wait_ready(self) -> tuple:
        """等到可以安全转发 start_docking 为止。返回 (ok, 说明)。

        **这道门是整个设计里最要紧的一处**, 它关掉的是一条今天被掩盖着的竞态。

        `docking_node.py:754-766` 查相机外参用的是**零超时、不重试**的 lookup:

            tf = self._tf_buffer.lookup_transform(
                base_frame, camera_frame, Time(seconds=0),
                timeout=Duration(seconds=0))

        失败即 `_dual.failure`, 调用点 (:1145-1150 和 :1396) 直接 abort_motion →
        MOTION_FAILED。而 `_on_start_docking` (:2411-2421) **没有任何就绪检查**,
        唯一的拒绝理由是 'already active'。

        今天这条竞态碰不到: 栈开机起一次, 然后在 IDLE 上坐几个小时才有人停泊。
        **改成按需后, start_docking 会紧贴在进程刚起来的几秒内发生 —— 正好踩进
        竞态。** 不能改 docking_node (硬约束), 所以只能由 supervisor 在外面守门。

        `service_is_ready()` **不算**就绪 —— 三个服务在 __init__ 里就建好了
        (:545-550), 比相机/TF/odom 早好几秒。它只当"进程还活着"的前置条件用。
        """
        deadline = time.monotonic() + self._ready_timeout
        tf_buf, probe = self._make_probe()
        try:
            while time.monotonic() < deadline:
                # 子进程提前退了就别干等到超时。照 odin_mapping.py:668-671:
                # 内参文件路径打错是秒级失败, 不该让人等满 20 秒。
                proc = self._proc
                if proc is not None and proc.poll() is not None:
                    return False, (f'栈进程已退出 (返回码 {proc.returncode}), '
                                   f'看日志 {STACK_LOG}')
                missing = self._ready_missing(tf_buf)
                if not missing:
                    # 五项到齐再多等一下。每一项证明的是**它自己**那对 DDS
                    # 端点发现完成了, 而竞态的本质是 docking_node 那侧的
                    # /tf_static 迟到。/tf_static 是 transient-local (latched),
                    # 晚加入者在发现完成后必然收到, 这一秒是给那一步的余量。
                    time.sleep(READY_SETTLE_SEC)
                    return True, ''
                time.sleep(0.2)
            return False, f'就绪超时 ({self._ready_timeout:.0f}s), 仍缺: ' + \
                          ', '.join(self._ready_missing(tf_buf))
        finally:
            self._drop_probe(probe)

    def _drop_probe(self, probe):
        """拆掉临时就绪探针 —— 用完就拆, 见 _make_probe 的注释。"""
        if probe is None:
            return
        node, executor, thread = probe
        try:
            # 顺序是有讲究的: 先让 executor 退出 spin, join 到它真的不在转了,
            # 最后才 destroy_node。反过来 (边转边销毁) 就是下面注释里那个
            # InvalidHandle。
            executor.shutdown()
            thread.join(timeout=2.0)
            if thread.is_alive():
                # 线程没退就别销毁节点了 —— 宁可漏一个节点, 也不要去踩一个
                # 正在被 spin 的句柄。漏掉的那个只在进程退出时才回收。
                self.get_logger().warn('就绪探针线程未退出, 不销毁节点以免踩句柄')
                return
            node.destroy_node()
        except Exception as exc:                       # noqa: BLE001
            self.get_logger().warn(f'就绪探针拆除失败: {exc}')

    def _make_probe(self):
        """临时就绪探针 —— **独立 Node + 独立 executor + 独立线程**。

        它承载就绪门里四项需要订阅的信号: TF 外参、camera_info、odom、
        detections。用完即拆。

        **为什么这四项都不常驻** (这条是量出来的, 别为了"省事"挪回 __init__):
        `/odin1/odometry_highfreq` 实测 ~385Hz。把它常驻订阅在这里, 光是
        rclpy 反序列化 Odometry (两个 36 元协方差数组) 就吃掉 **48% 一个核**
        —— 实测分线程画像: 主线程 26.6%, 两个 executor 线程各 8.5%。而这个
        订阅的全部产出是把 `_odom_seen` 置成 True 一次, 只在就绪门开着的那
        5~8 秒里被读。也就是说: 为一个每次停泊只需要一瞬的布尔量, 常驻烧掉
        三分之一个核 —— 在一个**为省 CPU 而做的节点**里, 这一项就把满栈
        171% 里省下来的其中 48% 又烧回去了。空回调的对照实验 (MTE+Reentrant
        0.40%) 证明贵的不是 executor, 就是这几条订阅本身。
        TF 那条同理且更甚: TransformListener 订阅 /tf, 本机 Nav2/odin 把它刷到
        ~815Hz。下面查询一律 timeout=0, 只问"现在缓冲区里有没有", 绝不阻塞等待
        —— 本机有过在回调里对着这条洪水做带 timeout 的 lookup 把自己锁死的先例。
        还有一条与判据有关的理由: 常驻 buffer 里的 static 变换**永不过期**, 栈
        收掉之后那一项会永远绿灯, 就不再是判据了。用完即抛才能每次都问一个新
        问题 —— 同理下面要先清掉三个标志位, 否则读到的是上一次探针的答案。

        **为什么要独立 executor** (这条是踩出来的, 别合回主节点):
        探针是用完就销毁的, 而 destroy_subscription 撞上主 executor 已经把这个
        句柄收进 wait set 的那一刻, spin() 会抛
        `InvalidHandle: cannot use Destroyable because destruction was requested`
        —— 那是从 spin 里抛出来的, 收不住, 整个 executor 当场死掉。实测连着调
        ready_check 第 4~5 次就复现: supervisor 直接退出, 而它退出时会顺手收栈,
        于是一个**只读的诊断接口**把正在跑的栈带走了。独立 executor 让销毁的
        影响止步于探针自己。
        """
        node = None
        try:
            self._tf_probe_seq += 1
            # 先清标志位: 三项都是"这次探针期间有没有收到", 留着上次的答案会让
            # 一个早已收掉的栈看上去仍然就绪。
            self._cam_info_seen = False
            self._odom_seen = False
            self._detections_seen = False
            # 名字带序号: 两个探针并存时 (ready_check 撞上就绪门) 不要重名。
            node = Node(f'docking_supervisor_probe_{self._tf_probe_seq}')
            buf = tf2_ros.Buffer()
            tf2_ros.TransformListener(buf, node, spin_thread=False)
            node.create_subscription(
                CameraInfo, CAMERA_INFO_TOPIC, self._on_cam_info, 1)
            node.create_subscription(
                Odometry, self._odom_topic, self._on_odom, 1)
            self._make_detections_sub(node)
            executor = SingleThreadedExecutor()
            executor.add_node(node)
            thread = threading.Thread(
                target=executor.spin, daemon=True,
                name=f'ready-probe-{self._tf_probe_seq}')
            thread.start()
            return buf, (node, executor, thread)
        except Exception as exc:                       # noqa: BLE001
            self.get_logger().warn(f'就绪探针建不起来, 跳过该项: {exc}')
            if node is not None:
                try:
                    node.destroy_node()
                except Exception:                      # noqa: BLE001
                    pass
            return None, None

    def _ready_missing(self, tf_buf) -> list:
        """还缺哪些就绪信号。空列表 = 可以转发了。"""
        missing = []

        # 1. 进程还活着且服务端点可见 —— 前置条件, 不是就绪判据。
        if not self._cli['dock'].service_is_ready():
            missing.append('start_docking 服务未出现')

        # 2. 控制循环真的在转。判据是"有 state 流量, 且不是活动态" ——
        #    **不能判 == idle**: 终态是会 latch 的 (reset() 没有生产调用者),
        #    接管一个上次停在 docked/motion_failed 的栈时, state 会一直是那个
        #    终态。判 idle 的话热栈复用和接管这两条路会永远等不到。
        if not self._state_mono:
            missing.append('state 无流量')
        elif time.monotonic() - self._state_mono > STATE_STALE_SEC:
            missing.append('state 已断流')
        elif self._state in ACTIVE_STATES:
            missing.append(f'栈正忙 ({self._state})')

        # 3. 相机出图了。
        if not self._cam_info_seen:
            missing.append('camera_info 未收到')

        # 4. odom 在。缺它 dual 路径 (:1393-1395) 会瞬间 MOTION_FAILED。
        #    这一路由 odin 驱动常驻发布, 与本栈无关, 基本瞬时满足 —— 它的作用
        #    是"odom 真的存在"这项别漏检, 而不是等待。
        if not self._odom_seen:
            missing.append(f'odom ({self._odom_topic}) 未收到')

        # 5. apriltag ↔ 相机接上了 (空数组也算, 见 _make_detections_sub)。
        if not self._detections_seen:
            missing.append('detections 无流量')

        # 6. 相机外参可解 —— 这一项是上面那条竞态的**直接对应物**:
        #    查的就是 docking_node.py:759-761 会查的那对 frame。前五项证明的是
        #    各自的发现完成了, 这一项才真正回答"外参到底在不在 TF 里"。
        if tf_buf is not None and not self._tf_ok(tf_buf):
            missing.append('相机外参 TF 未就绪')
        return missing

    def _tf_ok(self, tf_buf) -> bool:
        try:
            # timeout=0: 只问"缓冲区里现在有没有", 绝不阻塞等待。本机在 815Hz
            # 的 /tf 洪水下有过回调内带 timeout 的 lookup 自锁的先例。
            return tf_buf.can_transform(
                self._base_frame, self._camera_frame,
                Time(), Duration(seconds=0))
        except Exception:                              # noqa: BLE001
            return False

    # ── 转发 ────────────────────────────────────────────────────────

    def _call_downstream(self, which: str, timeout: float = 5.0) -> dict:
        """调 docking_node 的一个 Trigger 服务。

        call_async + 轮询 future, **绝不在回调里阻塞等待** —— 本机有过 /tf 洪水
        下回调内阻塞 lookup 把自己锁死的先例。本方法跑在服务回调线程里, 而
        executor 是多线程的, 所以轮询期间订阅回调照常在别的线程上跑 (就绪门
        依赖这一点)。
        """
        cli = self._cli[which]
        if not cli.service_is_ready():
            return {'success': False, 'message': f'{cli.srv_name} 不可用'}
        future = cli.call_async(Trigger.Request())
        deadline = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() > deadline:
                return {'success': False, 'message': f'调用 {which} 超时'}
            time.sleep(0.02)
        try:
            res = future.result()
        except Exception as exc:                       # noqa: BLE001
            return {'success': False, 'message': f'调用 {which} 异常: {exc}'}
        return {'success': bool(res.success), 'message': res.message}

    def _ensure_full_and_forward(self, which: str) -> dict:
        """确保满栈 → 等就绪 → 转发。dock 和 undock 共用这一条路径。"""
        try:
            with self._busy_guard(which):
                return self._ensure_full_and_forward_locked(which)
        except RuntimeError as exc:
            return {'success': False, 'message': str(exc)}

    def _ensure_full_and_forward_locked(self, which: str) -> dict:
        with self._lock:
            need_spawn = (self._stack == 'down' or self._mode != 'full')
            hot = (self._stack == 'ready' and self._mode == 'full')

        if hot:
            # 热栈复用: 上一次停泊刚结束、还没到收栈时刻。直接转发, 零冷启动。
            # 这正是"终态后延迟 N 秒"这个决策要买的东西。
            self.get_logger().info(f'{which}: 复用热栈')
        else:
            if need_spawn and self._stack != 'down':
                # camera → full 只能收了重起: 跑起来的 launch 加不了节点。
                # 代价是视频黑屏几秒 (RTSP 要重连), 页面上有明说。
                self.get_logger().info('camera 模式不够用, 收掉重起满栈')
                self._teardown()
            self._spawn('all')
            ok, why = self._wait_ready()
            if not ok:
                # 不留半死的栈。
                self._teardown()
                return {'success': False, 'message': f'栈未能就绪: {why}'}

        with self._lock:
            self._stack = 'ready'
            self._mode = 'full'
            self._holds.add('docking')
            self._release_at = None          # 活动期间不计时
            self._detail = f'已转发 {which}'

        res = self._call_downstream(which, timeout=8.0)
        if not res['success']:
            # 转发被拒 (比如 'already active')。栈是好的, 别收 —— 挂上延迟
            # 计时让它自然过期即可。
            with self._lock:
                self._release_at = time.monotonic() + self._idle_delay
                self._detail = f'{which} 被拒: {res["message"]}'
            return res

        with self._lock:
            # 边沿触发的关键一步: 转发成功后先标记"还没见过活动态"。
            # 终态会 latch, 热栈/接管栈上 docked 这类值可能一直挂着; 不先要求
            # 见到一次活动态就直接看终态, 会把**上一次**的终态当成这一次的结束,
            # 立刻起收栈计时 —— 机器人可能才刚要动。
            self._awaiting_active = True
            self._forward_mono = time.monotonic()
        return res

    # ── 服务回调 ────────────────────────────────────────────────────

    def _srv_dock(self, _req, res):
        out = self._ensure_full_and_forward('dock')
        res.success, res.message = out['success'], out['message']
        return res

    def _srv_undock(self, _req, res):
        out = self._ensure_full_and_forward('undock')
        res.success, res.message = out['success'], out['message']
        return res

    def _srv_cancel(self, _req, res):
        with self._lock:
            down = self._stack == 'down'
        if down:
            # 栈没起就没什么可取消的。返回 success 而不是报错: 调用者的意图是
            # "确保它没在动", 这个意图已经满足了。
            res.success, res.message = True, '栈未运行, 无需取消'
            return res
        out = self._call_downstream('cancel')
        res.success, res.message = out['success'], out['message']
        return res

    def _srv_video_hold(self, req, res):
        """取/退一个 video 占用。**这是租约, 不是开关。**

        取占用只把到期时刻推后 VIDEO_LEASE_TTL_SEC, 持有方 (web) 要在 TTL 内
        一直续。为什么不做成一次性开关: 那样一来 web 被 kill -9、掉电、或浏览器
        连接半开时, 那个 video_hold(False) 就永远发不出去, 相机链路被钉死在
        那儿常驻 —— 而不常驻正是这整件事的目的。租约把"没人续了"和"没人要了"
        变成同一件事, 不需要任何一侧可靠地说出最后那句话。
        """
        with self._lock:
            if req.data:
                self._holds.add('video')
                self._video_lease_until = time.monotonic() + VIDEO_LEASE_TTL_SEC
            else:
                self._holds.discard('video')
                self._video_lease_until = None
            holds = sorted(self._holds)
        res.success = True
        res.message = f'占用: {holds or "无"}'
        return res

    def _srv_ready_check(self, _req, res):
        """就绪门当前的判定结果 —— 只读, 不起栈、不转发、不让机器人动。

        存在的理由有两个。一是排查: 停泊起不来时想知道卡在哪一项, 不用翻日志猜。
        二是**验证这道门本身**: 门的作用是拦住会踩 TF 竞态的那几秒, 而要确认它
        真的会拦住/放行, 唯一安全的办法就是能在不触发停泊的前提下读到它的判定 ——
        否则每验证一次这道门, 就得让机器人真动一次。
        """
        tf_buf, probe = self._make_probe()
        try:
            # 新建的探针要留点时间收数据。TF 那条尤其要紧: /tf_static 是
            # transient-local (latched), 订阅建立后那条消息是**异步**送到的 ——
            # 实测建好 0ms 查是 false, 100ms 起就是 true。不留这个窗口, 这个诊断
            # 会对着一个明明健康的栈报"相机外参 TF 未就绪", 把人往错方向带。
            # camera_info / odom / detections 现在也挂在探针上 (见 _make_probe),
            # 所以这里等的是**六项全体**而不只是 TF —— 只等 TF 的话, 另外三项会
            # 因为订阅刚建好还没收到消息而被误报成缺失。
            # (_wait_ready 那边不需要这个窗口: 它的探针活过整个轮询循环。)
            deadline = time.monotonic() + TF_PROBE_SETTLE_SEC
            missing = self._ready_missing(tf_buf)
            while missing and time.monotonic() < deadline:
                time.sleep(0.05)
                missing = self._ready_missing(tf_buf)
        finally:
            self._drop_probe(probe)
        res.success = not missing
        res.message = '就绪, 可以转发 start_docking' if not missing \
            else '仍缺: ' + ', '.join(missing)
        return res

    def _srv_stack_down(self, _req, res):
        """人工立即收栈, 跳过延迟。排查用。"""
        with self._lock:
            self._holds.clear()
            self._release_at = None
        try:
            with self._busy_guard('stack_down'):
                self._teardown()
        except RuntimeError as exc:
            res.success, res.message = False, str(exc)
            return res
        res.success, res.message = True, '已收栈'
        return res

    # ── 主循环 ──────────────────────────────────────────────────────

    def _tick(self):
        try:
            self._evaluate()
        except Exception as exc:                       # noqa: BLE001
            self.get_logger().error(f'tick 异常: {exc}')
        self._publish_status()

    def _evaluate(self):
        now = time.monotonic()

        with self._lock:
            # 已经有人在起栈/收栈就直接让开。服务回调 (dock/undock) 和本定时器
            # 都会动进程, 两边同时动同一棵树就是互相抢信号。
            if self._busy:
                return
            stack, mode = self._stack, self._mode
            owned, proc = self._owned_pid, self._proc

        # /proc 全扫限速, 见 PROC_SCAN_SEC。下面两项 (接管、按进程树判死) 都走它。
        scan_due = now - self._last_proc_scan >= PROC_SCAN_SEC
        if scan_due:
            self._last_proc_scan = now

        # ── 有没有"别人起的栈"要接管 ──
        # 不只在启动时接管一次: 手工 `ros2 launch tagdocking docking.launch.py`
        # 或 `systemctl start whale-nav-tagdocking` 可能发生在 supervisor 起来
        # **之后**。不定期接管的话, 那棵栈就成了没人管的常驻栈 —— 既不会被收,
        # 也不会计入占用, 而"省 CPU"这件事整个落空, 且从 status 上看不出来。
        if stack == 'down' and scan_due:
            if self._adopt_existing():
                return

        # ── 栈死了没 ──
        # 持有 pid 但进程没了, 或 state 断流太久: 标 down 并清占用, 不要卡在
        # ready 上装作一切正常。
        if stack in ('ready', 'starting'):
            dead = proc is not None and proc.poll() is not None
            if not dead and owned is None and scan_due:
                dead = not launch_tree(process_snapshot(), DOCKING_LAUNCH_CMD)
            if dead:
                self.get_logger().warn('栈进程已消失, 标记为 down')
                with self._lock:
                    self._proc = None
                    self._owned_pid = None
                    self._stack = 'down'
                    self._mode = 'none'
                    self._holds.discard('docking')
                    self._holds.discard('adopted')
                    self._release_at = None
                    self._detail = '栈进程意外退出'
                    self._reset_ready_signals()
                return

        # ── 终态判定 (边沿触发) ──
        if self._awaiting_active:
            if self._state in ACTIVE_STATES:
                with self._lock:
                    self._awaiting_active = False
            elif now - self._forward_mono > ACTIVE_WAIT_SEC:
                # 转发成功了却始终没动起来。这和"跑完了"是两回事, 分开报 ——
                # 混在一起会让人以为停泊正常结束了。
                self.get_logger().warn(
                    f'转发后 {ACTIVE_WAIT_SEC:.0f}s 未见活动态 (state={self._state})')
                with self._lock:
                    self._awaiting_active = False
                    self._release_at = now + self._idle_delay
                    self._detail = '转发后未见活动态'
        elif self._state in TERMINAL_STATES:
            with self._lock:
                if 'docking' in self._holds and self._release_at is None:
                    self._release_at = now + self._idle_delay
                    self._detail = f'{self._state}, {self._idle_delay:.0f}s 后收栈'
                    self.get_logger().info(
                        f'到达终态 {self._state}, {self._idle_delay:.0f}s 后释放占用')

        # ── 占用到期 ──
        with self._lock:
            expired = (self._release_at is not None and now >= self._release_at)
            if expired:
                self._holds.discard('docking')
                self._holds.discard('adopted')
                self._release_at = None
            # video 是租约: 持有方停止续租就自动掉。没有这一段, 上面那个"租约"
            # 就只是个名字 —— 取了不退等于开关, 而开关正是要避免的那个失败模式。
            if (self._video_lease_until is not None
                    and now >= self._video_lease_until):
                self._video_lease_until = None
                had_video = 'video' in self._holds
                self._holds.discard('video')
                if had_video:
                    self.get_logger().info(
                        'video 租约过期 (无人续租), 释放视频占用')
            holds = set(self._holds)
            # 活动态期间绝不收栈 —— 机器人正在动。状态一直不到终态就一直留着,
            # 这是有意的。
            busy = self._state in ACTIVE_STATES

        if busy:
            return

        want = 'full' if (holds - {'video'}) else ('camera' if holds else 'none')

        # 下面要动进程了, 占住标志。拿不到就下一 tick 再说 —— 定时器 0.5s 一跳,
        # 没什么可着急的。
        try:
            with self._busy_guard('自动调整'):
                if want == 'none' and stack != 'down':
                    self.get_logger().info('无人占用, 收栈')
                    self._teardown()
                elif want == 'camera' and stack == 'down':
                    self._spawn('camera')
                    with self._lock:
                        # 相机模式不走就绪门: 就绪门那六项是为"能不能安全开始
                        # 停泊"设计的, 看画面只需要有图。
                        self._stack = 'ready'
                elif want == 'camera' and mode == 'full' and stack == 'ready':
                    # 停泊结束、观看者还在: 降级回相机模式。满栈是相机的超集,
                    # 但跑起来的 launch 减不了节点, 只能收了重起 —— 视频会
                    # 短暂中断。
                    self.get_logger().info('停泊结束, 降级回相机模式')
                    self._teardown()
                    self._spawn('camera')
                    with self._lock:
                        self._stack = 'ready'
        except RuntimeError:
            return

    def _publish_status(self):
        with self._lock:
            payload = {
                'stack': self._stack,
                'mode': self._mode,
                'pid': self._owned_pid,
                'holds': sorted(self._holds),
                'detail': self._detail,
                'state': self._state,
                'release_in': (round(self._release_at - time.monotonic(), 1)
                               if self._release_at else None),
                # 租约还剩多久。放进 status 是为了让"视频占用被钉住了"这件事
                # 从外面看得见 —— 否则一个不该存在的 video 占用只表现为"栈一直
                # 不收", 那是最难查的那种。
                'video_lease_in': (
                    round(self._video_lease_until - time.monotonic(), 1)
                    if self._video_lease_until else None),
            }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self._status_pub.publish(msg)

    # ── 退出 ────────────────────────────────────────────────────────

    def shutdown(self):
        """supervisor 自己要退了 —— 把栈也体面收掉。

        **这是必须的, 不是锦上添花**: 子进程用 start_new_session=True 起在独立
        会话里, 不在 supervisor 的 cgroup 信号范围内。systemd 的 KillMode 无论
        选哪个都救不了 —— `process` 只发信号给主进程, 整棵栈原地变孤儿;
        `control-group` 的最后一发 SIGKILL 又会绕过 docking_node 那 1 秒的
        刷零速度。所以只能由 supervisor 自己在退出路径上走一遍有序收栈。
        """
        self.get_logger().info('supervisor 退出, 收栈')
        # 有别的操作正在动进程就稍等一下, 但**不等到底**: 退出路径上"没收干净"
        # 比"多等几秒"严重得多, 等不到就硬上。此时 executor 已经 shutdown,
        # 不会再有新的 tick 进来。
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            with self._lock:
                if not self._busy:
                    break
            time.sleep(0.1)
        with self._lock:
            self._busy = False
        try:
            self._teardown()
        except Exception as exc:                       # noqa: BLE001
            self.get_logger().error(f'退出时收栈失败: {exc}')


def main(argv=None):
    parser = argparse.ArgumentParser(description='tagdocking 停泊栈按需启停')
    parser.add_argument('--ready-timeout', type=float, default=READY_TIMEOUT_SEC)
    parser.add_argument('--idle-delay', type=float, default=IDLE_STOP_DELAY_SEC)
    parser.add_argument('--odom-topic', default='/odin1/odometry_highfreq')
    # ros2 run 会塞 --ros-args …; 用 parse_known_args 忽略。
    args, _ = parser.parse_known_args(argv)

    rclpy.init(args=None)
    node = StackSupervisor(args)

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    stopping = threading.Event()

    def _on_signal(signum, _frame):
        # 只置事件, 收栈放到主线程做。收栈要发信号、要等进程退、要调服务,
        # 在信号处理器里干这些事是自找麻烦。
        if not stopping.is_set():
            stopping.set()
            executor.shutdown()

    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    try:
        executor.spin()
    except KeyboardInterrupt:
        stopping.set()
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
