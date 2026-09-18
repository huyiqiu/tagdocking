#!/usr/bin/env python3
"""tagdocking Web 控制台 —— 视频 + 状态 + 停泊/泊出/取消。

只读 ROS 话题、只调 ROS 服务: 本模块**自己**既不 spawn 也不发信号。进程生命周期
归 docking_supervisor —— 停泊/视频请求都转给它, 由它决定起栈、收栈。

为什么按需启停不做在这里 (这一层划分是有意的, 别把它合回来):

* ROS 入口要在栈没起时也能用。外部平台可能直接
  `ros2 service call /docking_supervisor/dock`, 不经过网页 —— 那条路不能依赖
  一个 HTTP 服务活着。
* web 是 `Restart=always`。改个静态文件、uvicorn 抽一下都会重启它, 而重启一个
  正在停泊的栈是不能接受的。栈的命归一个不会随 web 起落的节点管。

本模块不参与停泊决策: 停泊逻辑在 docking_node 里, 这里一行都不碰。

    ros2 run tagdocking docking_web --port 8090

设计要点:

* 图像订阅是惰性的。没人看网页时不订阅 /camera_sync/image_raw, 免得在 Jetson 上
  白白吃一路 JPEG 编码的 CPU。最后一个观看者断开后退订。
* 有观看者时向 supervisor 续租一个 video 占用, 让它把相机链路拉起来 (只相机,
  不起 apriltag/docking_node)。**租约式**而不是一次性开关: 见
  VIDEO_LEASE_RENEW_SEC。
* 服务调用一律 call_async + 在线程池里等 future。绝不在 ROS 回调里阻塞等待 ——
  本机有过 /tf 洪水下回调内阻塞 lookup 把自己锁死的先例。
* 超过 STALE_SEC 没收到 state 就判 docking_node 离线, 页面显式变灰, 而不是
  停在最后一个状态上装作一切正常。
"""

from __future__ import annotations

import argparse
import asyncio
import json
import os
import threading
import time
from typing import Optional

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

import cv2
from cv_bridge import CvBridge

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse, StreamingResponse
import uvicorn


# 超过这么久没收到 state 就认为 docking_node 掉了。
# 注意 state **每个 tick 都发** (docking_node._control_loop 20Hz, 无提前 return),
# 包括停在 idle 时 —— 所以"没有新消息"就是真的没人在发, 这个判据是可靠的。
# (这里原先注释成"只在状态变化时发布", 是错的。这条错误正好是会让人把就绪判定
#  设计错的那种: 若以为 idle 不发消息, 就会转去用别的信号猜"控制循环在不在转"。)
STALE_SEC = 5.0

STATE_TOPIC = '/docking_node/state'
ERROR_TOPIC = '/docking_node/error'
# 每轮停泊结束发一次的结果 JSON (锁存, 含失败码与原因)。直接订 docking_node,
# 不绕 supervisor —— 单次停泊的遥测 (state/error) 本来就是节点直连,
# 只有进程/栈的事实才走 supervisor。收栈后节点的锁存会没, 但本进程常驻,
# 上一条结果留在内存里, 页面照样显示得出。
OUTCOME_TOPIC = '/docking_node/outcome'
IMAGE_TOPIC = '/camera_sync/image_raw'
SUP_STATUS_TOPIC = '/docking_supervisor/status'

# 三个按钮打给 supervisor, 不再直接打 docking_node —— 按需启停之后 docking_node
# 平时根本不存在, 直接打它就是永远"服务不可用"。supervisor 负责确保栈起来、
# 等就绪门放行, 再把请求转下去。
SRV_DOCK = '/docking_supervisor/dock'
SRV_UNDOCK = '/docking_supervisor/undock'
SRV_CANCEL = '/docking_supervisor/cancel'
SRV_VIDEO_HOLD = '/docking_supervisor/video_hold'

# dock/undock 里含冷启动 (起栈 + 走完六项就绪门, 实测 5~8s; camera→满栈还要先
# 收掉相机栈再起, 更久)。5 秒的老超时会在栈没起时必然超时, 而服务其实还在正常
# 推进, 页面上看起来就是"点了没反应"。45 秒给足余量, 且仍大于就绪门自己的 20s
# 超时 + 收栈 8s 宽限之和。
TRIGGER_TIMEOUT_SEC = 45.0

# video 占用是**租约**, 不是开关: 有观看者就每 5 秒续一次, supervisor 侧过期即
# 自动释放。理由是失败模式 —— 一次性开关的话, web 被 kill -9 / 掉电 / 网页连接
# 半开, 那个 video_hold(False) 就永远发不出去, 相机链路被钉死在那里常驻, 而这
# 整件事本来就是为了不常驻。租约的代价只是多几次很便宜的服务调用。
VIDEO_LEASE_RENEW_SEC = 5.0
# 观看者计数的轮询间隔。续租是 5 秒一次, 但 0→1 这个沿要快点抓到, 否则第一个
# 观看者要干等 5 秒才开始起相机。整数比较, 开销可忽略。
HOLD_POLL_SEC = 0.5

# 视频是"开/关"而不是"调档": 开着就按 tagdocking 自己看到的原样推 ——
# 不缩放、不丢帧、JPEG 质量拉满。省资源靠的是关掉它 (关掉即退订, 零开销),
# 而不是把画面压糊 —— 压糊只是让人看不清, 检测器那一份根本不受影响, 省下的
# 那点编码开销换来的是排查时看不出问题。
#
# 唯一与检测器输入不同的地方: MJPEG 必须是 JPEG, 而检测器拿的是原始 BGR。
# 95 已接近视觉无损, 但严格说仍是有损的。分辨率和帧率则完全一致。
STREAM_JPEG_QUALITY = 95
# 轮询新帧的间隔。相机帧间隔 33~100ms, 5ms 轮询带来的额外延迟可忽略, 开销
# 也只是一次加锁 + 整数比较。用轮询而不是跨线程 Event: _on_image 跑在 ROS
# 执行器线程、推流跑在 asyncio 事件循环, 中间要 call_soon_threadsafe 转一道,
# 那处的生命周期管理正是容易出错的地方, 不值得为 5ms 去换。
STREAM_POLL_SEC = 0.005


class DockingWebNode(Node):
    """ROS 侧: 订阅状态/误差/图像, 持有三个 Trigger 客户端。"""

    def __init__(self, image_topic: str = IMAGE_TOPIC):
        super().__init__('docking_web')

        self._image_topic = image_topic
        self._bridge = CvBridge()
        self._cbg = ReentrantCallbackGroup()

        # ── 共享状态 (GIL 下单次赋值原子, 读侧只读不改, 无需加锁) ──
        self._state: Optional[str] = None
        self._state_mono: float = 0.0
        self._error = (0.0, 0.0, 0.0)
        self._error_mono: float = 0.0
        # 最后一次停泊结果 (dict), 解不开就保持 None。
        self._outcome: Optional[dict] = None
        self._outcome_mono: float = 0.0

        # ── 图像: 只存最新一帧, 慢的观看者自然丢帧而不是堆积内存 ──
        self._frame_lock = threading.Lock()
        self._frame = None            # 最近一帧 BGR ndarray
        self._frame_seq = 0           # 帧序号, 观看者据此判断是否有新帧
        self._frame_mono = 0.0
        self._fps = 0.0
        self._fps_count = 0
        self._fps_mono = time.monotonic()

        # ── 惰性图像订阅 ──
        self._viewers = 0
        self._viewer_lock = threading.Lock()
        self._img_sub = None

        # ── supervisor 状态 (栈起没起、什么模式) ──
        self._sup_status: Optional[dict] = None
        self._sup_mono: float = 0.0

        self.create_subscription(
            String, STATE_TOPIC, self._on_state, 10, callback_group=self._cbg)
        self.create_subscription(
            Vector3, ERROR_TOPIC, self._on_error, 10, callback_group=self._cbg)
        self.create_subscription(
            String, SUP_STATUS_TOPIC, self._on_sup_status, 10,
            callback_group=self._cbg)
        # QoS 要与发布端的锁存对上 (transient_local/depth 1), 否则本进程
        # 先起来时收不到那次锁存值。
        self.create_subscription(
            String, OUTCOME_TOPIC, self._on_outcome,
            QoSProfile(depth=1,
                       reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL),
            callback_group=self._cbg)

        self._cli = {
            'dock': self.create_client(
                Trigger, SRV_DOCK, callback_group=self._cbg),
            'undock': self.create_client(
                Trigger, SRV_UNDOCK, callback_group=self._cbg),
            'cancel': self.create_client(
                Trigger, SRV_CANCEL, callback_group=self._cbg),
        }
        self._hold_cli = self.create_client(
            SetBool, SRV_VIDEO_HOLD, callback_group=self._cbg)

        # ── video 租约线程 ──
        # 为什么是独立线程, 而不是在 acquire_viewer/release_viewer 里直接调:
        # 那两个函数都持着 _viewer_lock, 而 release_viewer 跑在 MJPEG 生成器的
        # finally 里。在持锁状态下做一次可能要等好几秒的服务调用, 正是本机那次
        # /tf 洪水自锁的形状 —— 观看者断开时卡在锁上, 新观看者进不来, 页面看着
        # 像挂了。所以这里只读一个整数计数, 服务调用全在锁外、也全在 ROS 回调外。
        self._hold_stop = threading.Event()
        self._hold_thread = threading.Thread(
            target=self._hold_loop, daemon=True, name='video-lease')
        self._hold_thread.start()

        self.get_logger().info(
            f'docking_web 已就绪: state={STATE_TOPIC} image={self._image_topic}')

    # ── 订阅回调 ────────────────────────────────────────────────────

    def _on_state(self, msg: String):
        self._state = msg.data
        self._state_mono = time.monotonic()

    def _on_error(self, msg: Vector3):
        self._error = (msg.x, msg.y, msg.z)
        self._error_mono = time.monotonic()

    def _on_outcome(self, msg: String):
        try:
            parsed = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        if not isinstance(parsed, dict):
            return
        self._outcome = parsed
        self._outcome_mono = time.monotonic()

    def _on_sup_status(self, msg: String):
        try:
            self._sup_status = json.loads(msg.data)
        except (ValueError, TypeError):
            # 解不开就当没收到 —— 页面会显示"supervisor 无状态", 好过塞一坨
            # 半截 JSON 上去。
            return
        self._sup_mono = time.monotonic()

    def _on_image(self, msg: Image):
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:                      # noqa: BLE001
            # 编码不认识时不要把回调打崩 —— 掉一帧远好过掉整个订阅。
            self.get_logger().warn(f'图像转换失败: {exc}', throttle_duration_sec=5.0)
            return
        with self._frame_lock:
            self._frame = frame
            self._frame_seq += 1
            self._frame_mono = time.monotonic()
        self._fps_count += 1
        now = time.monotonic()
        span = now - self._fps_mono
        if span >= 1.0:
            self._fps = self._fps_count / span
            self._fps_count = 0
            self._fps_mono = now

    # ── 惰性订阅管理 ────────────────────────────────────────────────

    def acquire_viewer(self):
        """观看者进入; 第一个进入时才真正建立图像订阅。

        这里**只**动本地订阅, 不调 supervisor —— video 占用由租约线程按
        `_viewers` 这个计数自己跟随 (见 _hold_loop)。别把服务调用挪进来:
        本函数持着 _viewer_lock。
        """
        with self._viewer_lock:
            self._viewers += 1
            if self._img_sub is None:
                # 必须与发布端匹配: rtsp_camera 是 RELIABLE depth=10。
                # 用 sensor_data (BEST_EFFORT) 会收不到。
                qos = QoSProfile(depth=10,
                                 reliability=ReliabilityPolicy.RELIABLE)
                self._img_sub = self.create_subscription(
                    Image, self._image_topic, self._on_image, qos,
                    callback_group=self._cbg)
                self.get_logger().info(
                    f'首个观看者接入, 订阅 {self._image_topic}')

    def release_viewer(self):
        """观看者离开; 最后一个离开后退订, 空闲时不吃 CPU。

        同 acquire_viewer: 不在这里调 supervisor。本函数跑在 MJPEG 生成器的
        finally 里并且持着 _viewer_lock —— 在这两个条件下做一次可能要等好几秒的
        服务调用是本机踩过的死锁形状。
        """
        with self._viewer_lock:
            self._viewers = max(0, self._viewers - 1)
            if self._viewers == 0 and self._img_sub is not None:
                self.destroy_subscription(self._img_sub)
                self._img_sub = None
                with self._frame_lock:
                    self._frame = None
                self._fps = 0.0
                self.get_logger().info('最后一个观看者离开, 已退订图像')

    # ── video 租约 ──────────────────────────────────────────────────

    def _hold_loop(self):
        """有观看者就向 supervisor 续租 video 占用, 没观看者就退租。

        续租而不是一次性开关, 见 VIDEO_LEASE_RENEW_SEC 的注释。这个循环故意写得
        很笨: 只读一个整数, 没有任何跨线程状态机 —— 它的正确性不能依赖"退租那一
        次调用一定送到"。
        """
        held = False
        last_renew = 0.0
        while not self._hold_stop.wait(HOLD_POLL_SEC):
            want = self._viewers > 0
            now = time.monotonic()
            try:
                if want and (not held or now - last_renew >= VIDEO_LEASE_RENEW_SEC):
                    out = self._call_hold(True)
                    if out['success']:
                        if not held:
                            self.get_logger().info('已向 supervisor 取得视频占用')
                        held, last_renew = True, now
                elif not want and held:
                    self._call_hold(False)
                    # 无论成败都置 False: 失败也没关系, supervisor 侧的租约会
                    # 自己过期。反过来 (失败就一直重试) 才是问题 —— 那会在
                    # supervisor 不在时每 0.5 秒打一次不存在的服务。
                    held = False
                    self.get_logger().info('已释放视频占用')
            except Exception as exc:                  # noqa: BLE001
                self.get_logger().warn(f'视频占用续租异常: {exc}',
                                       throttle_duration_sec=10.0)

    def _call_hold(self, want: bool) -> dict:
        """调 supervisor 的 video_hold。只在租约线程里调。"""
        if not self._hold_cli.service_is_ready():
            return {'success': False, 'message': 'supervisor 不在线'}
        req = SetBool.Request()
        req.data = bool(want)
        future = self._hold_cli.call_async(req)
        # 这个调用很便宜 (supervisor 那边只动一个 set), 5 秒足够; 真起相机栈是
        # 它自己的定时器在后台做的, 不在这次调用里。
        deadline = time.monotonic() + 5.0
        while not future.done():
            if time.monotonic() > deadline:
                return {'success': False, 'message': 'video_hold 超时'}
            time.sleep(0.02)
        try:
            res = future.result()
        except Exception as exc:                      # noqa: BLE001
            return {'success': False, 'message': f'video_hold 异常: {exc}'}
        return {'success': bool(res.success), 'message': res.message}

    # ── 供 HTTP 侧读取 ──────────────────────────────────────────────

    def snapshot(self) -> dict:
        now = time.monotonic()
        # online 判的是 **supervisor**, 不是 docking_node。按需启停之后
        # docking_node 平时根本不存在, 照旧判它就是所有按钮永久禁用 —— 而栈没起
        # 正是最需要能点"停泊"的时候 (点它就是让 supervisor 去起栈)。
        online = any(c.service_is_ready() for c in self._cli.values())
        sup = self._sup_status
        sup_fresh = bool(self._sup_mono and now - self._sup_mono <= STALE_SEC)
        ex, ey, eyaw = self._error
        return {
            'online': online,
            'state': self._state,
            'state_age': (now - self._state_mono) if self._state_mono else None,
            'stale': bool(self._state_mono and now - self._state_mono > STALE_SEC),
            'error': {'x': ex, 'y': ey, 'yaw': eyaw},
            'error_age': (now - self._error_mono) if self._error_mono else None,
            'viewers': self._viewers,
            'fps': round(self._fps, 1),
            'image_topic': self._image_topic,
            # 栈的状态单独一档, 与 online 分开: online=true / stack=down 是完全
            # 正常的常态 (supervisor 在, 栈按需不起), 页面要能把这两件事说清。
            'stack': (sup.get('stack') if (sup and sup_fresh) else None),
            'stack_mode': (sup.get('mode') if (sup and sup_fresh) else None),
            'stack_detail': (sup.get('detail') if (sup and sup_fresh) else None),
            'stack_holds': (sup.get('holds') if (sup and sup_fresh) else None),
            'release_in': (sup.get('release_in') if (sup and sup_fresh) else None),
            # 这一轮的结果 (成功/失败 + 码 + 原因), 整块透传给前端。**不判**
            # 新鲜度: 它是锁存的边沿值, 年龄一直涨, 用 STALE_SEC 判会把
            # 刚失败 6 秒的结果当成过期的丢掉。前端按 state 是否对得上来决定
            # 显不显示。
            'outcome': self._outcome,
        }

    def latest_frame(self):
        with self._frame_lock:
            if self._frame is None:
                return None, self._frame_seq
            return self._frame, self._frame_seq

    def shutdown_holds(self):
        """停掉租约线程并退掉 video 占用。进程退出路径上调一次。"""
        self._hold_stop.set()
        self._hold_thread.join(timeout=2.0)
        try:
            self._call_hold(False)
        except Exception:                             # noqa: BLE001
            # 退出路径上失败无所谓: supervisor 侧的租约会自己过期。
            pass

    def call_trigger(self, which: str,
                     timeout: float = TRIGGER_TIMEOUT_SEC) -> dict:
        """调用一个 Trigger 服务并等结果。只在线程池里调, 不在 ROS 回调里调。"""
        cli = self._cli[which]
        if not cli.service_is_ready():
            return {'success': False,
                    'message': f'服务 {cli.srv_name} 不可用 '
                               f'(docking_supervisor 未运行?)'}
        future = cli.call_async(Trigger.Request())
        deadline = time.monotonic() + timeout
        # executor 在另一个线程里转, 这里只轮询 future, 不夺取 spin。
        while not future.done():
            if time.monotonic() > deadline:
                return {'success': False, 'message': f'调用 {which} 超时'}
            time.sleep(0.02)
        try:
            res = future.result()
        except Exception as exc:                      # noqa: BLE001
            return {'success': False, 'message': f'调用 {which} 异常: {exc}'}
        return {'success': bool(res.success), 'message': res.message}


# ── HTTP 应用 ───────────────────────────────────────────────────────

def build_app(node: DockingWebNode) -> FastAPI:
    app = FastAPI(title='tagdocking console')
    static_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'static')

    @app.get('/')
    async def index():
        return FileResponse(os.path.join(static_dir, 'index.html'))

    @app.get('/api/status')
    async def status():
        return JSONResponse(node.snapshot())

    @app.websocket('/ws')
    async def ws(sock: WebSocket):
        await sock.accept()
        try:
            while True:
                await sock.send_text(json.dumps(node.snapshot()))
                await asyncio.sleep(0.2)          # 5Hz 够用, 状态不是高频量
        except WebSocketDisconnect:
            pass
        except Exception:                          # noqa: BLE001
            pass

    @app.get('/api/video')
    async def video():
        # 开关式: 没有画质/缩放/帧率参数。连上就是原分辨率、原帧率, 断开即退订。
        async def gen():
            node.acquire_viewer()
            last_seq = -1
            try:
                while True:
                    frame, seq = node.latest_frame()
                    if frame is not None and seq != last_seq:
                        last_seq = seq
                        ok, buf = await asyncio.get_running_loop().run_in_executor(
                            None,
                            lambda f=frame: cv2.imencode(
                                '.jpg', f,
                                [int(cv2.IMWRITE_JPEG_QUALITY),
                                 STREAM_JPEG_QUALITY]))
                        if ok:
                            jpg = buf.tobytes()
                            yield (b'--frame\r\nContent-Type: image/jpeg\r\n'
                                   b'Content-Length: ' +
                                   str(len(jpg)).encode() + b'\r\n\r\n' +
                                   jpg + b'\r\n')
                    await asyncio.sleep(STREAM_POLL_SEC)
            finally:
                # 客户端断开 / 服务器关闭都会走到这里, 保证订阅计数不泄漏。
                node.release_viewer()

        return StreamingResponse(
            gen(), media_type='multipart/x-mixed-replace; boundary=frame')

    async def _trigger(which: str):
        loop = asyncio.get_running_loop()
        result = await loop.run_in_executor(None, node.call_trigger, which)
        return JSONResponse(result)

    @app.post('/api/dock')
    async def dock():
        return await _trigger('dock')

    @app.post('/api/undock')
    async def undock():
        return await _trigger('undock')

    @app.post('/api/cancel')
    async def cancel():
        return await _trigger('cancel')

    return app


def main(argv=None):
    parser = argparse.ArgumentParser(description='tagdocking web 控制台')
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=8090)
    parser.add_argument('--image-topic', default=IMAGE_TOPIC,
                        help='用于网页预览的图像话题 (默认是 apriltag 实际检测的那一路)')
    # ros2 run 会塞 --ros-args ...; 用 parse_known_args 忽略它们。
    args, _ = parser.parse_known_args(argv)

    rclpy.init(args=None)
    node = DockingWebNode(image_topic=args.image_topic)

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    app = build_app(node)
    try:
        uvicorn.run(app, host=args.host, port=args.port, log_level='warning')
    except KeyboardInterrupt:
        pass
    finally:
        # 退出前主动退租一次, 别让 supervisor 白等一个租约周期。租约本身是兜底,
        # 这里是礼貌: 正常退出时相机链路当场就能收掉。
        node.shutdown_holds()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
