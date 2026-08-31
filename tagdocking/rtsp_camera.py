"""RTSP → ROS2 相机桥 — 给无 ROS 相机驱动的机器人(如机器狗)合成图像话题。

背景
----
机器狗的摄像头只提供 RTSP 视频流, 机上没有 ROS 相机驱动, 因此缺三样东西:
  1. 没有 sensor_msgs/Image 话题可订阅;
  2. 没有 camera_info 发布者(AprilTag 解算 PnP 必需相机内参);
  3. 没有 base_link→相机坐标系 的 TF(docking_node 按 base_link 系测量)。

本节点一次补齐:
  - 后台线程用 OpenCV 拉 RTSP 流(FFmpeg 或 GStreamer 后端, 见
    capture_backend 参数), 每帧以**到达时刻**打戳,
    发布 Image 到 image_out_topic (默认 /camera_sync/image_raw);
  - 内参来自 YAML 文件(camera_info_file, 由 scripts/calibrate_rtsp 生成)
    或内联参数, 合成 CameraInfo(与 Image 同一时间戳, 逐帧配对)发布到
    camera_info_out_topic;
  - 发布静态 TF base_frame→frame_id = 相机安装位姿 × 光学系旋转。

下游管线与差速车完全一致, 不需要任何改动:

    rtsp_camera ─→ /camera_sync/image_raw + /camera_sync/camera_info
                ─→ apriltag_node ─→ /detections + TF ─→ docking_node ─→ /cmd_vel

(rtsp 模式下不再需要 camera_info_bridge: 本节点自己产出的 image 与
camera_info 时间戳天然逐帧一致, 且内含降采样, 少一跳大帧 DDS 转发。)

时间戳语义
----------
帧打的是"到达本进程"的时刻。RTSP 传输+解码延迟(典型 0.1~0.5s)意味着画面
内容比时间戳略"旧"。停靠系统是走停式(stop-and-go): 每次机动后 settle 窗口
(stopgo.turn_settle_sec, 默认 0.8s)等图像稳定才重新测量, 天然容忍该延迟;
若狗上实测发现停稳后读到的位姿仍"滞后", 适当调大 turn_settle_sec 即可。

用法
----
    # 先标定(见 scripts/calibrate_rtsp), 然后:
    ros2 run tagdocking rtsp_camera --ros-args \\
        -p rtsp_url:="rtsp://192.168.1.100:8554/live" \\
        -p camera_info_file:=/path/to/rtsp_camera_info.yaml

    # 或经 launch 一键起整套(见 launch/docking.launch.py):
    ros2 launch tagdocking docking.launch.py base_type:=omni \\
        rtsp_url:=rtsp://192.168.1.100:8554/live \\
        camera_info_file:=/path/to/rtsp_camera_info.yaml \\
        odom_topic:=/odom camera_mount_z:=0.35
"""

import math
import os
import re
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image

try:
    import cv2
except ImportError as exc:  # pragma: no cover
    raise SystemExit(
        'rtsp_camera 需要 OpenCV(含 FFmpeg 后端)。安装: '
        'sudo apt install python3-opencv') from exc

# RTSP 强制走 TCP: 相机/机器狗端常用 UDP, 无线丢包会造成花屏/解不出帧,
# 检测质量骤降。用户已显式设置此环境变量时不覆盖(可用它进一步调
# max_delay 等 ffmpeg 选项, 如 "rtsp_transport;tcp|max_delay;500000")。
os.environ.setdefault('OPENCV_FFMPEG_CAPTURE_OPTIONS', 'rtsp_transport;tcp')

# 光学系(x右, y下, z前) 相对 相机体系(x前, y左, z上, REP-103) 的旋转,
# 四元数 (x,y,z,w), 对应 R = [[0,0,1],[-1,0,0],[0,-1,0]]:
# 光学系 z 轴(正前) 对到 体系 +x(正前), x(右) 对到 -y, y(下) 对到 -z。
_Q_BODY_TO_OPTICAL = (-0.5, 0.5, -0.5, 0.5)


# ── 四元数小工具 (不依赖 scipy/tf_transformations) ──────────────────

def _quat_mul(a, b):
    """Hamilton 积。四元数格式 (x, y, z, w)。"""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _euler_zyx_quat(yaw, pitch, roll):
    """ZYX 欧拉角 (Rz(yaw)·Ry(pitch)·Rx(roll)) → 四元数 (x, y, z, w)。"""
    qz = (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))
    qy = (0.0, math.sin(pitch * 0.5), 0.0, math.cos(pitch * 0.5))
    qx = (math.sin(roll * 0.5), 0.0, 0.0, math.cos(roll * 0.5))
    return _quat_mul(_quat_mul(qz, qy), qx)


class RtspCameraNode(Node):
    """RTSP 视频流 → ROS2 Image + CameraInfo + 静态 TF。"""

    def __init__(self):
        super().__init__('rtsp_camera')

        self._declare_params()

        # ── 必要参数校验 ─────────────────────────────────────────
        self._url = str(self._p('rtsp_url')).strip()
        if not self._url:
            self._fatal('缺少 rtsp_url 参数 (如 rtsp://192.168.1.100:8554/live)')

        self._frame_id = str(self._p('frame_id'))
        self._reconnect_sec = float(self._p('reconnect_sec'))
        self._frame_timeout_sec = float(self._p('frame_timeout_sec'))
        self._max_fps = float(self._p('max_fps'))
        self._downscale = int(self._p('downscale'))
        self._target_width = int(self._p('target_width'))

        # ── 内参 (文件或内联参数) ────────────────────────────────
        self._intr = self._load_intrinsics()
        # (stream_w, stream_h) → 缩放后的内参数值缓存; 分辨率变化自动重算
        self._scaled_cache = {}
        # 流分辨率 → 降采样因子缓存 (downscale=0 时按 target_width 自动算)
        self._factor_cache = {}
        self._intr_scale_warned = False

        # ── 发布者 ───────────────────────────────────────────────
        # 与 camera_info_bridge 输出端一致: RELIABLE depth=10。
        # (实测 apriltag_ros 的 image_transport 订阅在本机对 BEST_EFFORT
        #  收不到 image, 见 camera_info_bridge 注释。)
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._img_pub = self.create_publisher(
            Image, str(self._p('image_out_topic')), qos)
        self._info_pub = self.create_publisher(
            CameraInfo, str(self._p('camera_info_out_topic')), qos)

        # ── 静态 TF: base_frame → 相机光学系 ─────────────────────
        if bool(self._p('publish_static_tf')):
            self._publish_mount_tf()

        # ── 统计 ─────────────────────────────────────────────────
        self._pub_count = 0
        self._last_frame_mono = 0.0
        self._reconnects = 0
        self._stream_wh = None
        self._last_stats_mono = time.monotonic()
        self._last_stats_count = 0
        self._stats_timer = self.create_timer(5.0, self._log_stats)

        # ── 采集线程 ─────────────────────────────────────────────
        self._stop = threading.Event()
        self._thread = threading.Thread(
            target=self._capture_loop, name='rtsp_capture', daemon=True)
        self._thread.start()

        rclpy.get_default_context().on_shutdown(self._stop.set)

        self.get_logger().info(
            f'RTSP 相机桥就绪: {self._url} → '
            f'{self._p("image_out_topic")} (frame={self._frame_id})')

    # ── 参数 ──────────────────────────────────────────────────────

    def _declare_params(self):
        """Declare all ROS2 parameters with defaults."""
        # 流
        self.declare_parameter('rtsp_url', '')
        self.declare_parameter('image_out_topic', '/camera_sync/image_raw')
        self.declare_parameter('camera_info_out_topic', '/camera_sync/camera_info')
        self.declare_parameter('frame_id', 'camera_color_optical_frame')
        # 拉流后端: ffmpeg (默认) | gstreamer。
        # Jetson 上 FFmpeg 后端对某些流(H.265/高码率)会解出冻结残帧
        # (画面卡住、检测永远失败), 换 gstreamer 走 nvv4l2decoder 硬解。
        self.declare_parameter('capture_backend', 'ffmpeg')
        self.declare_parameter('gst_latency', 200)  # rtspsrc 抖动缓冲 ms

        # 内参来源 1: YAML 文件 (推荐, scripts/calibrate_rtsp 生成)
        self.declare_parameter('camera_info_file', '')
        # 内参来源 2: 内联参数 (camera_info_file 为空时 width/height/fx/fy 必填;
        # cx/cy 缺省取画面中心; distortion 为逗号分隔字符串 "k1,k2,p1,p2,k3")
        self.declare_parameter('width', 0)
        self.declare_parameter('height', 0)
        self.declare_parameter('fx', 0.0)
        self.declare_parameter('fy', 0.0)
        self.declare_parameter('cx', 0.0)
        self.declare_parameter('cy', 0.0)
        self.declare_parameter('distortion', '0,0,0,0,0')
        self.declare_parameter('distortion_model', 'plumb_bob')

        # 输出降采样: 0=按 target_width 自动取整倍; >0=固定倍数。
        # 输出宽度控制在 ~640: 16cm tag @1m 仍有 ~50px 供检测, 且避免大帧
        # 打爆本机 DDS (大帧 RELIABLE 投递问题见 camera_info_bridge 注释)。
        self.declare_parameter('downscale', 0)
        self.declare_parameter('target_width', 640)

        self.declare_parameter('max_fps', 0.0)          # 0 = 不限流
        self.declare_parameter('reconnect_sec', 2.0)    # 断流重连间隔
        self.declare_parameter('frame_timeout_sec', 5.0)  # 无帧告警阈值

        # 静态 TF: base_frame → frame_id (相机安装位姿, 相机体系 x 前 y 左 z 上)
        self.declare_parameter('publish_static_tf', True)
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('mount.x', 0.0)          # m, base_link 系
        self.declare_parameter('mount.y', 0.0)
        self.declare_parameter('mount.z', 0.0)
        self.declare_parameter('mount.yaw_deg', 0.0)    # 相机朝向, 0=正前
        self.declare_parameter('mount.pitch_deg', 0.0)  # 正=低头俯视, 负=抬头仰视
        self.declare_parameter('mount.roll_deg', 0.0)

    def _p(self, name: str):
        return self.get_parameter(name).value

    def _fatal(self, msg: str):
        self.get_logger().error(msg)
        raise RuntimeError(msg)

    # ── 内参加载 ──────────────────────────────────────────────────

    def _load_intrinsics(self) -> dict:
        """读内参, 返回 {width, height, fx, fy, cx, cy, d, model}。"""
        path = str(self._p('camera_info_file')).strip()
        if path:
            if not os.path.isfile(path):
                self._fatal(
                    f'camera_info_file 不存在: {path}\n'
                    '  先标定: python3 scripts/calibrate_rtsp --url <rtsp地址> '
                    '--out config/rtsp_camera_info.yaml')
            import yaml
            try:
                with open(path, encoding='utf-8') as f:
                    data = yaml.safe_load(f)
            except Exception as exc:
                self._fatal(f'解析 camera_info_file 失败: {exc}')
            return self._parse_intrinsics_yaml(data, path)

        # 内联参数模式
        w = int(self._p('width'))
        h = int(self._p('height'))
        fx = float(self._p('fx'))
        fy = float(self._p('fy'))
        if w <= 0 or h <= 0 or fx <= 0.0 or fy <= 0.0:
            self._fatal(
                '缺少相机内参! 内参错误会导致测距完全不准。二选一:\n'
                '  1) camera_info_file 指向标定 YAML '
                '(先跑 scripts/calibrate_rtsp 标定, 推荐);\n'
                '  2) 内联参数 width/height/fx/fy (cx/cy/distortion 可选)。')
        cx = float(self._p('cx'))
        cy = float(self._p('cy'))
        if cx <= 0.0:
            cx = (w - 1) / 2.0
            self.get_logger().warn(f'cx 未提供, 取画面中心 {cx:.1f}')
        if cy <= 0.0:
            cy = (h - 1) / 2.0
            self.get_logger().warn(f'cy 未提供, 取画面中心 {cy:.1f}')
        d = self._parse_distortion(str(self._p('distortion')))
        return {'width': w, 'height': h, 'fx': fx, 'fy': fy,
                'cx': cx, 'cy': cy, 'd': d,
                'model': str(self._p('distortion_model'))}

    def _parse_intrinsics_yaml(self, data: dict, path: str) -> dict:
        """解析标定 YAML (scripts/calibrate_rtsp 的输出格式)。"""
        try:
            mat = data.get('camera_matrix') or data.get('K')
            if mat is None:
                raise ValueError('缺少 camera_matrix (或 K)')
            k = self._matrix_to_list(mat)
            fx, cx, fy, cy = k[0], k[2], k[4], k[5]
            d = data.get('distortion', data.get('D', []))
            if not isinstance(d, (list, tuple)):
                raise ValueError(f'distortion 应为列表, 实际 {type(d).__name__}')
            d = [float(v) for v in d]
            # width/height 可省略: 记 0, 首帧按流分辨率补上(视为同分辨率)
            w = int(data.get('width', 0) or 0)
            h = int(data.get('height', 0) or 0)
            if fx <= 0 or fy <= 0:
                raise ValueError(f'fx/fy 非法: fx={fx}, fy={fy}')
        except (ValueError, TypeError, KeyError) as exc:
            self._fatal(f'{path} 内参格式不对: {exc}\n'
                        '期望字段: width, height, camera_matrix (3x3), '
                        'distortion (列表)。用 scripts/calibrate_rtsp 重新生成。')
        model = str(data.get('distortion_model', 'plumb_bob'))
        return {'width': w, 'height': h, 'fx': fx, 'fy': fy,
                'cx': cx, 'cy': cy, 'd': d, 'model': model}

    @staticmethod
    def _matrix_to_list(m):
        """camera_matrix 兼容 3x3 嵌套 / 9 元素扁平 / {fx,fy,cx,cy} 字典。"""
        if isinstance(m, dict):
            return [float(m.get('fx', 0)), 0.0, float(m.get('cx', 0)),
                    0.0, float(m.get('fy', 0)), float(m.get('cy', 0)),
                    0.0, 0.0, 1.0]
        flat = []
        for row in m:
            if isinstance(row, (list, tuple)):
                flat.extend(float(v) for v in row)
            else:
                flat.append(float(row))
        if len(flat) != 9:
            raise ValueError(f'camera_matrix 应为 3x3 (9 个元素), 实际 {len(flat)} 个')
        return flat

    @staticmethod
    def _parse_distortion(s: str) -> list:
        return [float(v) for v in s.replace(' ', '').split(',') if v]

    # ── 分辨率适配 / 降采样 ────────────────────────────────────────

    def _factor_for(self, w: int) -> int:
        """该流分辨率下的降采样因子。downscale>0 固定; 否则按 target_width 自动。"""
        if w in self._factor_cache:
            return self._factor_cache[w]
        if self._downscale > 0:
            f = self._downscale
        else:
            f = max(1, math.ceil(w / max(self._target_width, 1)))
        self._factor_cache[w] = f
        return f

    def _scaled_intr(self, sw: int, sh: int) -> dict:
        """内参缩放到 (sw×sh 流分辨率 ÷ 降采样因子) 后的数值, 带缓存。

        内参标定分辨率与实际流不一致时(如标了主码流、拉了子码流)按比例缩放;
        畸变系数与分辨率无关, 不缩放。
        """
        f = self._factor_for(sw)
        key = (sw, sh)
        if key in self._scaled_cache:
            return self._scaled_cache[key]

        intr = dict(self._intr)
        if intr['width'] == 0:            # YAML 没写分辨率 → 视为流分辨率
            intr['width'], intr['height'] = sw, sh
        sx = sw / intr['width']
        sy = sh / intr['height']
        if abs(sx - 1.0) > 1e-6 or abs(sy - 1.0) > 1e-6:
            if not self._intr_scale_warned:
                self._intr_scale_warned = True
                self.get_logger().warn(
                    f'内参分辨率({intr["width"]}x{intr["height"]})与实际流'
                    f'({sw}x{sh})不一致, 已按比例缩放 (建议用相同流重新标定)')
        scaled = {'fx': intr['fx'] * sx / f, 'fy': intr['fy'] * sy / f,
                  'cx': intr['cx'] * sx / f, 'cy': intr['cy'] * sy / f,
                  'd': intr['d'], 'model': intr['model'],
                  'width': sw // f, 'height': sh // f}
        self._scaled_cache[key] = scaled
        return scaled

    # ── 静态 TF ────────────────────────────────────────────────────

    def _publish_mount_tf(self):
        """发布静态 TF base_frame→frame_id (安装位姿 × 光学系旋转)。

        mount.* 是相机**体系**(x前 y左 z上)在 base_link 下的安装位姿;
        frame_id 是图像**光学系**(x右 y下 z前, camera_info 内参所属系),
        两者相差固定旋转 _Q_BODY_TO_OPTICAL。总旋转 = R_mount · R_optical。
        """
        from geometry_msgs.msg import TransformStamped
        import tf2_ros

        yaw = math.radians(float(self._p('mount.yaw_deg')))
        pitch = math.radians(float(self._p('mount.pitch_deg')))
        roll = math.radians(float(self._p('mount.roll_deg')))
        qx, qy, qz, qw = _quat_mul(_euler_zyx_quat(yaw, pitch, roll),
                                   _Q_BODY_TO_OPTICAL)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = str(self._p('base_frame'))
        tf_msg.child_frame_id = self._frame_id
        tf_msg.transform.translation.x = float(self._p('mount.x'))
        tf_msg.transform.translation.y = float(self._p('mount.y'))
        tf_msg.transform.translation.z = float(self._p('mount.z'))
        tf_msg.transform.rotation.x = qx
        tf_msg.transform.rotation.y = qy
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw

        self._static_tf = tf2_ros.StaticTransformBroadcaster(self)
        self._static_tf.sendTransform(tf_msg)
        self.get_logger().info(
            f'静态TF: {tf_msg.header.frame_id} → {self._frame_id} | '
            f'安装位姿 x={self._p("mount.x")} y={self._p("mount.y")} '
            f'z={self._p("mount.z")} m, '
            f'yaw={self._p("mount.yaw_deg")} pitch={self._p("mount.pitch_deg")} '
            f'roll={self._p("mount.roll_deg")} deg')

    # ── 采集与发布 ─────────────────────────────────────────────────

    def _open_capture(self):
        """打开 RTSP 流, 失败返回 None。"""
        if str(self._p('capture_backend')) == 'gstreamer':
            return self._open_gstreamer()
        cap = cv2.VideoCapture(self._url, cv2.CAP_FFMPEG)
        # 对 FFMPEG 后端此设置多数版本被忽略, 但无害; 读线程逐帧消费,
        # 不积压即不涨延迟。
        try:
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception:
            pass
        if not cap.isOpened():
            cap.release()
            return None
        return cap

    def _open_gstreamer(self):
        """GStreamer 管线打开流: Jetson 硬解管线优先, 失败回退通用管线。

        Jetson: decodebin 自动选 nvv4l2decoder 硬解, nvvidconv 把 NVMM 帧转
        系统内存 BGRx; 非 Jetson 无 nvvidconv 插件, 回退纯 videoconvert
        (decodebin 走 avdec 软解)。protocols=tcp 与 FFmpeg 后端一致防 UDP
        花屏; appsink drop+max-buffers=1 始终取最新帧, 不积压涨延迟。
        """
        # build info 是列对齐格式, "GStreamer:" 与 YES 间是多个空格, 不能用单空格子串匹配
        if not re.search(r'GStreamer:\s+YES', cv2.getBuildInformation()):
            self.get_logger().error(
                '当前 OpenCV 未编译 GStreamer 支持, capture_backend:=gstreamer '
                '不可用 (Jetson 请用 JetPack 自带 OpenCV)')
            return None
        latency = int(self._p('gst_latency'))
        tail = 'appsink drop=true max-buffers=1 sync=false'
        src = f'rtspsrc location={self._url} latency={latency} protocols=tcp'
        pipes = [
            f'{src} ! decodebin ! nvvidconv ! video/x-raw,format=BGRx ! '
            f'videoconvert ! video/x-raw,format=BGR ! {tail}',
            f'{src} ! decodebin ! videoconvert ! video/x-raw,format=BGR ! {tail}',
        ]
        for pipe in pipes:
            cap = cv2.VideoCapture(pipe, cv2.CAP_GSTREAMER)
            if cap.isOpened():
                self.get_logger().info(f'GStreamer 管线: {pipe}')
                return cap
            cap.release()
        return None

    def _capture_loop(self):
        """采集线程: 连接 → 循环读帧发布 → 断流重连。"""
        while rclpy.ok() and not self._stop.is_set():
            cap = self._open_capture()
            if cap is None:
                self.get_logger().error(
                    f'RTSP 连接失败: {self._url}, '
                    f'{self._reconnect_sec:.1f}s 后重试',
                    throttle_duration_sec=5.0)
                time.sleep(self._reconnect_sec)
                continue

            self.get_logger().info(f'RTSP 已连接: {self._url}')
            if self._pump(cap):
                break   # 正常退出(节点关闭)
            cap.release()
            if rclpy.ok() and not self._stop.is_set():
                self._reconnects += 1
                self.get_logger().warn(
                    f'RTSP 流中断, {self._reconnect_sec:.1f}s 后重连 '
                    f'(累计 {self._reconnects} 次)',
                    throttle_duration_sec=5.0)
                time.sleep(self._reconnect_sec)

    def _pump(self, cap) -> bool:
        """循环读帧并发布。返回 True=正常结束(节点关闭), False=流断开需重连。"""
        min_interval = 0.0 if self._max_fps <= 0 else 1.0 / self._max_fps
        last_pub = 0.0
        while rclpy.ok() and not self._stop.is_set():
            ok, frame = cap.read()
            if not ok or frame is None:
                return False
            now_mono = time.monotonic()
            if min_interval > 0.0 and (now_mono - last_pub) < min_interval - 1e-4:
                continue   # 限流: 帧已消费丢弃, 不积压
            last_pub = now_mono
            try:
                self._publish_frame(frame)
            except Exception as exc:   # 发布失败不该杀死采集线程
                self.get_logger().error(f'发布帧失败: {exc}',
                                        throttle_duration_sec=5.0)
        return True

    def _publish_frame(self, frame):
        """一帧 → Image + CameraInfo (同一时间戳, RELIABLE)。"""
        sh, sw = frame.shape[:2]
        self._stream_wh = (sw, sh)
        f = self._factor_for(sw)
        if f > 1:
            frame = cv2.resize(frame, (sw // f, sh // f),
                               interpolation=cv2.INTER_AREA)
        intr = self._scaled_intr(sw, sh)

        stamp = self.get_clock().now().to_msg()

        img = Image()
        img.header.stamp = stamp
        img.header.frame_id = self._frame_id
        img.height, img.width = frame.shape[:2]
        if frame.ndim == 2:
            img.encoding = 'mono8'
            img.step = img.width
        else:
            img.encoding = 'bgr8'
            img.step = img.width * frame.shape[2]
        img.is_bigendian = 0
        img.data = frame.tobytes()

        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = self._frame_id
        info.height = intr['height']
        info.width = intr['width']
        info.distortion_model = intr['model']
        info.d = [float(v) for v in intr['d']]
        info.k = [intr['fx'], 0.0, intr['cx'],
                  0.0, intr['fy'], intr['cy'],
                  0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0,
                  0.0, 1.0, 0.0,
                  0.0, 0.0, 1.0]
        info.p = [intr['fx'], 0.0, intr['cx'], 0.0,
                  0.0, intr['fy'], intr['cy'], 0.0,
                  0.0, 0.0, 1.0, 0.0]
        info.binning_x = f
        info.binning_y = f

        self._img_pub.publish(img)
        self._info_pub.publish(info)
        self._pub_count += 1
        self._last_frame_mono = time.monotonic()

    # ── 统计 ───────────────────────────────────────────────────────

    def _log_stats(self):
        """每 5s 打印帧率/分辨率, 无帧超时告警 — 机器狗首调时看这里。"""
        now = time.monotonic()
        dt = now - self._last_stats_mono
        if dt <= 0:
            return
        fps = (self._pub_count - self._last_stats_count) / dt
        self._last_stats_mono = now
        self._last_stats_count = self._pub_count

        res = 'x'.join(str(v) for v in self._stream_wh) if self._stream_wh else '?'
        f = self._factor_cache.get(self._stream_wh[0], 1) if self._stream_wh else 1
        age = now - self._last_frame_mono if self._last_frame_mono else float('inf')
        if age > self._frame_timeout_sec:
            self.get_logger().warn(
                f'RTSP 无帧已 {age:.1f}s (流={res}, 最近发布率 {fps:.1f}fps, '
                f'重连 {self._reconnects} 次) — 检查相机供电/网络/URL')
        else:
            self.get_logger().info(
                f'RTSP: {res} → x{f} 降采样, 发布 {fps:.1f} fps, '
                f'累计 {self._pub_count} 帧, 重连 {self._reconnects} 次')


def main(args=None):
    rclpy.init(args=args)
    try:
        node = RtspCameraNode()
    except RuntimeError as exc:
        # 参数/内参缺失: 错误信息已在节点里 log 过, 这里静默退出非 0
        print(f'[rtsp_camera] 启动失败: {exc}', flush=True)
        if rclpy.ok():
            rclpy.shutdown()
        raise SystemExit(1)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node._stop.set()
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
