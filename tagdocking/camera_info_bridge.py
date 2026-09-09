"""CameraInfo timestamp-synchronization bridge.

为什么需要它
------------
相机驱动（Astra / usb_cam）发布的 ``image`` 和 ``camera_info`` 时间戳几乎总是
不对齐：两者由各自的采集线程独立打戳，差几毫秒到几十毫秒。apriltag_ros 用
``image_transport::CameraSubscriber`` 严格按时间戳配对 image + camera_info，
错开的帧被当作无法配对而丢弃 —— 实测 ``Synchronized pairs: 0``，apriltag
检测频率跌到个位数，转向后来不及重新看到 tag 而丢失。

本桥做的事
----------
订阅源 ``image_topic`` 和 ``camera_info_topic``。每收到一帧 image，就用**该
image 的消息头时间戳**重发一份最新的 camera_info（并拷贝 image 头一起重发），
保证输出端 ``sync_image_topic`` / ``sync_info_topic`` 时间戳逐帧对齐，apriltag
能拿到全部配对帧。

注意：camera_info 内容用最近收到的那份原样转发（只改 header），因为内参极少
变化；只是把它"盖"到 image 的时间戳上。

内参合成模式 (无 camera_info 源的相机, 如 odin1)
------------------------------------------------
odin1 驱动只发 ``/odin1/image/undistorted`` (去畸变 RGB 1600x1296), 全系统
没有 camera_info 话题, 且图像 frame_id 为空。设置 ``camera_info_file`` (或
内联 fx/fy) 后桥进入合成模式: 不再订阅源 camera_info, 每帧用标定内参现场
构造 CameraInfo —— 标定分辨率与实际图像不一致时按比例缩放, 再按 downscale
缩小 (K/P 同步换算, 同 rtsp_camera)。同时把 image/camera_info 的 frame_id
重打为 ``frame_id`` 参数 (apriltag 广播的 tag TF 挂在该坐标系下, 空 frame_id
会导致 TF 链断裂), 并可选发布 base_frame→相机光学系 静态 TF (mount.* 安装
位姿, 与 rtsp_camera._publish_mount_tf 同一套约定)。

参数
----
image_topic            源 image 话题 (默认 /image_raw)
camera_info_topic      源 camera_info 话题 (默认 /camera_info; 合成模式不用)
image_out_topic        桥输出的 image 话题 (默认 /camera_sync/image_raw)
camera_info_out_topic  桥输出的 camera_info 话题 (默认 /camera_sync/camera_info)
downscale              降采样因子 (整数, 按像素抽取; image 与内参同步缩小)
max_fps                输入限流 fps (0=不限流; 下游 apriltag 消化不掉时防
                       订阅队列积压出秒级延迟, odin 全分辨率实测 ~1.15s)
camera_info_file       内参 YAML (设置后进入合成模式; calibrate_rtsp 输出格式,
                       camera_matrix 3x3 或 {fx,fy,cx,cy}, width/height 可省)
fx/fy/cx/cy            内联内参 (camera_info_file 为空且 fx/fy>0 时生效;
                       cx/cy 缺省取画面中心)
distortion             内联畸变 (逗号分隔 "k1,k2,p1,p2,k3"; 仅合成模式)
distortion_model       畸变模型 (默认 plumb_bob; 仅合成模式)
frame_id               非空时重打 image/camera_info 的 frame_id (odin 图像
                       frame_id 为空, 必须给)
base_frame             静态 TF 父系 (默认 base_link)
mount.*                相机体系安装位姿 (x/y/z m + yaw/pitch/roll deg,
                       publish_static_tf=true 时生效)
publish_static_tf      发布 base_frame→frame_id 静态 TF (默认 false)

用法
----
    # 透传模式 (Astra / usb_cam 等自带 camera_info 的相机)
    ros2 run tagdocking camera_info_bridge

    # 合成模式 (odin1 去畸变流; 内参见 config/odin_camera_info.yaml)
    ros2 run tagdocking camera_info_bridge --ros-args \\
        -p image_topic:=/odin1/image/undistorted \\
        -p camera_info_file:=config/odin_camera_info.yaml \\
        -p frame_id:=camera_color_optical_frame \\
        -p downscale:=1 -p publish_static_tf:=true
"""

import array
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, CameraInfo


class CameraInfoBridge(Node):

    def __init__(self):
        super().__init__('camera_info_bridge')

        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('camera_info_topic', '/camera_info')
        self.declare_parameter('image_out_topic', '/camera_sync/image_raw')
        self.declare_parameter('camera_info_out_topic', '/camera_sync/camera_info')
        # 降采样因子 (整数, 按像素抽取)。>1 时把 image 和 camera_info 内参同步缩小,
        # 再转发给 apriltag。理由: 本机 FastDDS 对大帧 RELIABLE image 投递有缺陷,
        # 640x480 rgb8=900KB/帧会把发送通道堵住, 使同回调背靠背发的 camera_info
        # 错开到达 → apriltag 的 exact 时间戳同步配不上对 (Synchronized pairs≈0)
        # → 检测几乎为 0 → 停靠锁不到码。降到 320x240(=230KB) 负载减 4 倍, 同步恢复。
        self.declare_parameter('downscale', 2)

        # 输入限流 (fps, 0=不限流)。相机帧率高于下游 apriltag 消化能力时,
        # RELIABLE 订阅队列恒满, apriltag 每帧处理的都是积压的旧图 —— 实测
        # odin 全分辨率 22fps 输入、apriltag 仅消化 ~6fps, 检出时间戳年龄
        # 积到 ~1.15s (移动 tag 后读数要 2s 才跟上)。限流在订阅回调里直接
        # 丢帧不外发, 下游队列不再积压 (同 rtsp_camera max_fps 的做法)。
        self.declare_parameter('max_fps', 0.0)

        # ── 内参合成模式 (odin1 等无 camera_info 源) ──────────────
        self.declare_parameter('camera_info_file', '')
        self.declare_parameter('fx', 0.0)
        self.declare_parameter('fy', 0.0)
        self.declare_parameter('cx', 0.0)
        self.declare_parameter('cy', 0.0)
        self.declare_parameter('distortion', '')
        self.declare_parameter('distortion_model', 'plumb_bob')
        # frame_id 重打: odin 图像 frame_id 为空, apriltag 的 tag TF 会挂在
        # 空 frame 下 → docking 的 TF 链 (base→camera→tag) 断裂。非空时重打。
        self.declare_parameter('frame_id', '')
        # 静态 TF (同 rtsp_camera 的 mount.* 约定)
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('mount.x', 0.0)
        self.declare_parameter('mount.y', 0.0)
        self.declare_parameter('mount.z', 0.0)
        self.declare_parameter('mount.yaw_deg', 0.0)
        self.declare_parameter('mount.pitch_deg', 0.0)
        self.declare_parameter('mount.roll_deg', 0.0)
        self.declare_parameter('publish_static_tf', False)

        image_topic = self.get_parameter('image_topic').value
        info_topic = self.get_parameter('camera_info_topic').value
        image_out = self.get_parameter('image_out_topic').value
        info_out = self.get_parameter('camera_info_out_topic').value
        self._downscale = int(self.get_parameter('downscale').value)
        self._frame_id = str(self.get_parameter('frame_id').value)
        self._max_fps = float(self.get_parameter('max_fps').value)
        self._min_interval = 0.0 if self._max_fps <= 0 else 1.0 / self._max_fps
        self._last_pub_mono = 0.0

        # 合成模式的内参 (None = 原透传模式)
        self._synth_intr = self._load_intrinsics()
        # (图像w, 图像h) → 换算后的内参缓存
        self._intr_cache = {}

        # 最近一份 camera_info (仅透传模式; 无回波前为 None, 期间丢弃 image
        # 不转发, 避免把空内参喂给 apriltag)
        self._latest_info = None

        # 输入端: 相机驱动可能是 RELIABLE 也可能是 BEST_EFFORT
        # (Astra color=image_raw 实测是 RELIABLE), 用 sensor_data profile 兼容两者。
        in_qos = qos_profile_sensor_data

        # 输出端: 发 RELIABLE (depth=10)。
        # 实测 apriltag_ros 的 image_transport::CameraSubscriber 在本机用 BEST_EFFORT
        # 订阅时反而收不到 image (同话题同 QoS 下普通订阅能收 8Hz, apriltag 收 0,
        # 怀疑 image_transport 插件层与 BEST_EFFORT 不兼容); 发 RELIABLE 时 apriltag
        # 虽然只收到 ~0.3 帧/s 且 warning 刷 "Synchronized pairs: 0", 但检测能正常出
        # (detections 有输出, 停靠可进行)。故保留 RELIABLE, warning 视为噪声。
        out_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        self.create_subscription(Image, image_topic, self._on_image, in_qos)
        # 合成模式没有源 camera_info 可订, 内参每帧现场构造
        if self._synth_intr is None:
            self.create_subscription(CameraInfo, info_topic, self._on_info, in_qos)

        self._img_pub = self.create_publisher(Image, image_out, out_qos)
        self._info_pub = self.create_publisher(CameraInfo, info_out, out_qos)

        if bool(self.get_parameter('publish_static_tf').value):
            self._publish_mount_tf()

        mode = (f'合成 {self._synth_intr_src}' if self._synth_intr is not None
                else f'透传 {info_topic}')
        self.get_logger().info(
            f'bridge ({mode}): {image_topic} -> {image_out}+{info_out} '
            f'(downscale={self._downscale}, max_fps={self._max_fps or "不限"}, '
            f'frame_id={self._frame_id or "源值"})')

    # ── 内参加载 (合成模式) ───────────────────────────────────────

    def _load_intrinsics(self):
        """读内参, 返回 {width,height,fx,fy,cx,cy,d,model}; 无内参返回 None。"""
        path = str(self.get_parameter('camera_info_file').value).strip()
        if path:
            import yaml
            try:
                with open(path, encoding='utf-8') as f:
                    data = yaml.safe_load(f)
                intr = self._parse_intrinsics_yaml(data)
            except Exception as exc:
                self.get_logger().error(f'解析 camera_info_file 失败: {path}: {exc}')
                raise SystemExit(f'camera_info_file 解析失败: {exc}')
            self._synth_intr_src = f'文件 {path}'
            return intr

        fx = float(self.get_parameter('fx').value)
        fy = float(self.get_parameter('fy').value)
        if fx <= 0.0 or fy <= 0.0:
            return None  # 无内参 → 透传模式
        # 内联模式 width/height 未知, 记 0 → 首帧按图像分辨率视为标定分辨率;
        # cx/cy 为 0 时 _intr_for 里按画面中心补
        cx = float(self.get_parameter('cx').value)
        cy = float(self.get_parameter('cy').value)
        if cx <= 0.0 or cy <= 0.0:
            self.get_logger().warn('cx/cy 未提供, 首帧按画面中心取值')
        d = [float(v) for v in
             str(self.get_parameter('distortion').value).replace(' ', '').split(',')
             if v]
        self._synth_intr_src = '内联 fx/fy'
        return {'width': 0, 'height': 0, 'fx': fx, 'fy': fy,
                'cx': cx, 'cy': cy, 'd': d,
                'model': str(self.get_parameter('distortion_model').value)}

    def _parse_intrinsics_yaml(self, data: dict) -> dict:
        """解析标定 YAML (与 rtsp_camera 同格式: camera_matrix/K, distortion)。"""
        mat = data.get('camera_matrix') or data.get('K')
        if mat is None:
            raise ValueError('缺少 camera_matrix (或 K)')
        k = self._matrix_to_list(mat)
        fx, cx, fy, cy = k[0], k[2], k[4], k[5]
        if fx <= 0 or fy <= 0:
            raise ValueError(f'fx/fy 非法: fx={fx}, fy={fy}')
        d = data.get('distortion', data.get('D', []))
        if not isinstance(d, (list, tuple)):
            raise ValueError(f'distortion 应为列表, 实际 {type(d).__name__}')
        # width/height 可省略: 记 0, 首帧按图像分辨率视为标定分辨率
        return {'width': int(data.get('width', 0) or 0),
                'height': int(data.get('height', 0) or 0),
                'fx': fx, 'fy': fy, 'cx': cx, 'cy': cy,
                'd': [float(v) for v in d],
                'model': str(data.get('distortion_model', 'plumb_bob'))}

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

    def _intr_for(self, w: int, h: int) -> dict:
        """内参换算到 (w×h 图像 ÷ downscale), 带缓存。

        标定 YAML 未写分辨率时视图像分辨率为标定分辨率; 不一致时按比例缩放。
        """
        key = (w, h)
        if key in self._intr_cache:
            return self._intr_cache[key]
        intr = self._synth_intr
        cw = intr['width'] or w
        ch = intr['height'] or h
        f = self._downscale
        # cx/cy 未提供时按标定分辨率画面中心补 (同 rtsp_camera 内联模式)
        cx = intr['cx'] if intr['cx'] > 0.0 else (cw - 1) / 2.0
        cy = intr['cy'] if intr['cy'] > 0.0 else (ch - 1) / 2.0
        val = {'fx': intr['fx'] * w / cw / f, 'fy': intr['fy'] * h / ch / f,
               'cx': cx * w / cw / f, 'cy': cy * h / ch / f,
               'd': intr['d'], 'model': intr['model'],
               'width': w // f, 'height': h // f}
        self._intr_cache[key] = val
        return val

    def _build_info(self, img: Image, src_w: int, src_h: int, f: int) -> CameraInfo:
        """用图像头时间戳 + 换算后内参构造 CameraInfo。

        src_w/src_h 是降采样**前**的源分辨率, downscale 只在 _intr_for 内
        统一换算, 保证内参与输出图像尺寸一致。
        """
        intr = self._intr_for(src_w, src_h)
        info = CameraInfo()
        info.header.stamp = img.header.stamp
        info.header.frame_id = img.header.frame_id
        info.width = intr['width']
        info.height = intr['height']
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
        return info

    # ── 静态 TF (与 rtsp_camera._publish_mount_tf 同一套约定) ─────

    def _publish_mount_tf(self):
        """发布静态 TF base_frame→frame_id (安装位姿 × 光学系旋转)。

        mount.* 是相机**体系**(x前 y左 z上)在 base_link 下的安装位姿;
        frame_id 是图像**光学系**(x右 y下 z前, camera_info 内参所属系),
        两者相差固定旋转 _Q_BODY_TO_OPTICAL。总旋转 = R_mount · R_optical。
        四元数工具从 rtsp_camera 惰性导入 (避免让不需要 TF 的透传用法背上
        rtsp_camera 的 cv2 硬依赖)。
        """
        from tagdocking.rtsp_camera import (
            _Q_BODY_TO_OPTICAL, _euler_zyx_quat, _quat_mul)
        import math
        import tf2_ros
        from geometry_msgs.msg import TransformStamped

        yaw = math.radians(float(self.get_parameter('mount.yaw_deg').value))
        pitch = math.radians(float(self.get_parameter('mount.pitch_deg').value))
        roll = math.radians(float(self.get_parameter('mount.roll_deg').value))
        qx, qy, qz, qw = _quat_mul(_euler_zyx_quat(yaw, pitch, roll),
                                   _Q_BODY_TO_OPTICAL)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = str(self.get_parameter('base_frame').value)
        tf_msg.child_frame_id = self._frame_id
        tf_msg.transform.translation.x = float(self.get_parameter('mount.x').value)
        tf_msg.transform.translation.y = float(self.get_parameter('mount.y').value)
        tf_msg.transform.translation.z = float(self.get_parameter('mount.z').value)
        tf_msg.transform.rotation.x = qx
        tf_msg.transform.rotation.y = qy
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw

        self._static_tf = tf2_ros.StaticTransformBroadcaster(self)
        self._static_tf.sendTransform(tf_msg)
        self.get_logger().info(
            f'静态TF: {tf_msg.header.frame_id} → {self._frame_id} | '
            f'安装位姿 x={tf_msg.transform.translation.x} '
            f'y={tf_msg.transform.translation.y} '
            f'z={tf_msg.transform.translation.z} m, '
            f'yaw={self.get_parameter("mount.yaw_deg").value} '
            f'pitch={self.get_parameter("mount.pitch_deg").value} '
            f'roll={self.get_parameter("mount.roll_deg").value} deg')

    # ── 转发 ──────────────────────────────────────────────────────

    def _on_info(self, msg: CameraInfo):
        self._latest_info = msg

    def _on_image(self, msg: Image):
        # 限流: 超额帧在这里直接丢弃 (不外发)。不丢帧的话下游 apriltag
        # 消化不掉只会积压出秒级延迟, 还多烧一份 DDS/处理带宽。
        if self._min_interval > 0.0:
            now_mono = time.monotonic()
            if (now_mono - self._last_pub_mono) < self._min_interval - 1e-4:
                return
            self._last_pub_mono = now_mono

        if self._frame_id:
            msg.header.frame_id = self._frame_id

        # 合成模式: 每帧现场构造 CameraInfo, 不依赖源 camera_info
        if self._synth_intr is not None:
            f = self._downscale
            src_w, src_h = msg.width, msg.height
            if f > 1:
                msg = self._downscale_image(msg, f)
            self._img_pub.publish(msg)
            # 内参按**降采样前**的源分辨率换算 (_intr_for 语义是"标定系 →
            # w×h 图像 ÷ downscale")。传缩小后的尺寸会把 downscale 除两次:
            # fx 减半、width 减半, 与实际图像错一倍 → PnP 距离差 2 倍、
            # 横向差数倍 (2026-09-08 实测 400x324 内参配 800x648 图)。
            self._info_pub.publish(self._build_info(msg, src_w, src_h, f))
            return

        info = self._latest_info
        if info is None:
            return

        f = self._downscale
        if f > 1:
            msg = self._downscale_image(msg, f)

        # 用 image 的时间戳重发 image 和 camera_info (改 header)
        # 两者的 header.stamp 取同一值, 保证 apriltag 严格配对。
        self._img_pub.publish(msg)

        sync = CameraInfo()
        sync.header.stamp = msg.header.stamp
        # camera_info 的 frame 由相机驱动保证与 image 一致, 直接沿用
        sync.header.frame_id = self._frame_id or info.header.frame_id
        sync.distortion_model = info.distortion_model
        sync.d = info.d
        sync.r = info.r
        if f > 1:
            sync.width = info.width // f
            sync.height = info.height // f
            sync.k = self._scale_k(info.k, f)
            sync.p = self._scale_p(info.p, f)
            sync.binning_x = (info.binning_x or 1) * f
            sync.binning_y = (info.binning_y or 1) * f
        else:
            sync.width = info.width
            sync.height = info.height
            sync.k = info.k
            sync.p = info.p
            sync.binning_x = info.binning_x
            sync.binning_y = info.binning_y
        sync.roi = info.roi
        self._info_pub.publish(sync)

    @staticmethod
    def _downscale_image(msg: Image, f: int) -> Image:
        """按因子 f 抽样降采样 (无损于 apriltag 检测, 16cm tag @1m 仍有 ~50px)。"""
        w, h = msg.width, msg.height
        bpp = msg.step // w if w else 1
        try:
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, bpp)
            small = arr[::f, ::f, :]
            # array.array('B') 走 Image.data 快路径; bytes 会触发 setter 逐元素
            # 校验(全帧两遍 Python 迭代), 大帧下烧满 CPU (同 rtsp_camera 的教训)
            msg.data = array.array('B', small.tobytes())
            msg.height = small.shape[0]
            msg.width = small.shape[1]
            msg.step = small.shape[1] * bpp
        except Exception:
            # 解析失败就原样发 (宁可图大也别发空图)
            pass
        return msg

    @staticmethod
    def _scale_k(k, f: int):
        # K = [fx, 0, cx,  0, fy, cy,  0, 0, 1]
        k = list(k)
        k[0] /= f; k[2] /= f
        k[4] /= f; k[5] /= f
        return k

    @staticmethod
    def _scale_p(p, f: int):
        # P = [fx, 0, cx, Tx,  0, fy, cy, Ty,  0, 0, 1, 0]
        p = list(p)
        p[0] /= f; p[2] /= f; p[3] /= f
        p[5] /= f; p[6] /= f; p[7] /= f
        return p


def main(args=None):
    rclpy.init(args=args)
    node = CameraInfoBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
