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

参数
----
image_topic            源 image 话题 (默认 /image_raw)
camera_info_topic      源 camera_info 话题 (默认 /camera_info)
image_out_topic        桥输出的 image 话题 (默认 /camera_sync/image_raw)
camera_info_out_topic  桥输出的 camera_info 话题 (默认 /camera_sync/camera_info)

用法
----
    ros2 run tagdocking camera_info_bridge
"""

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

        image_topic = self.get_parameter('image_topic').value
        info_topic = self.get_parameter('camera_info_topic').value
        image_out = self.get_parameter('image_out_topic').value
        info_out = self.get_parameter('camera_info_out_topic').value

        # 最近一份 camera_info (无回波前为 None, 期间丢弃 image 不转发,
        # 避免把空内参喂给 apriltag)
        self._latest_info = None

        # 输入端: 相机驱动可能是 RELIABLE 也可能是 BEST_EFFORT
        # (Astra color=image_raw 实测是 RELIABLE), 用 sensor_data profile 兼容两者。
        in_qos = qos_profile_sensor_data

        # 输出端: apriltag_ros 默认按 RELIABLE 订阅 image_rect,
        # 这里发 RELIABLE 与之匹配。
        out_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        self.create_subscription(Image, image_topic, self._on_image, in_qos)
        self.create_subscription(CameraInfo, info_topic, self._on_info, in_qos)

        self._img_pub = self.create_publisher(Image, image_out, out_qos)
        self._info_pub = self.create_publisher(CameraInfo, info_out, out_qos)

        self.get_logger().info(
            f'bridge: {image_topic}+{info_topic} -> '
            f'{image_out}+{info_out}')

    def _on_info(self, msg: CameraInfo):
        self._latest_info = msg

    def _on_image(self, msg: Image):
        info = self._latest_info
        if info is None:
            return

        # 用 image 的时间戳重发 image (原样) 和 camera_info (改 header)
        # 两者的 header.stamp 取同一值, 保证 apriltag 严格配对。
        self._img_pub.publish(msg)

        sync = CameraInfo()
        sync.header.stamp = msg.header.stamp
        # camera_info 的 frame 由相机驱动保证与 image 一致, 直接沿用
        sync.header.frame_id = info.header.frame_id
        sync.height = info.height
        sync.width = info.width
        sync.distortion_model = info.distortion_model
        sync.d = info.d
        sync.k = info.k
        sync.r = info.r
        sync.p = info.p
        sync.binning_x = info.binning_x
        sync.binning_y = info.binning_y
        sync.roi = info.roi
        self._info_pub.publish(sync)


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
