"""Launch file for tagdocking — AprilTag auto-docking framework.

Starts:
  1. Camera source — 二选一:
       a. rtsp_url 非空 (机器狗): rtsp_camera 桥 (拉流 + 内参合成 + 静态TF)
       b. 默认 (ROS 相机话题): camera_info_bridge (时间戳同步 + 降采样)
  2. april_tag node (tag detection + TF broadcast)
  3. docking_node (tagdocking controller)

Usage:
  # Minimal: camera already publishing, just start docking stack
  ros2 launch tagdocking docking.launch.py

  # With custom tag
  ros2 launch tagdocking docking.launch.py dock_tag_id:=5 tag_size:=0.21

  # Omni wheel mode
  ros2 launch tagdocking docking.launch.py base_type:=omni

  # 机器狗 (RTSP 相机 + cmd_vel): 先 scripts/calibrate_rtsp 标定, 再:
  ros2 launch tagdocking docking.launch.py base_type:=omni \\
      rtsp_url:=rtsp://192.168.1.100:8554/live \\
      camera_info_file:=$PWD/config/rtsp_camera_info.yaml \\
      odom_topic:=/odom camera_mount_z:=0.35
"""

import os
import subprocess
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def launch_setup(context):
    # ── Kill stray apriltag_node processes from previous crashed runs ──
    # apriltag_node is a separate process from docking_node; if docking_node
    # crashed (or was SIGKILLed), the apriltag node survived. Multiple stray
    # apriltag nodes each broadcast the SAME tag TF frame (tag<family>:<id>),
    # so the TF listener returns whichever competing transform arrived last —
    # producing wild, contradictory lat/dist jumps between measurements that
    # make docking impossible. Reap them before starting a fresh one.
    try:
        subprocess.run(['pkill', '-9', '-f', 'apriltag_node'],
                       timeout=5, check=False)
    except Exception:
        pass

    image_topic = LaunchConfiguration('image_topic').perform(context)
    camera_info_topic = LaunchConfiguration('camera_info_topic').perform(context)
    family = LaunchConfiguration('family').perform(context)
    tag_size = float(LaunchConfiguration('tag_size').perform(context))
    dock_tag_id = int(LaunchConfiguration('dock_tag_id').perform(context))
    camera_frame = LaunchConfiguration('camera_frame').perform(context)
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic').perform(context)
    base_type = LaunchConfiguration('base_type').perform(context)
    dock_distance = float(LaunchConfiguration('dock_distance').perform(context))
    final_straight_distance = float(LaunchConfiguration('final_straight_distance').perform(context))
    final_straight_yaw_deg = float(LaunchConfiguration('final_straight_yaw_deg').perform(context))

    # ── RTSP 相机模式 (机器狗) ──
    rtsp_url = LaunchConfiguration('rtsp_url').perform(context).strip()
    camera_info_file = LaunchConfiguration('camera_info_file').perform(context).strip()
    camera_downscale = int(LaunchConfiguration('camera_downscale').perform(context) or 0)
    camera_backend = LaunchConfiguration('camera_backend').perform(context)
    odom_topic = LaunchConfiguration('odom_topic').perform(context)
    base_frame = LaunchConfiguration('base_frame').perform(context)
    mount_x = float(LaunchConfiguration('camera_mount_x').perform(context) or 0.0)
    mount_y = float(LaunchConfiguration('camera_mount_y').perform(context) or 0.0)
    mount_z = float(LaunchConfiguration('camera_mount_z').perform(context) or 0.0)
    mount_yaw = float(LaunchConfiguration('camera_mount_yaw_deg').perform(context) or 0.0)
    mount_pitch = float(LaunchConfiguration('camera_mount_pitch_deg').perform(context) or 0.0)
    mount_roll = float(LaunchConfiguration('camera_mount_roll_deg').perform(context) or 0.0)
    use_rtsp = rtsp_url != ''
    if use_rtsp and not camera_info_file:
        print('[docking.launch] 警告: rtsp_url 已设置但 camera_info_file 为空, '
              'rtsp_camera 将启动失败。先标定: python3 scripts/calibrate_rtsp '
              '--url <rtsp地址> --out config/rtsp_camera_info.yaml',
              file=sys.stderr)

    pkg_share = get_package_share_directory('tagdocking')
    config_path = os.path.join(pkg_share, 'config', 'docking.yaml')

    # AprilTag frame name convention: tag<family>:<id>
    tag_frame = f'tag{family}:{dock_tag_id}'

    # 同步桥输出话题。apriltag_ros 的 image_transport::CameraSubscriber 从 image
    # 话题名同级派生 camera_info (image_raw → camera_info), 所以 apriltag 必须订阅
    # 桥的 image 输出, 才能拿到时间戳对齐的 camera_info。
    sync_image_topic = '/camera_sync/image_raw'
    sync_info_topic = '/camera_sync/camera_info'

    nodes = []

    if use_rtsp:
        # ── RTSP 相机桥 (机器狗模式) ─────────────────────────────
        # 替代"相机驱动 + camera_info_bridge": 直接产出时间戳逐帧对齐的
        # image + camera_info 到 sync 话题 (apriltag 订阅口不变), 并发布
        # base_frame→相机光学系 静态 TF (安装位姿 mount.*), 内含降采样。
        nodes.append(Node(
            package='tagdocking',
            executable='rtsp_camera',
            name='rtsp_camera',
            parameters=[{
                'rtsp_url': rtsp_url,
                'camera_info_file': camera_info_file,
                'image_out_topic': sync_image_topic,
                'camera_info_out_topic': sync_info_topic,
                'frame_id': camera_frame,
                'downscale': camera_downscale,
                'capture_backend': camera_backend,
                'base_frame': base_frame,
                'mount.x': mount_x,
                'mount.y': mount_y,
                'mount.z': mount_z,
                'mount.yaw_deg': mount_yaw,
                'mount.pitch_deg': mount_pitch,
                'mount.roll_deg': mount_roll,
                'publish_static_tf': True,
            }],
            output='screen',
        ))
    else:
        # ── CameraInfo 同步桥 (ROS 相机话题模式) ─────────────────
        # 相机 (usb_cam/astra) 的 image 与 camera_info 时间戳不对齐, apriltag 的严格
        # 时间同步会丢弃几乎所有帧 (Synchronized pairs: 0) → 检测频率极低 → 转向后
        # 来不及重新看到 tag 而丢失。桥每收到一帧 image 就用其时间戳重发 camera_info,
        # 保证每帧都能配对。
        #
        # camera_info_bridge 是 tagdocking 包内节点 (原依赖的独立 autodock 包已废弃,
        # 实现见 tagdocking/camera_info_bridge.py)。
        nodes.append(Node(
            package='tagdocking',
            executable='camera_info_bridge',
            name='camera_info_bridge',
            parameters=[{
                'image_topic': image_topic,
                'camera_info_topic': camera_info_topic,
                'image_out_topic': sync_image_topic,
                'camera_info_out_topic': sync_info_topic,
            }],
            output='screen',
        ))

    # ── AprilTag detection node ─────────────────────────────────
    nodes.append(Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_node',
        parameters=[{
            'family': family,
            'size': tag_size,
            'tag_ids': [dock_tag_id],
            'tag_frames': [tag_frame],
            'publish_tf': True,
            'z_up': False,
        }],
        remappings=[
            # 订阅桥的同步输出而非裸 image_raw; camera_info 自动派生到 sync_info_topic
            ('image_rect', sync_image_topic),
        ],
        output='screen',
    ))

    # ── Docking controller ──────────────────────────────────────
    nodes.append(Node(
        package='tagdocking',
        executable='docking_node',
        name='docking_node',
        parameters=[
            config_path,
            {
                'tag.frame': tag_frame,
                'tag.id': dock_tag_id,
                'tag.family': family,
                'tag.size': tag_size,
                'camera_frame': camera_frame,
                'base_frame': base_frame,
                'base.cmd_vel_topic': cmd_vel_topic,
                'base.type': base_type,
                'odom_topic': odom_topic,
                # 两阶段停泊参数 (覆盖 yaml)
                'dock_target.distance': dock_distance,
                'final_straight.start_distance': final_straight_distance,
                'final_straight.yaw_threshold_deg': final_straight_yaw_deg,
            },
        ],
        output='screen',
    ))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('image_topic', default_value='/image_raw',
                             description='Camera image topic for apriltag_ros'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera_info',
                             description='Camera info topic (source for sync bridge)'),
        DeclareLaunchArgument('family', default_value='36h11',
                             description='AprilTag family (36h11, 25h9, etc.)'),
        DeclareLaunchArgument('tag_size', default_value='0.16',
                             description='Tag edge size in meters'),
        DeclareLaunchArgument('dock_tag_id', default_value='0',
                             description='Tag ID to dock to'),
        DeclareLaunchArgument('camera_frame',
                             default_value='camera_color_optical_frame',
                             description='Camera optical frame name'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='cmd_vel',
                             description='Velocity command topic'),
        DeclareLaunchArgument('base_type', default_value='diff_drive',
                             description='Chassis type: diff_drive, omni, quadruped'),
        DeclareLaunchArgument('dock_distance', default_value='0.55',
                             description='最终停泊距离 (m), 底盘距 tag'),
        DeclareLaunchArgument('final_straight_distance', default_value='0.85',
                             description='直行阶段起点距离 (m), 到此距离后纯直行不再调角 (须 > dock_distance)'),
        DeclareLaunchArgument('final_straight_yaw_deg', default_value='5.0',
                             description='进入直行阶段的航向门槛 (deg, 方阵误差)'),

        # ── RTSP 相机模式 (机器狗) ──────────────────────────────
        DeclareLaunchArgument('rtsp_url', default_value='',
                             description='RTSP 地址。非空时用 rtsp_camera 桥替代 '
                                         'camera_info_bridge + 外部相机话题 (机器狗模式)'),
        DeclareLaunchArgument('camera_info_file', default_value='',
                             description='相机内参 YAML (rtsp 模式必填; 由 '
                                         'scripts/calibrate_rtsp 生成)'),
        DeclareLaunchArgument('camera_downscale', default_value='0',
                             description='RTSP 输出降采样倍数 (0=自动到 ~640 宽)'),
        DeclareLaunchArgument('camera_backend', default_value='ffmpeg',
                             description='RTSP 拉流后端: ffmpeg | gstreamer '
                                         '(Jetson 硬解, FFmpeg 解码冻结时用)'),
        DeclareLaunchArgument('odom_topic', default_value='/odom_combined',
                             description='里程计话题 (机器狗按其实际话题设置)'),
        DeclareLaunchArgument('base_frame', default_value='base_link',
                             description='机器人基座坐标系 (静态 TF 父系 + docking 测量系)'),
        DeclareLaunchArgument('camera_mount_x', default_value='0.0',
                             description='相机安装位置 x (m, base_link 系, rtsp 模式)'),
        DeclareLaunchArgument('camera_mount_y', default_value='0.0',
                             description='相机安装位置 y (m, base_link 系, rtsp 模式)'),
        DeclareLaunchArgument('camera_mount_z', default_value='0.0',
                             description='相机安装高度 z (m, base_link 系, rtsp 模式)'),
        DeclareLaunchArgument('camera_mount_yaw_deg', default_value='0.0',
                             description='相机朝向偏航 (deg, 0=正前, rtsp 模式)'),
        DeclareLaunchArgument('camera_mount_pitch_deg', default_value='0.0',
                             description='相机俯仰 (deg, 正=低头, 负=抬头, rtsp 模式)'),
        DeclareLaunchArgument('camera_mount_roll_deg', default_value='0.0',
                             description='相机横滚 (deg, rtsp 模式)'),
        OpaqueFunction(function=launch_setup),
    ])
