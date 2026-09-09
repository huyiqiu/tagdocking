"""Launch file for tagdocking — AprilTag auto-docking framework.

Starts:
  1. Camera source — 三选一:
       a. rtsp_url 非空 (机器狗): rtsp_camera 桥 (拉流 + 内参合成 + 静态TF)
       b. use_odin:=true (odin1 相机): camera_info_bridge 合成模式
          (/odin1/image/undistorted 去畸变流 + 包内内参 + frame_id 重打 + 静态TF)
       c. 默认 (ROS 相机话题): camera_info_bridge (时间戳同步 + 降采样)
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

  # odin1 相机 (后装 3D 视觉模组, 去畸变流 1600x1296): 按实际安装位姿传
  # camera_mount_* (与 rtsp 模式同一套参数), 内参自动用包内 odin_camera_info.yaml:
  ros2 launch tagdocking docking.launch.py base_type:=omni use_odin:=true \\
      camera_mount_z:=0.30 camera_mount_pitch_deg:=0.0
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
    # ── Kill stray apriltag_node / camera_info_bridge processes ──
    # apriltag_node is a separate process from docking_node; if docking_node
    # crashed (or was SIGKILLed), the apriltag node survived. Multiple stray
    # apriltag nodes each broadcast the SAME tag TF frame (tag<family>:<id>),
    # so the TF listener returns whichever competing transform arrived last —
    # producing wild, contradictory lat/dist jumps between measurements that
    # make docking impossible. A stray camera_info_bridge (left over from a
    # crashed run or a manual ros2 run) is just as bad: it double-publishes
    # /camera_sync/* images and a second static TF. Reap both before starting
    # a fresh set.
    try:
        subprocess.run(['pkill', '-9', '-f', 'apriltag_node'],
                       timeout=5, check=False)
        subprocess.run(['pkill', '-9', '-f', 'camera_info_bridge'],
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
    base_type = LaunchConfiguration('base_type').perform(context).strip()
    dock_distance = float(LaunchConfiguration('dock_distance').perform(context))
    final_straight_distance = float(LaunchConfiguration('final_straight_distance').perform(context))
    final_straight_yaw_deg = float(LaunchConfiguration('final_straight_yaw_deg').perform(context))
    # 走停单步最大 jog 距离 — 空值时用 yaml 权威值 (同 base_type 的约定)
    jog_max = LaunchConfiguration('jog_max').perform(context).strip()

    # 静止站立 (posture.*) — 空值时用 yaml 权威值
    l1w_prefix = LaunchConfiguration('l1w_prefix').perform(context).strip()
    posture_enable = LaunchConfiguration('posture_enable').perform(context).strip()
    static_settle_sec = LaunchConfiguration('static_settle_sec').perform(context).strip()

    # ── RTSP 相机模式 (机器狗) ──
    rtsp_url = LaunchConfiguration('rtsp_url').perform(context).strip()
    camera_info_file = LaunchConfiguration('camera_info_file').perform(context).strip()
    camera_downscale = int(LaunchConfiguration('camera_downscale').perform(context) or 0)
    camera_backend = LaunchConfiguration('camera_backend').perform(context)
    odom_topic = LaunchConfiguration('odom_topic').perform(context).strip()
    base_frame = LaunchConfiguration('base_frame').perform(context)
    mount_x = float(LaunchConfiguration('camera_mount_x').perform(context) or 0.0)
    mount_y = float(LaunchConfiguration('camera_mount_y').perform(context) or 0.0)
    mount_z = float(LaunchConfiguration('camera_mount_z').perform(context) or 0.0)
    mount_yaw = float(LaunchConfiguration('camera_mount_yaw_deg').perform(context) or 0.0)
    mount_pitch = float(LaunchConfiguration('camera_mount_pitch_deg').perform(context) or 0.0)
    mount_roll = float(LaunchConfiguration('camera_mount_roll_deg').perform(context) or 0.0)
    use_rtsp = rtsp_url != ''

    # ── Odin1 相机模式 (后装 3D 视觉模组) ──
    use_odin = (LaunchConfiguration('use_odin').perform(context)
                .strip().lower() == 'true')
    if use_odin and use_rtsp:
        raise RuntimeError('rtsp_url 与 use_odin 互斥: 一次只能选一种相机源')

    pkg_share = get_package_share_directory('tagdocking')
    if use_rtsp and not camera_info_file:
        # 未显式给内参时回退到包内标定文件 (config/rtsp_camera_info.yaml, 由
        # scripts/calibrate_rtsp 生成后随包安装) —— 换机器/重部署不用记着传路径。
        default_intr = os.path.join(pkg_share, 'config', 'rtsp_camera_info.yaml')
        if os.path.isfile(default_intr):
            camera_info_file = default_intr
            print(f'[docking.launch] camera_info_file 未指定, 使用包内标定文件: '
                  f'{default_intr}')
        else:
            print('[docking.launch] 警告: rtsp_url 已设置但 camera_info_file 为空 '
                  '(包内也无 config/rtsp_camera_info.yaml), rtsp_camera 将启动失败。'
                  '先标定: python3 scripts/calibrate_rtsp '
                  '--url <rtsp地址> --out config/rtsp_camera_info.yaml',
                  file=sys.stderr)

    odin_downscale = 2
    if use_odin:
        # odin 驱动只发去畸变图像 (无 camera_info, frame_id 为空), 内参回退到
        # 包内 odin_camera_info.yaml (抄自 odin calib.yaml, 随包安装)。
        if not camera_info_file:
            odin_intr = os.path.join(pkg_share, 'config', 'odin_camera_info.yaml')
            if os.path.isfile(odin_intr):
                camera_info_file = odin_intr
                print(f'[docking.launch] use_odin: camera_info_file 未指定, '
                      f'使用包内内参: {odin_intr}')
            else:
                raise RuntimeError(
                    'use_odin 需要 camera_info_file (包内也无 '
                    'config/odin_camera_info.yaml, 包未重新构建?)')
        # 用户没显式改 image_topic (仍是默认 /image_raw) 时才指到 odin 去畸变流,
        # 允许显式覆盖 (如想试原始鱼眼流 /odin1/image, 需配对应的鱼眼内参)。
        if image_topic == '/image_raw':
            image_topic = '/odin1/image/undistorted'
        # 降采样+限流默认 (实测 2026-09-08, Jetson): 全分辨率 1600x1296 下
        # apriltag 只消化 ~6fps, odin 22fps 输入把 RELIABLE 队列塞满, 检出
        # 时间戳年龄积到 ~1.15s (移动 tag 读数 2s 才跟上)。÷2 后 apriltag
        # 提速 ~1 倍, 配合桥 max_fps=10 限流丢帧, 队列不再积压, 延迟 ~0.2s。
        # 16cm tag @1m ÷2 后 ≈58px, 仍远高于 36h11 检测下限。要全分辨率
        # (更远的小 tag) 时显式传 camera_downscale:=1。
        odin_downscale = camera_downscale if camera_downscale > 0 else 2
        print(f'[docking.launch] use_odin: {image_topic}, 内参 {camera_info_file}, '
              f'downscale={odin_downscale}')

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
    elif use_odin:
        # ── Odin1 相机桥 (camera_info_bridge 合成模式) ───────────
        # odin_driver 只发 /odin1/image/undistorted (sensor_msgs/Image, 去畸变
        # 1600x1296), 全系统无 camera_info 话题且图像 frame_id 为空。桥用
        # odin_camera_info.yaml 每帧现场构造 CameraInfo (apriltag PnP 必需)、
        # 重打 frame_id (tag TF 挂靠点)、发布 base_frame→相机光学系 静态 TF
        # (mount.* 安装位姿, 与 rtsp 模式同一套参数)。apriltag 订阅口不变。
        nodes.append(Node(
            package='tagdocking',
            executable='camera_info_bridge',
            name='camera_info_bridge',
            parameters=[{
                'image_topic': image_topic,
                'image_out_topic': sync_image_topic,
                'camera_info_out_topic': sync_info_topic,
                'camera_info_file': camera_info_file,
                'frame_id': camera_frame,
                'downscale': odin_downscale,
                # 限流: odin 22fps 远超 apriltag 消化能力, 不限流则队列积压出
                # 秒级延迟 (实测全分辨率 ~1.15s)。10fps 与 rtsp_camera 默认一致。
                'max_fps': 10.0,
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
            # apriltag_ros 3.4.0+ (上游 ROS2 重写, 节点名 /apriltag) 只认嵌套
            # tag.ids/tag.frames/tag.sizes (且无 publish_tf/z_up, TF 无条件
            # 发布), 旧 fork 只认扁平 tag_ids/tag_frames。未识别的参数被静默
            # 忽略, 两套同传兼容两代版本 —— 只传扁平参数时 3.4.0 会"有检测、
            # 无 TF"(tag 不在配置里, 不解算位姿也不广播 TF)。
            'tag.ids': [dock_tag_id],
            'tag.frames': [tag_frame],
            'tag.sizes': [tag_size],
            'publish_tf': True,
            'z_up': False,
            # 默认 decimate=2 会把桥输出的 ~640 宽图再降一半, 远距离小 tag
            # (1m 外 16cm ≈ 25px) 低于 36h11 检测下限 → 检测全空。关掉。
            'detector.decimate': 1.0,
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
                # 两阶段停泊参数 (覆盖 yaml)
                'dock_target.distance': dock_distance,
                'final_straight.start_distance': final_straight_distance,
                'final_straight.yaw_threshold_deg': final_straight_yaw_deg,
            },
            # base_type / odom_topic 为空时用 yaml 值 (yaml 权威): 只有显式传参
            # 才覆盖, 避免无参启动时 launch 默认值悄悄顶掉 yaml 里的底盘配置。
        ] + ([{'base.type': base_type}] if base_type else [])
          + ([{'odom_topic': odom_topic}] if odom_topic else [])
          + ([{'base.l1w_prefix': l1w_prefix}] if l1w_prefix else [])
          + ([{'posture.enable': posture_enable.lower() == 'true'}]
             if posture_enable else [])
          + ([{'posture.static_settle_sec': float(static_settle_sec)}]
             if static_settle_sec else [])
          + ([{'stopgo.jog_max': float(jog_max)}] if jog_max else []),
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
        DeclareLaunchArgument('base_type', default_value='',
                             description='Chassis type: diff_drive, omni, quadruped '
                                         '(空 = 使用 config/docking.yaml 的 base.type)'),
        DeclareLaunchArgument('dock_distance', default_value='0.55',
                             description='最终停泊距离 (m), 底盘距 tag'),
        DeclareLaunchArgument('final_straight_distance', default_value='0.85',
                             description='直行阶段起点距离 (m), 到此距离后纯直行不再调角 (须 > dock_distance)'),
        DeclareLaunchArgument('final_straight_yaw_deg', default_value='5.0',
                             description='进入直行阶段的航向门槛 (deg, 方阵误差)'),
        DeclareLaunchArgument('jog_max', default_value='',
                             description='走停单步最大 jog 距离 (m); 空 = 使用 config/docking.yaml 的 stopgo.jog_max'),

        # ── 静止站立 (posture.*) — 走停 × 呼吸抑制 ────────────────
        DeclareLaunchArgument('l1w_prefix', default_value='',
                              description='狗模式服务前缀 (空 = 使用 yaml 的 '
                                          'base.l1w_prefix; 默认 /l1w_control)'),
        DeclareLaunchArgument('posture_enable', default_value='',
                              description='停稳切静止站立总开关 true/false '
                                          '(空 = 使用 yaml 的 posture.enable)'),
        DeclareLaunchArgument('static_settle_sec', default_value='',
                              description='停→量测最短间隔 (s, 空 = 使用 yaml 的 '
                                          'posture.static_settle_sec)'),

        # ── RTSP 相机模式 (机器狗) ──────────────────────────────
        DeclareLaunchArgument('rtsp_url', default_value='',
                             description='RTSP 地址。非空时用 rtsp_camera 桥替代 '
                                         'camera_info_bridge + 外部相机话题 (机器狗模式)'),
        DeclareLaunchArgument('use_odin', default_value='false',
                             description='用 odin1 相机 (/odin1/image/undistorted '
                                         '去畸变流 + 包内 odin_camera_info.yaml 内参 '
                                         '+ frame_id 重打 + 静态TF) 替代普通相机话题'),
        DeclareLaunchArgument('camera_info_file', default_value='',
                             description='相机内参 YAML (rtsp 模式必填; use_odin 时 '
                                         '缺省用包内 odin_camera_info.yaml; 由 '
                                         'scripts/calibrate_rtsp 生成)'),
        DeclareLaunchArgument('camera_downscale', default_value='0',
                             description='输出降采样倍数 (0=自动: RTSP 到 ~640 宽; '
                                         'use_odin 到 800x648)。odin 全分辨率传 1'),
        DeclareLaunchArgument('camera_backend', default_value='ffmpeg',
                             description='RTSP 拉流后端: ffmpeg | gstreamer '
                                         '(Jetson 硬解, FFmpeg 解码冻结时用)'),
        DeclareLaunchArgument('odom_topic', default_value='',
                             description='里程计话题 (空 = 使用 config/docking.yaml 的 '
                                         'odom_topic; 机器狗为 /dog/odom)'),
        DeclareLaunchArgument('base_frame', default_value='base_link',
                             description='机器人基座坐标系 (静态 TF 父系 + docking 测量系)'),
        DeclareLaunchArgument('camera_mount_x', default_value='0.0',
                             description='相机安装位置 x (m, base_link 系, rtsp/odin 模式)'),
        DeclareLaunchArgument('camera_mount_y', default_value='0.0',
                             description='相机安装位置 y (m, base_link 系, rtsp/odin 模式)'),
        DeclareLaunchArgument('camera_mount_z', default_value='0.0',
                             description='相机安装高度 z (m, base_link 系, rtsp/odin 模式)'),
        DeclareLaunchArgument('camera_mount_yaw_deg', default_value='0.0',
                             description='相机朝向偏航 (deg, 0=正前, rtsp/odin 模式)'),
        DeclareLaunchArgument('camera_mount_pitch_deg', default_value='0.0',
                             description='相机俯仰 (deg, 正=低头, 负=抬头, rtsp/odin 模式)'),
        DeclareLaunchArgument('camera_mount_roll_deg', default_value='0.0',
                             description='相机横滚 (deg, rtsp/odin 模式)'),
        OpaqueFunction(function=launch_setup),
    ])
