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

  # 双二维码模式 (墙码 36h11:0 15cm + 桩码 5cm 联合对准 → 纯直行 → 距墙码
  # 0.50m 停泊): 桩码 ID=51 已确认; 全分辨率保 5cm 桩码远距可检。
  ros2 launch tagdocking docking.launch.py dual_enable:=true \\
      pile_tag_id:=51 camera_downscale:=1
"""

import os
import subprocess
import sys
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


DUAL_TUNING_ARGS = (
    'observation_distance', 'observation_tolerance', 'forward_step',
    'reverse_step', 'lateral_step', 'yaw_step_deg', 'settle_sec',
    'missing_confirm_sec', 'missing_timeout_sec', 'qualification_sec',
    'dock_distance', 'dock_tolerance', 'straight_start_distance',
    'min_lateral_m', 'lateral_tolerance_m', 'observe_timeout_sec',
    'camera_wait_sec', 'visibility_margin_px', 'visibility_sample_pad_px',
    'visibility_samples', 'score_improvement', 'feedback_min_improvement',
    'feedback_fail_windows', 'no_candidate_windows', 'log_period_sec',
    'odom_fresh_sec', 'action_timeout_sec', 'response_timeout_sec',
    'odom_noise_m', 'odom_noise_rad',
    'pile_lock_distance', 'crouch_settle_sec',
)


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
    # 直行入口横向门槛 — 空值时用 yaml 权威值 (同 jog_max 的约定)
    entry_lateral_m = LaunchConfiguration('entry_lateral_m').perform(context).strip()
    # 相机安装横向偏移补偿 (m, + = 相机偏左) — 空值时用 yaml 权威值
    camera_lateral_offset_m = LaunchConfiguration('camera_lateral_offset_m').perform(context).strip()
    # 近场横移修正/捷径横向门槛 (m) — 空值时用 yaml 权威值
    final_straight_lateral_threshold = LaunchConfiguration('final_straight_lateral_threshold').perform(context).strip()
    # 近场法线对准门槛 (deg) — 空值时用 yaml 权威值
    final_straight_normal_yaw_deg = LaunchConfiguration('final_straight_normal_yaw_deg').perform(context).strip()
    # 远/近分界 (m): dist ≤ 此值近场精调, > 此值远场粗对准+纯前进 — 空值时用 yaml 权威值
    final_straight_tighten_distance = LaunchConfiguration('final_straight_tighten_distance').perform(context).strip()
    # 远场粗对准方位门槛 (deg) — 空值时用 yaml 权威值
    final_straight_far_yaw_deg = LaunchConfiguration('final_straight_far_yaw_deg').perform(context).strip()
    # 远场粗对准横向门槛 (m) — 空值时用 yaml 权威值
    final_straight_far_lateral_m = LaunchConfiguration('final_straight_far_lateral_m').perform(context).strip()
    # 到位即 DOCKED 的方位门槛 (deg) — 空值时用 yaml 权威值
    final_servo_yaw_deg = LaunchConfiguration('final_servo_yaw_deg').perform(context).strip()
    # 走停单步最大 jog 距离 — 空值时用 yaml 权威值 (同 base_type 的约定)
    jog_max = LaunchConfiguration('jog_max').perform(context).strip()

    # 静止站立 (posture.*) — 空值时用 yaml 权威值
    l1w_prefix = LaunchConfiguration('l1w_prefix').perform(context).strip()
    posture_enable = LaunchConfiguration('posture_enable').perform(context).strip()
    static_settle_sec = LaunchConfiguration('static_settle_sec').perform(context).strip()
    # 充电收尾 (charge.*) — 空值时用 yaml 权威值
    charge_enable = LaunchConfiguration('charge_enable').perform(context).strip()
    charge_passive = LaunchConfiguration('charge_passive').perform(context).strip()
    charge_static_stand = LaunchConfiguration('charge_static_stand').perform(context).strip()

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

    # ── 双二维码模式 (dual.*) 参数解析 ───────────────────────────
    # apriltag 节点启动即需桩码 ID/边长 (逐 tag 解 PnP + 广播 TF), 而
    # docking.yaml 是 dual.* 的权威默认 —— launch 参数空时从 yaml 读,
    # 显式传参才覆盖 (与 base_type 同款约定)。
    try:
        with open(config_path) as f:
            _yaml_params = (yaml.safe_load(f) or {}).get(
                'docking_node', {}).get('ros__parameters', {})
    except Exception:
        _yaml_params = {}

    dual_enable = LaunchConfiguration('dual_enable').perform(context).strip()
    if dual_enable == '':
        dual_enable = 'true' if _yaml_params.get('dual.enable') else 'false'
    if dual_enable.lower() not in ('true', 'false'):
        raise ValueError('dual_enable must be true or false')
    dual_enabled = dual_enable.lower() == 'true'
    dual_tuning = {}
    for name in ('projection_mode', 'camera_info_topic'):
        value = LaunchConfiguration('dual_' + name).perform(context).strip()
        if value:
            dual_tuning['dual.' + name] = value
    # Select rectified only for the known undistorted Odin source, never merely
    # because apriltag's subscription happens to be named image_rect.
    if 'dual.projection_mode' not in dual_tuning and use_odin and image_topic == '/odin1/image/undistorted':
        dual_tuning['dual.projection_mode'] = 'rectified'
    for name in DUAL_TUNING_ARGS:
        value = LaunchConfiguration('dual_' + name).perform(context).strip()
        if value:
            number = float(value)
            if name in ('visibility_samples', 'feedback_fail_windows', 'no_candidate_windows'):
                if not number.is_integer():
                    raise ValueError('dual_' + name + ' must be an integer')
                number = int(number)
            dual_tuning['dual.' + name] = number
    # 墙码边长: 空 = yaml dual.wall_tag_size, 再退 tag_size 参数
    _arg = LaunchConfiguration('wall_tag_size').perform(context).strip()
    wall_tag_size = (float(_arg) if _arg
                     else float(_yaml_params.get('dual.wall_tag_size')
                                or tag_size))
    # 桩码 ID: 空 = yaml dual.pile_tag_id
    _arg = LaunchConfiguration('pile_tag_id').perform(context).strip()
    pile_tag_id = (int(_arg) if _arg
                   else int(_yaml_params.get('dual.pile_tag_id') or 51))
    # 桩码边长: 空 = yaml dual.pile_tag_size
    _arg = LaunchConfiguration('pile_tag_size').perform(context).strip()
    pile_tag_size = (float(_arg) if _arg
                     else float(_yaml_params.get('dual.pile_tag_size')
                                or 0.05))

    pile_frame = f'tag{family}:{pile_tag_id}'
    if dual_enabled:
        if pile_tag_id == dock_tag_id:
            raise RuntimeError(
                f'双码配置错误: 桩码 ID ({pile_tag_id}) 与墙码 ID '
                f'({dock_tag_id}) 相同 —— apriltag 无法区分两个同 ID tag, '
                '请传 pile_tag_id:=<其他ID> 或改 yaml dual.pile_tag_id')
        # 5cm 桩码在降采样图上 1m 外低于 36h11 检测下限 → 双码建议全分辨率
        if use_rtsp:
            _eff_scale = camera_downscale
        elif use_odin:
            _eff_scale = odin_downscale
        else:
            _eff_scale = 1
        if _eff_scale != 1:
            print(f'[docking.launch] 警告: 双码模式下 5cm 桩码在降采样 '
                  f'({_eff_scale}x) 图上 1m 外不可检, 建议 '
                  'camera_downscale:=1 (全分辨率)', file=sys.stderr)

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
            arguments=['--ros-args', '--log-level', 'rtsp_camera:=error'],
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
    # 双码模式: 墙码 + 桩码两 tag 逐 tag 边长 (嵌套 tag.sizes; 桩码 5cm 与
    # 墙码 15cm 边长不同, 旧 fork 的单一 size 参数解不出桩码正确 PnP ——
    # 本包 deps.repos 锁定 apriltag_ros master/3.4.0+, 支持嵌套逐 tag 边长)。
    apriltag_ids = [dock_tag_id]
    apriltag_frames = [tag_frame]
    apriltag_sizes = [wall_tag_size if dual_enabled else tag_size]
    if dual_enabled:
        apriltag_ids.append(pile_tag_id)
        apriltag_frames.append(pile_frame)
        apriltag_sizes.append(pile_tag_size)
    nodes.append(Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_node',
        parameters=[{
            'family': family,
            'size': apriltag_sizes[0],
            'tag_ids': apriltag_ids,
            'tag_frames': apriltag_frames,
            # apriltag_ros 3.4.0+ (上游 ROS2 重写, 节点名 /apriltag) 只认嵌套
            # tag.ids/tag.frames/tag.sizes (且无 publish_tf/z_up, TF 无条件
            # 发布), 旧 fork 只认扁平 tag_ids/tag_frames。未识别的参数被静默
            # 忽略, 两套同传兼容两代版本 —— 只传扁平参数时 3.4.0 会"有检测、
            # 无 TF"(tag 不在配置里, 不解算位姿也不广播 TF)。
            'tag.ids': apriltag_ids,
            'tag.frames': apriltag_frames,
            'tag.sizes': apriltag_sizes,
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
                'tag.size': apriltag_sizes[0],
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
          + ([{'stopgo.jog_max': float(jog_max)}] if jog_max else [])
          + ([{'final_straight.entry_lateral_m': float(entry_lateral_m)}]
             if entry_lateral_m else [])
          + ([{'camera.lateral_offset_m': float(camera_lateral_offset_m)}]
             if camera_lateral_offset_m else [])
          + ([{'final_straight.lateral_threshold_m': float(final_straight_lateral_threshold)}]
             if final_straight_lateral_threshold else [])
          + ([{'final_straight.normal_yaw_threshold_deg': float(final_straight_normal_yaw_deg)}]
             if final_straight_normal_yaw_deg else [])
          + ([{'final_straight.tighten_distance': float(final_straight_tighten_distance)}]
             if final_straight_tighten_distance else [])
          + ([{'final_straight.far_yaw_threshold_deg': float(final_straight_far_yaw_deg)}]
             if final_straight_far_yaw_deg else [])
          + ([{'final_straight.far_lateral_m': float(final_straight_far_lateral_m)}]
             if final_straight_far_lateral_m else [])
          + ([{'final_servo.yaw_tol_deg': float(final_servo_yaw_deg)}]
             if final_servo_yaw_deg else [])
          + ([{'charge.enable': charge_enable.lower() == 'true'}]
             if charge_enable else [])
          + ([{'charge.passive': charge_passive.lower() == 'true'}]
             if charge_passive else [])
          + ([{'charge.static_stand': charge_static_stand.lower() == 'true'}]
             if charge_static_stand else [])
          # 双码模式: launch 解析后的最终值显式覆盖 yaml —— 用户 launch
          # 传参 (pile_tag_id:= 等) 必须同时作用于 apriltag 与 docking_node
          # 两侧, 否则节点找的 TF frame 名与 apriltag 广播的对不上。
          # dual_enable=false 时零改动 (yaml 权威, 节点全程旁路)。
          + ([dual_tuning, {'dual.enable': dual_enabled,
               'dual.wall_tag_size': wall_tag_size,
               'dual.pile_tag_id': pile_tag_id,
               'dual.pile_tag_size': pile_tag_size}]
             ),
        output='screen',
    ))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('dual_projection_mode', default_value='',
            description='raw or rectified; empty uses YAML (known Odin undistorted source selects rectified)'),
        DeclareLaunchArgument('dual_camera_info_topic', default_value='',
            description='Exact detection-image CameraInfo; empty preserves YAML'),
        *[DeclareLaunchArgument('dual_' + name, default_value='',
            description='Override dual.' + name + '; empty preserves YAML')
          for name in DUAL_TUNING_ARGS],
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
        DeclareLaunchArgument('final_straight_yaw_deg', default_value='3.0',
                             description='直行入口方位门槛 (deg): 进入直行距离时方位误差超此值报导航失败; '
                                         '近场(dist ≤ tighten_distance)的方位修正门槛同此值'),
        DeclareLaunchArgument('entry_lateral_m', default_value='',
                             description='直行入口横向门槛 (m): 进入直行距离时 |横向| 超此值报导航失败 '
                                         '(空 = 使用 yaml 的 final_straight.entry_lateral_m)'),
        DeclareLaunchArgument('camera_lateral_offset_m', default_value='',
                             description='相机光学中心相对底盘中心线的横向偏移 (m, + = 相机偏左): '
                                         '加回量测 lat 补偿安装误差, 直行前触发左移修正 '
                                         '(空 = 使用 yaml 的 camera.lateral_offset_m; 本机实测 0.03)'),
        DeclareLaunchArgument('final_straight_lateral_threshold', default_value='',
                             description='近场横移修正/捷径横向门槛 (m, 比入口 entry_lateral_m 更紧): '
                                         '量测补偿加回偏置后防止真实偏移被捷径放行直行 '
                                         '(空 = 使用 yaml 的 final_straight.lateral_threshold_m)'),
        DeclareLaunchArgument('final_straight_normal_yaw_deg', default_value='',
                             description='近场法线(normal)对准门槛 (deg): 收紧到 ~2° 让先对齐法线再横移成立; '
                                         '之前用 stopgo.yaw_threshold_deg(10°) 太松导致横移走错方向 '
                                         '(空 = 使用 yaml 的 final_straight.normal_yaw_threshold_deg; 摆头可放宽 3~5°)'),
        DeclareLaunchArgument('final_straight_tighten_distance', default_value='',
                              description='远/近分界 (m): dist ≤ 此值进入近场精调(法线对准+横移+直行), '
                                          '> 此值远场粗对准+纯前进 (1.5m 处 normal 噪声放大, 远场不横移) '
                                          '(空 = 使用 yaml 的 final_straight.tighten_distance; 默认 1.3)'),
        DeclareLaunchArgument('final_straight_far_yaw_deg', default_value='',
                              description='远场粗对准方位门槛 (deg): dist > tighten_distance 时方位 ≤ 此值即前进, '
                                          '不做微调/横移 (空 = 使用 yaml 的 final_straight.far_yaw_threshold_deg; '
                                          '默认 15.0)'),
        DeclareLaunchArgument('final_straight_far_lateral_m', default_value='',
                              description='远场粗对准横向门槛 (m): dist > tighten_distance 时 |lat| ≤ 此值即前进, '
                                          '横向偏走近场再修 (空 = 使用 yaml 的 final_straight.far_lateral_m; '
                                          '默认 0.20)'),
        DeclareLaunchArgument('final_servo_yaw_deg', default_value='',
                              description='到位即 DOCKED 的方位门槛 (deg): 直行到位后方位偏差 '
                                          '≤ 此值即判定成功, 不再累积稳定/追角 '
                                          '(空 = 使用 yaml 的 final_servo.yaw_tol_deg)'),
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
        DeclareLaunchArgument('charge_enable', default_value='',
                              description='充电收尾总开关 true/false '
                                          '(空 = 使用 yaml 的 charge.enable)'),
        DeclareLaunchArgument('charge_passive', default_value='',
                              description='阻尼(泄力)步开关 true/false; false=仅锁定 '
                                          '(空 = 使用 yaml 的 charge.passive)'),
        DeclareLaunchArgument('charge_static_stand', default_value='',
                              description='DOCKED 后先 static_stand 锁定再阻尼 true/false; '
                                          'false=跳过锁定直接阻尼 '
                                          '(空 = 使用 yaml 的 charge.static_stand)'),

        # ── 双二维码模式 (dual.*) ────────────────────────────────
        DeclareLaunchArgument('dual_enable', default_value='',
                              description='双二维码对准总开关 true/false: 墙码(36h11:0)+桩码 '
                                          '联合对准 → 纯直行 → 距墙码 0.50m 停泊 '
                                          '(空 = 使用 yaml 的 dual.enable)'),
        DeclareLaunchArgument('wall_tag_size', default_value='',
                              description='墙码边长 (m, 36h11:0): dual 启用时 apriltag 按此解 PnP '
                                          '(空 = 使用 yaml 的 dual.wall_tag_size, 默认 0.15)'),
        DeclareLaunchArgument('pile_tag_id', default_value='',
                              description='桩码 ID (贴充电桩底座, 现场已确认 51): '
                                          '不得与 dock_tag_id 相同 '
                                          '(空 = 使用 yaml 的 dual.pile_tag_id, 默认 51)'),
        DeclareLaunchArgument('pile_tag_size', default_value='',
                              description='桩码边长 (m): '
                                          '(空 = 使用 yaml 的 dual.pile_tag_size, 默认 0.05)'),

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
