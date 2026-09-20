"""Launch file for tagdocking — AprilTag dual-tag auto-docking stack.

Starts:
  1. rtsp_camera — RTSP 拉流桥 (解码 + 内参合成 + 降采样 + 静态TF),
     产出时间戳逐帧对齐的 image + camera_info 到 /camera_sync/*
  2. april_tag node (tag detection + TF broadcast)
  3. docking_node (tagdocking controller)

Usage:
  # 完整停泊栈 (机器狗默认配置: mediamtx front 流 + /odin1/odometry_highfreq):
  ros2 launch tagdocking docking.launch.py

  # 只起相机链路 (Web 看画面用, 由 docking_supervisor 按需调用):
  ros2 launch tagdocking docking.launch.py nodes:=camera

  # 换 RTSP 源 / 相机内参 / 安装位姿 (先 scripts/calibrate_rtsp 标定):
  ros2 launch tagdocking docking.launch.py \\
      rtsp_url:=rtsp://192.168.1.100:8554/live \\
      camera_info_file:=$PWD/config/rtsp_camera_info.yaml \\
      odom_topic:=/odom camera_mount_z:=0.35

  # 双二维码方案 (墙码 36h11:0 15cm + 桩码 5cm 联合对准 → 纯直行 → 距墙码
  # 0.50m 停泊): 桩码 ID=51 已确认; 全分辨率保 5cm 桩码远距可检。
  ros2 launch tagdocking docking.launch.py pile_tag_id:=51 camera_downscale:=1
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
    'reverse_step', 'lateral_step', 'yaw_step_deg', 'yaw_fine_step_deg',
    'straight_yaw_tol_deg', 'straight_yaw_max_turns',
    'prealign_tolerance_deg', 'prealign_step_deg', 'prealign_max_steps',
    'settle_sec',
    'missing_confirm_sec', 'missing_timeout_sec', 'qualification_sec',
    'dock_distance', 'dock_tolerance', 'straight_start_distance',
    'reverse_limit', 'reverse_count',
    'standoff_reverse_limit', 'standoff_reverse_count',
    'min_lateral_m', 'lateral_tolerance_m', 'exit_lateral_tolerance_m',
    'observe_timeout_sec',
    'camera_wait_sec', 'visibility_margin_px', 'visibility_sample_pad_px',
    'visibility_samples', 'score_improvement', 'feedback_min_improvement',
    'feedback_fail_windows', 'no_candidate_windows', 'log_period_sec',
    'odom_fresh_sec', 'action_timeout_sec', 'response_timeout_sec',
    'odom_noise_m', 'odom_noise_rad', 'action_startup_sec',
)

DEFAULT_RTSP_URL = 'rtsp://127.0.0.1:8555/front'

# rtsp 模式的默认内参: 写成包内绝对路径而不是留空靠回退逻辑猜。
#   - `--show-args` 里直接看得到用的是哪一份, 不用去读回退代码才敢确定;
#   - 不用相对路径 config/rtsp_camera_info.yaml —— systemd 下工作目录不确定,
#     相对路径会指到别处或直接找不到;
#   - symlink-install 让这条路径就是源码那一份 (已确认是软链), 重新标定后
#     不用 rebuild 即生效。
DEFAULT_RTSP_CAMERA_INFO = os.path.join(
    get_package_share_directory('tagdocking'), 'config', 'rtsp_camera_info.yaml')


def launch_setup(context):
    # ── 参数体检要在清场之前 ──────────────────────────────────────
    # 下面那段 pkill 有副作用: 它会收掉正在跑的 apriltag_node, 而 apriltag
    # 一死 `ros2 launch` 会连坐把整棵树带走。所以参数拼错必须在**动手之前**
    # 就炸掉 —— 否则一条 `nodes:=camrea` 的手抖就把好好跑着的栈收了, 而且自
    # 己还起不来。(实测踩过: nodes:=bogus 把活栈整棵带下来了。)
    nodes_mode = LaunchConfiguration('nodes').perform(context).strip().lower()
    if nodes_mode not in ('all', 'camera'):
        # 拼错的值当场炸, 不静默退回默认值 —— 悄悄起了满栈就把"按需省 CPU"
        # 这件事整个抵消了。
        raise RuntimeError(
            f"nodes 只能是 all 或 camera, 收到 '{nodes_mode}'")

    # ── Kill stray apriltag_node processes ──────────────────────
    # apriltag_node is a separate process from docking_node; if docking_node
    # crashed (or was SIGKILLed), the apriltag node survived. Multiple stray
    # apriltag nodes each broadcast the SAME tag TF frame (tag<family>:<id>),
    # so the TF listener returns whichever competing transform arrived last —
    # producing wild, contradictory lat/dist jumps between measurements that
    # make docking impossible. Reap it before starting a fresh set.
    try:
        subprocess.run(['pkill', '-9', '-f', 'apriltag_node'],
                       timeout=5, check=False)
    except Exception:
        pass

    family = LaunchConfiguration('family').perform(context)
    tag_size = float(LaunchConfiguration('tag_size').perform(context))
    dock_tag_id = int(LaunchConfiguration('dock_tag_id').perform(context))
    camera_frame = LaunchConfiguration('camera_frame').perform(context)
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic').perform(context)

    # 狗模式服务前缀 — 空值时用 yaml 权威值
    l1w_prefix = LaunchConfiguration('l1w_prefix').perform(context).strip()
    # 充电收尾 (charge.*) — 空值时用 yaml 权威值
    charge_enable = LaunchConfiguration('charge_enable').perform(context).strip()
    charge_passive = LaunchConfiguration('charge_passive').perform(context).strip()
    charge_static_stand = LaunchConfiguration('charge_static_stand').perform(context).strip()

    # ── RTSP 相机 ──
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

    pkg_share = get_package_share_directory('tagdocking')
    # 默认值已是包内绝对路径 (DEFAULT_RTSP_CAMERA_INFO)。显式传空串时补回来,
    # 保留"换机器/重部署不用记着传路径"的老行为。
    if not camera_info_file:
        camera_info_file = DEFAULT_RTSP_CAMERA_INFO
    # 存在性检查对显式传入的路径同样生效: 打错一个字符就在这里报, 而不是
    # 等 rtsp_camera 起不来再去翻它的日志。
    if os.path.isfile(camera_info_file):
        print(f'[docking.launch] rtsp 内参: {camera_info_file}')
    else:
        print(f'[docking.launch] 警告: 内参文件不存在: {camera_info_file} '
              '—— rtsp_camera 将启动失败。先标定: '
              'python3 scripts/calibrate_rtsp '
              '--url <rtsp地址> --out config/rtsp_camera_info.yaml',
              file=sys.stderr)

    config_path = os.path.join(pkg_share, 'config', 'docking.yaml')

    # AprilTag frame name convention: tag<family>:<id>
    tag_frame = f'tag{family}:{dock_tag_id}'

    # ── 双二维码 (dual.*) 参数解析 ───────────────────────────────
    # apriltag 节点启动即需桩码 ID/边长 (逐 tag 解 PnP + 广播 TF), 而
    # docking.yaml 是 dual.* 的权威默认 —— launch 参数空时从 yaml 读,
    # 显式传参才覆盖 (同 odom_topic 的约定)。
    try:
        with open(config_path) as f:
            _yaml_params = (yaml.safe_load(f) or {}).get(
                'docking_node', {}).get('ros__parameters', {})
    except Exception:
        _yaml_params = {}

    dual_tuning = {}
    for name in ('projection_mode', 'camera_info_topic'):
        value = LaunchConfiguration('dual_' + name).perform(context).strip()
        if value:
            dual_tuning['dual.' + name] = value
    for name in DUAL_TUNING_ARGS:
        value = LaunchConfiguration('dual_' + name).perform(context).strip()
        if value:
            number = float(value)
            if name in ('visibility_samples', 'feedback_fail_windows',
                        'no_candidate_windows', 'prealign_max_steps',
                        'reverse_count', 'standoff_reverse_count',
                        'straight_yaw_max_turns'):
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
    if pile_tag_id == dock_tag_id:
        raise RuntimeError(
            f'双码配置错误: 桩码 ID ({pile_tag_id}) 与墙码 ID '
            f'({dock_tag_id}) 相同 —— apriltag 无法区分两个同 ID tag, '
            '请传 pile_tag_id:=<其他ID> 或改 yaml dual.pile_tag_id')
    # 5cm 桩码在降采样图上 1m 外低于 36h11 检测下限 → 建议全分辨率
    if camera_downscale not in (0, 1):
        print(f'[docking.launch] 警告: 双码模式下 5cm 桩码在降采样 '
              f'({camera_downscale}x) 图上 1m 外不可检, 建议 '
              'camera_downscale:=1 (全分辨率)', file=sys.stderr)

    # 同步桥输出话题。apriltag_ros 的 image_transport::CameraSubscriber 从 image
    # 话题名同级派生 camera_info (image_raw → camera_info), 所以 apriltag 必须订阅
    # 桥的 image 输出, 才能拿到时间戳对齐的 camera_info。
    sync_image_topic = '/camera_sync/image_raw'
    sync_info_topic = '/camera_sync/camera_info'

    nodes = []

    # ── RTSP 相机桥 ────────────────────────────────────────────
    # 直接产出时间戳逐帧对齐的 image + camera_info 到 sync 话题 (apriltag
    # 订阅口不变), 并发布 base_frame→相机光学系 静态 TF (安装位姿 mount.*)。
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

    if nodes_mode == 'camera':
        # ── camera 模式在此收尾 ──────────────────────────────────
        # 只要相机链路: rtsp_camera (含它自己发的 mount 静态 TF), 不起检测
        # 器也不起控制器。供 Web 在停泊栈没起时单纯看画面用 —— 看画面不需
        # 要检测, 而 apriltag 那份 decimate=1.0 的逐帧检测正是这次要省掉的
        # 开销大头。用提前 return 而不是把下面两段包进 if: 相机链路本来就
        # 全在上面, 早退是顺着结构来的。
        return nodes

    # ── AprilTag detection node ─────────────────────────────────
    # 双码方案: 墙码 + 桩码两 tag 逐 tag 边长 (嵌套 tag.sizes; 桩码 5cm 与
    # 墙码 15cm 边长不同, 旧 fork 的单一 size 参数解不出桩码正确 PnP ——
    # 本包 deps.repos 锁定 apriltag_ros master/3.4.0+, 支持嵌套逐 tag 边长)。
    apriltag_ids = [dock_tag_id]
    apriltag_frames = [tag_frame]
    apriltag_sizes = [wall_tag_size]
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
            },
            # odom_topic 为空时用 yaml 值 (yaml 权威): 只有显式传参
            # 才覆盖, 避免无参启动时 launch 默认值悄悄顶掉 yaml 里的配置。
        ] + ([{'odom_topic': odom_topic}] if odom_topic else [])
          + ([{'base.l1w_prefix': l1w_prefix}] if l1w_prefix else [])
          + ([{'charge.enable': charge_enable.lower() == 'true'}]
             if charge_enable else [])
          + ([{'charge.passive': charge_passive.lower() == 'true'}]
             if charge_passive else [])
          + ([{'charge.static_stand': charge_static_stand.lower() == 'true'}]
             if charge_static_stand else [])
          # 双码: launch 解析后的最终值显式覆盖 yaml —— 用户 launch
          # 传参 (pile_tag_id:= 等) 必须同时作用于 apriltag 与 docking_node
          # 两侧, 否则节点找的 TF frame 名与 apriltag 广播的对不上。
          + [dual_tuning, {'dual.wall_tag_size': wall_tag_size,
               'dual.pile_tag_id': pile_tag_id,
               'dual.pile_tag_size': pile_tag_size}],
        output='screen',
    ))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('dual_projection_mode', default_value='',
            description='raw or rectified; empty uses YAML'),
        DeclareLaunchArgument('dual_camera_info_topic', default_value='',
            description='Exact detection-image CameraInfo; empty preserves YAML'),
        *[DeclareLaunchArgument('dual_' + name,
            default_value='',
            description='Override dual.' + name + '; empty preserves YAML')
          for name in DUAL_TUNING_ARGS],
        DeclareLaunchArgument('family', default_value='36h11',
                             description='AprilTag family (36h11, 25h9, etc.)'),
        DeclareLaunchArgument('tag_size', default_value='0.15',
                             description='Tag edge size in meters (墙码实测 0.15)'),
        DeclareLaunchArgument('dock_tag_id', default_value='0',
                             description='Tag ID to dock to'),
        DeclareLaunchArgument('camera_frame',
                             default_value='camera_color_optical_frame',
                             description='Camera optical frame name'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='cmd_vel_dock',
                             description='Velocity command topic'),

        # ── 狗模式服务前缀 + 充电收尾 ─────────────────────────────
        DeclareLaunchArgument('l1w_prefix', default_value='',
                              description='狗模式服务前缀 (空 = 使用 yaml 的 '
                                          'base.l1w_prefix; 默认 /l1w_control)'),
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

        # ── 双二维码 (dual.*) ────────────────────────────────────
        DeclareLaunchArgument('wall_tag_size', default_value='',
                              description='墙码边长 (m, 36h11:0): apriltag 按此解 PnP '
                                          '(空 = 使用 yaml 的 dual.wall_tag_size, 默认 0.15)'),
        DeclareLaunchArgument('pile_tag_id', default_value='',
                              description='桩码 ID (贴充电桩底座, 现场已确认 51): '
                                          '不得与 dock_tag_id 相同 '
                                          '(空 = 使用 yaml 的 dual.pile_tag_id, 默认 51)'),
        DeclareLaunchArgument('pile_tag_size', default_value='',
                              description='桩码边长 (m): '
                                          '(空 = 使用 yaml 的 dual.pile_tag_size, 默认 0.05)'),

        # ── RTSP 相机 ───────────────────────────────────────────
        DeclareLaunchArgument('rtsp_url',
                             default_value=DEFAULT_RTSP_URL,
                             description='RTSP 地址, 默认指向 mediamtx 的 front 通道 '
                                         '(实测默认配置)'),
        DeclareLaunchArgument('camera_info_file', default_value=DEFAULT_RTSP_CAMERA_INFO,
                             description='相机内参 YAML (默认=包内 rtsp 标定文件绝对路径, '
                                         '由 scripts/calibrate_rtsp 生成)'),
        DeclareLaunchArgument('camera_downscale', default_value='0',
                             description='输出降采样倍数 (0=自动: RTSP 到 ~640 宽)。'
                                         '全分辨率传 1'),
        DeclareLaunchArgument('camera_backend', default_value='ffmpeg',
                             description='RTSP 拉流后端: ffmpeg | gstreamer '
                                         '(Jetson 硬解, FFmpeg 解码冻结时用)'),
        DeclareLaunchArgument('nodes', default_value='all',
                             description='起哪些节点: all = 相机+检测+控制器 (默认); '
                                         'camera = 只起相机链路 (rtsp_camera), 供 Web '
                                         '在停泊栈未起时看画面, 由 docking_supervisor '
                                         '按需使用'),
        DeclareLaunchArgument('odom_topic',
                             default_value='/odin1/odometry_highfreq',
                             description='机器狗本体里程计 (l1w_control 发布; '
                                         '空 = 使用 config/docking.yaml 的 odom_topic)'),
        DeclareLaunchArgument('base_frame', default_value='base_link',
                             description='机器人基座坐标系 (静态 TF 父系 + docking 测量系)'),
        DeclareLaunchArgument('camera_mount_x', default_value='0.0',
                             description='相机安装位置 x (m, base_link 系)'),
        DeclareLaunchArgument('camera_mount_y', default_value='0.0',
                             description='相机安装位置 y (m, base_link 系)'),
        DeclareLaunchArgument('camera_mount_z', default_value='0.0',
                             description='相机安装高度 z (m, base_link 系)'),
        DeclareLaunchArgument('camera_mount_yaw_deg', default_value='0.0',
                             description='相机朝向偏航 (deg, 0=正前)'),
        DeclareLaunchArgument('camera_mount_pitch_deg', default_value='0.0',
                             description='相机俯仰 (deg, 正=低头, 负=抬头)'),
        DeclareLaunchArgument('camera_mount_roll_deg', default_value='0.0',
                             description='相机横滚 (deg)'),
        OpaqueFunction(function=launch_setup),
    ])
