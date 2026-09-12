# tagdocking — AprilTag 自动停靠框架

工业级 ROS2 Humble AprilTag 视觉自动停靠系统，支持多底盘（差速/全向/四足）、**走停式（Stop-and-Go）盲动机动 + 里程计航位推算**、相机时间戳同步（ROS 相机话题 / RTSP 视频流两种接入）、完整状态机、失败自动倒车重试与泊出（Undock）。停靠由外部服务直接触发（`/docking_node/start_docking`），预停靠导航由调用方负责把机器人送入 Tag 范围。

---

## 目录

1. [系统架构](#1-系统架构)
2. [快速开始](#2-快速开始)
3. [触发方式](#3-触发方式)
4. [状态机详解](#4-状态机详解)
5. [底盘适配器](#5-底盘适配器)
6. [参数配置](#6-参数配置)
7. [调试方法](#7-调试方法)
8. [仿真测试](#8-仿真测试)
9. [注意事项](#9-注意事项)
10. [附录 A：全部可调参数速查表](#10-附录-a全部可调参数速查表)

---

## 1. 系统架构

### 整体流程

```
              开始停靠 (/docking_node/start_docking)
                             |
                             v
                      [SEARCH_TAG]
              角度步进扫描: 转30°(里程计闭环)→停稳→检测→再转
                             | 锁定 (持续可见 0.5s)
                             v
                       [APPROACH]                 ← ALIGN 已并入此阶段 (遗留态, 跳过)
            走停式两阶段逼近 (盲转-盲走-盲转 / 纯直行)
                 阶段1: 法线机动对准  阶段2: ≤0.85m 纯直行不调角
                        | 距目标 ≤ 2×位置容差        | 直行入口对准过差
                        v                           v
                  [FINAL_SERVO]                 [RETRYING]
              到位确认 (距离±5cm+方位≤15° 单帧DOCKED)  盲退 0.5m 重新锁定 (≤2次)
                        |                           → 回 SEARCH_TAG
                        v
                      [DOCKED] ✓
                             |
                  泊出 (/docking_node/start_undock)
                             v
                     [UNDOCKING]
              盲退 0.5m → 原地转 180° (纯里程计)
                             v
                     [UNDOCKED] ✓
```

> 停靠节点本身不负责把机器人导航到 Tag 附近——由外部服务（Nav2 / 业务节点 /
> 遥控）把机器人送到 Tag 视野内后，再调用 `/docking_node/start_docking` 进入
> 上面的视觉停靠流程。

### 走停式（Stop-and-Go）核心思想

**运动期间从不相信相机，静止时从不相信里程计。** 每一步机动是：

```
停稳 → 静止站立锁定 (posture.*，身体不喘) → settle 等图像清晰 + 稳定帧
  → 采一帧新鲜测量 → 规划器算出完整小段路径 → stand_up 恢复运动
  → 冻结检测 → 里程计闭环盲动到目标 → 停稳 → …(循环)
```

- 机动期（盲转/盲走）所有检测帧被**冻结丢弃**（`_frozen` 门控）：运动模糊和
  视野边缘的坏帧绝不能污染规划用的位姿；tag 中途出视野是预期行为，不算丢失
- 每个动作（前进/转向/横移）由 `ActionExecutor` 用**里程计位移闭环**掐断，
  不依赖时间估算，无加速曲线带来的过冲
- 单步前进上限 `stopgo.jog_max`（0.15m）：步子小、运动期短，低帧率（6Hz）
  相机下 tag 也不易因模糊丢失
- settle 窗口（`stopgo.turn_settle_sec`，0.8s）结束后**清空全部旧位姿**并重新
  播种 EMA 滤波——规划永远只用停稳后的新鲜帧

### 静止站立 × 呼吸抑制 (posture.*) — 停看循环升级

狗的站立步态会"呼吸"（身体持续微幅起伏），tag 位姿因此始终带抖动。每个停看
点的"停"由 `PostureMode`（`tagdocking/posture_mode.py`）升级为完整闭环：

```
停 → /l1w_control/static_stand (CMD_LOCK_MODE, 身体锁定不喘)
   → 等 posture_state==static_stand 且距停稳 ≥ posture.static_settle_sec
   → 解冻清空旧位姿 → 等待 ≥ posture.min_stable_frames 帧连续 accepted
   → 量测 + 重规划 (锁定下进行, 位姿最稳)
   → /l1w_control/stand_up 恢复运动模式 → 等 motion_enabled==True → 起步
```

- **所有停看点统一**：APPROACH 每步、丢 tag 搜索步、失败重试、泊出入口；
  盲链子步之间（turn-drive-turn 内部）不停不锁
- **解锁确认以 `/l1w_control/motion_enabled` 为准**——它就是桥的 cmd_vel
  门控本身（`authority && motion_enabled_ && !static_stand`），固件退出 LOCK
  有滞后，只有 Bool 变 True 才真的能走
- **DOCKED 保持静止站立**（不呼吸、姿态最稳），随后进入充电收尾（见
  §3.2）；泊出自动先 stand_up；取消/超时/失败等错误终态自动恢复运动模式
  把狗还给遥控
- **机动中被外部切入静止站立**（如网页台"静止站立"按钮）→ 立即
  MOTION_FAILED（cmd_vel 已死，不静默耗完超时）；判定要求 motion_enabled
  为 False，避免解锁瞬间 posture_state 滞后残留误判
- **降级语义**：`/l1w_control` 服务不存在（备用桥 zsibot_bridge / 纯台架）→
  `service_wait_sec` 宽限后自动停用并告警一次，行为退回纯感知侧 settle；
  static_stand 确认超时 → 本次停降级为未锁定继续；stand_up 失败不可降级
  （狗还锁着）→ 重试 `unlock_retries` 次后 MOTION_FAILED
- **每停多花 ~1.5-2.5s**（锁定 + settle + 稳定帧 + 解锁），`timeout_sec` 已
  相应上调（120→180、approach 60→90、search 60→120）
- **节点退出/急停不自动 stand_up**——锁定站立是最安全的姿态；恢复用
  `ros2 service call /l1w_control/stand_up std_srvs/srv/Trigger '{}'`
  或网页台"站起"按钮
- 台架无狗调试：`scripts/mock_l1w_control` 模拟模式接口 + cmd_vel 门控 +
  里程计积分，可跑通完整走停闭环（见 §8）

### 模块结构

```
tagdocking/
├── action/Dock.action           # ROS2 Action 定义
├── config/docking.yaml          # 全部参数
├── config/rtsp_camera_info_example.yaml  # RTSP 相机内参示例
├── launch/docking.launch.py     # 启动文件 (相机源: ROS话题桥 / RTSP桥 / Odin1 三选一)
├── scripts/
│   ├── docking_node             # 停泊主节点入口
│   ├── camera_info_bridge       # ROS 相机时间戳同步桥入口
│   ├── rtsp_camera              # RTSP 相机桥入口 (机器狗)
│   ├── calibrate_camera         # 棋盘格标定 (ROS 相机, ssh -X)
│   ├── calibrate_rtsp           # 棋盘格标定 (RTSP, 纯 ssh 无 GUI)
│   ├── test_apriltag            # 相机直测 tag 距离/横向 (验证内参+TF)
│   ├── test_turn_angle          # 转角精度测试 (cmd vs 里程计)
│   ├── test_jog_distance        # 直行精度测试 (cmd vs 里程计)
│   └── mock_l1w_control         # 台架替身: 模拟狗模式接口+门控+里程计
├── tagdocking/
│   ├── docking_node.py          # 主节点 — 20Hz 控制循环, 集成所有子系统
│   ├── state_machine.py         # 状态机 + 超时/重试/泊出管理
│   ├── posture_mode.py          # 静止站立(锁定)管理器 — 停看循环 停→锁→稳→规划→解锁→走
│   ├── charge_mode.py           # 充电收尾管理器 — DOCKED 后 静止(锁定)→阻尼
│   ├── geometry_planner.py      # 几何规划器 — 法线机动(turn-drive-turn)/两阶段直行
│   ├── dual_docking.py          # 双二维码对准 — 墙码+桩码视差解算/对准保持/纯直行相位
│   ├── action_executor.py       # 动作执行器 — 里程计航位推算闭环
│   ├── camera_info_bridge.py    # 相机话题时间戳同步桥 (ROS 相机模式 + Odin1 内参合成模式)
│   ├── rtsp_camera.py           # RTSP→ROS 相机桥 (机器狗模式: 拉流+内参+静态TF)
│   ├── pose_buffer.py           # 时间戳位姿缓冲 (自适应时效窗口)
│   ├── pid_controller.py        # PID (含 anti-windup) — 遗留, 主流程未使用
│   ├── utils.py                 # 角度/四元数/Tag 位姿类型
│   └── base_adapter/
│       ├── base_adapter.py      # 抽象基类: publish_jog / publish_turn / publish_stop
│       ├── diff_drive.py        # 差速轮 (linear.x + angular.z)
│       ├── omni.py              # 全向轮 (+ publish_lateral 横移)
│       └── quadruped.py         # 四足 SDK 回调桥接 (+ 横移)
└── CMakeLists.txt
```

### 数据流

```
/detections ──► _on_detections (机动期冻结丢弃)
                     │ 命中目标 tag id
                     v
          TF lookup (base_link→tag36h11:0, REP-103)
                     │ 跳变拒绝 + EMA 平滑 (normal 用圆周 EMA)
                     v
     PoseBuffer (时效窗口 = max(150ms, 实测检测间隔×3), 自适应)
                     │
                     ▼
        20Hz 控制循环: 状态机 evaluate (转移/超时/丢tag重锁)
                     │
    GeometryPlanner ◄┘► ActionExecutor ◄── /odom (航位推算闭环)
   (法线机动/直行规划)     (jog/turn/lateral 常速盲动)
                     │
                BaseAdapter
                     │
              ┌──────┼───────┐
              ▼      ▼       ▼
         DiffDrive  Omni  Quadruped
              │      │       │
              ▼      ▼       ▼
          /cmd_vel /cmd_vel  SDK move(vx,vy,wz)
```

相机侧（三选一，由 launch 的 `rtsp_url` / `use_odin` 参数决定）：

```
ROS 相机模式:  相机驱动(/image_raw+/camera_info) ─► camera_info_bridge ─► /camera_sync/*
RTSP 模式:     rtsp_camera (拉流+内参合成+静态TF) ─────────────────────► /camera_sync/*
Odin1 模式:    /odin1/image/undistorted ─► camera_info_bridge(合成模式: 内参合成
                              +frame_id重打+静态TF) ─► /camera_sync/image_raw + /camera_sync/camera_info
                                        └► apriltag_node ─► /detections + TF
```

---

## 2. 快速开始

### 2.1 前置条件

- ROS2 Humble 已安装
- `apriltag_ros` 已安装并能正常检测 Tag
- 相机接入三选一：
  - **ROS 相机模式**（差速车等）：相机驱动发布 `/image_raw` 和 `/camera_info`，且已标定
  - **RTSP 模式**（机器狗）：相机提供 RTSP 流，先用 `scripts/calibrate_rtsp` 标定（见 2.6）
  - **Odin1 模式**：odin 驱动发布 `/odin1/image/undistorted`（去畸变流），内参用包内
    `config/odin_camera_info.yaml`（见 2.7）
- Tag 贴在停靠目标上，且 TF 树连通（`base_link → 相机光学系 → tag36h11:0`）
- 机器人发布里程计（默认话题 `/odom_combined`，RTSP/机器狗用 `odom_topic` 参数指定）
- 机器人已被外部服务（Nav2 / 业务节点 / 遥控）送到 Tag 视野范围内

### 2.2 编译

```bash
cd ~/ros2_ws
colcon build --packages-select tagdocking --symlink-install
source install/setup.bash
```

### 2.3 启动

```bash
# 终端 1: 启动停靠系统
ros2 launch tagdocking docking.launch.py \
    base_type:=diff_drive

# 终端 2: 先启动你的机器人底层驱动 (若未启动)
# ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py

# 终端 3: 确保 apriltag_ros 有图像输入 (若 launch 中的 camera 话题不匹配)
# ros2 run tagdocking camera_info_bridge
```

> **注意**: launch 会自动启动 `camera_info_bridge` 和 `apriltag_node`：
> 桥订阅 `/image_raw` + `/camera_info`（话题名可用 `image_topic` /
> `camera_info_topic` 参数改），把二者时间戳逐帧对齐后（附降采样）重发到
> `/camera_sync/image_raw` + `/camera_sync/camera_info`；`apriltag_node` 订阅
> 同步图像并发布 `/detections` + TF。若相机话题不是 `/image_raw`，通过 launch
> 参数指定即可。

> 停泊节点只做视觉停靠。若需要先把机器人导航到 Tag 附近，由外部 Nav2 / 业务
> 节点完成，到位后再触发 `/docking_node/start_docking`。

### 2.4 底盘类型选择

```bash
# 差速轮 (默认) — 只输出 linear.x + angular.z, 横向误差靠"转向-前进-转向"机动消除
ros2 launch tagdocking docking.launch.py base_type:=diff_drive

# 全向轮 / Mecanum — 额外支持 linear.y 横移, 横偏直接平移消除
ros2 launch tagdocking docking.launch.py base_type:=omni

# 四足机器人 — SDK 回调桥接 (不发 cmd_vel)
# 注意: docking_node 构造 QuadrupedAdapter 时未注入 move_callback,
# 直接选此类型命令是空操作 —— 需在代码中接入 SDK (见 §5.4)。
# cmd_vel 驱动的机器狗请直接用 base_type:=omni (见 2.6)。
```

### 2.5 自定义 Tag

```bash
ros2 launch tagdocking docking.launch.py \
    dock_tag_id:=5 \
    tag_size:=0.21 \
    family:=36h11 \
    camera_frame:=camera_color_optical_frame
```

### 2.6 机器狗部署 (RTSP 相机 + cmd_vel)

机器狗用 `/cmd_vel` 驱动（与差速车相同，`base_type:=omni` 即可，狗支持
横移故用全向适配器），但相机只提供 **RTSP 视频流**，没有 ROS 相机驱动——
即没有 image 话题、没有 camera_info 内参、没有 `base_link→相机` TF。
`rtsp_camera` 桥节点一次补齐这三样，下游 `apriltag_node → docking_node`
管线与差速车完全一致：

```
rtsp_camera ─→ /camera_sync/image_raw + /camera_sync/camera_info (时间戳逐帧对齐)
            ─→ 静态TF base_link→相机光学系
apriltag_node ─→ /detections + TF ─→ docking_node ─→ /cmd_vel
```

**前置依赖**（狗上执行）：

```bash
sudo apt install python3-opencv        # OpenCV (含 FFmpeg/RTSP 支持)
```

**第 1 步 — 标定狗相机**（棋盘格，无 ROS 依赖，纯 ssh 即可）：

```bash
# 打印棋盘格 (如 10x7 方格 = 9x6 内角点), 用尺量方格实际边长
python3 scripts/calibrate_rtsp --url rtsp://192.168.1.100:8554/live \
    --size 9x6 --square 0.025
# 手持棋盘在相机前 0.2~1m 移动 (远近/平移/倾斜, 覆盖四角), 自动采帧解算
# → 生成 config/rtsp_camera_info.yaml (内参)
# Jetson 上若 --show 画面冻结/始终检测不到(FFmpeg 解码问题), 加 --backend gstreamer 硬解
```

**第 2 步 — 验证检测**（强烈建议先做，排除内参/TF 问题）：

```bash
# 终端 1: 只起 RTSP 桥
ros2 run tagdocking rtsp_camera --ros-args \
    -p rtsp_url:="rtsp://192.168.1.100:8554/live" \
    -p camera_info_file:=$PWD/config/rtsp_camera_info.yaml \
    -p mount.z:=0.35          # 相机离地高度, 按实际安装填

# 终端 2: 实时测量 (把 tag 放相机前, 看距离准不准)
python3 scripts/test_apriltag --image-topic /camera_sync/image_raw \
    --measure-frame base_link --known-distance 1.0
```

> 距离偏差大 → 内参或 `tag_size` 不对；TF 查不到 → 检查 `base_frame` 与
> 狗的实际基座坐标系名是否一致。

**第 3 步 — 启动停泊**：

```bash
ros2 launch tagdocking docking.launch.py \
    base_type:=omni \
    rtsp_url:=rtsp://192.168.1.100:8554/live \
    camera_info_file:=$PWD/config/rtsp_camera_info.yaml \
    odom_topic:=/odom \
    camera_mount_x:=0.25 camera_mount_z:=0.35   # 相机在 base_link 下的安装位姿
# 然后与差速车相同: ros2 service call /docking_node/start_docking std_srvs/srv/Trigger
```

关键 launch 参数：

| 参数 | 说明 |
|------|------|
| `rtsp_url` | RTSP 地址。非空即切换到机器狗模式（替代 camera_info_bridge） |
| `camera_info_file` | 内参 YAML（`calibrate_rtsp` 生成；不传时自动用包内 `config/rtsp_camera_info.yaml`，包内也没有才报错） |
| `odom_topic` | 狗的里程计话题（默认 `/odom_combined`，按实际改） |
| `camera_mount_x/y/z` | 相机在 base_link 下的安装位置（米） |
| `camera_mount_yaw/pitch/roll_deg` | 相机安装姿态（0=正前水平；低头用正 pitch，抬头用负 pitch） |
| `camera_downscale` | 输出降采样（0=自动到 ~640 宽，防大帧打爆 DDS） |
| `camera_backend` | RTSP 拉流后端 `ffmpeg`（默认）/ `gstreamer`（Jetson 硬解，FFmpeg 冻结时用） |
| `base_frame` | 狗的基座坐标系名（默认 `base_link`） |

**RTSP 延迟注意**：帧打的是到达时间戳，画面内容比时间戳旧 0.1~0.5s。
系统是走停式（机动后 settle 0.8s 等稳定再测量），天然容忍；若实测停稳后
位姿仍滞后（如转完头量出的横向偏差不对），把 `stopgo.turn_settle_sec`
（docking.yaml）调大到 1.0~1.5s。

> 若狗的相机/TF 已由其他节点提供，设 `rtsp_camera` 的
> `publish_static_tf:=false` 避免重复广播。

### 2.7 Odin1 相机部署 (use_odin)

后装 odin1 视觉模组的机器狗（RTSP 广角流压缩后 tag 太小难识别时）。odin 驱动
只发 `/odin1/image/undistorted`（去畸变 RGB 1600x1296，全分辨率下 16cm tag
@1m ≈ 117px，约为 RTSP 640 宽流的 4~5 倍像素），但**没有 camera_info 话题、
图像 frame_id 为空、也没有 `base_link→相机` TF**。`camera_info_bridge` 的
合成模式一次补齐三样，下游管线不变：

```
odin_driver ─→ /odin1/image/undistorted
camera_info_bridge ─→ /camera_sync/image_raw + /camera_sync/camera_info (内参合成+frame_id重打)
                   ─→ 静态TF base_link→相机光学系
apriltag_node ─→ /detections + TF ─→ docking_node ─→ /cmd_vel
```

内参无需标定：去畸变图像的针孔内参就是 odin 标定（`odin_ros_driver/config/
calib.yaml`）里的 A11/A22/u0/v0，已抄录为包内 `config/odin_camera_info.yaml`
（随包自动安装）。**换 odin 机身（不同序列号）后需从对应机身的 calib.yaml
重新抄录**。

**启动停泊**（按实际安装位姿传 `camera_mount_*`）：

```bash
ros2 launch tagdocking docking.launch.py \
    base_type:=omni \
    use_odin:=true \
    odom_topic:=/odom \
    camera_mount_z:=0.30   # odin 在 base_link 下的安装位姿, 按实际填
# 然后与 RTSP 模式相同: ros2 service call /docking_node/start_docking std_srvs/srv/Trigger
```

关键 launch 参数：

| 参数 | 说明 |
|------|------|
| `use_odin` | true 时用 `/odin1/image/undistorted` + 内参合成替代普通相机话题 |
| `camera_info_file` | 缺省自动用包内 `config/odin_camera_info.yaml` |
| `camera_mount_x/y/z`、`camera_mount_yaw/pitch/roll_deg` | 与 RTSP 模式同一套安装位姿参数 |
| `camera_downscale` | 默认 0=自动 ×2（800x648）；传 1 用全分辨率 1600x1296（仅追更远小 tag 时用，见下方延迟说明） |
| `image_topic` | 缺省即 `/odin1/image/undistorted`，可显式覆盖 |

**延迟实测（2026-09-08, Jetson）**：odin 源 ~55ms、桥 ~45ms 都很快，瓶颈在
apriltag —— 全分辨率 1600x1296 下它只消化 ~6fps，odin 22fps 输入把 RELIABLE
订阅队列塞满，检出时间戳年龄积到 **~1.15s**。默认 `downscale=2 + 桥
max_fps=10 限流`（桥在订阅回调里直接丢超额帧）后队列不再积压，延迟回落到
**~0.1s**。全分辨率小 tag 场景若嫌 5~6fps 检出率低，可 `camera_downscale:=1`
并在桥上加大限流（保持限流 ≤ apriltag 实际消化率即可不积压）。

> test_apriltag 读数提示：`>` 汇总行每 **2s** 打一次、且是**最近 50 次**检测
> 的均值 —— 移动 tag 后数值"追上来"要几秒是显示平滑，不是管线延迟；看单次
> 行 `[n] 距离=...`（每 10 次检测打一行）或加 `--sample-batch 10` 更跟手。

> 单独验证（不起底盘）：`ros2 run tagdocking camera_info_bridge --ros-args
> -p image_topic:=/odin1/image/undistorted
> -p camera_info_file:=config/odin_camera_info.yaml
> -p frame_id:=camera_color_optical_frame -p downscale:=2 -p max_fps:=10.0
> -p publish_static_tf:=true -p mount.z:=0.30`，tag 放视野内后
> `ros2 run tf2_ros tf2_echo base_link tag36h11:0` 应输出合理位姿。

---

## 3. 触发方式

系统提供 **三种** 触发方式：

### 方式一：Service (推荐，最简单)

```bash
# 启动停靠
ros2 service call /docking_node/start_docking std_srvs/srv/Trigger

# 取消停靠
ros2 service call /docking_node/cancel_docking std_srvs/srv/Trigger
```

**返回示例**:
```
success: True
message: "state=SEARCH_TAG"
```

### 方式二：ROS2 Action (适合业务系统集成)

```bash
# 发送 Action Goal
ros2 action send_goal /docking_node/dock tagdocking/action/Dock "{dock_id: 'charger_0'}"

# 取消
ros2 action send_goal /docking_node/dock tagdocking/action/Dock "{dock_id: 'charger_0'}" --cancel
```

**Feedback 实时输出**:
```
distance_error: 0.342   # 距离误差 (m)
yaw_error: 0.087        # 朝向误差 (rad)
state: "approach"       # 当前状态
```

> Action goal 中的 `dock_id` 目前仅作业务标识，实际停靠目标由 `tag.id` 参数决定。

### 方式三：从 Python 代码调用

```python
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

class MyDockingClient(Node):
    def __init__(self):
        super().__init__('my_docking_client')
        self._start_cli = self.create_client(Trigger, '/docking_node/start_docking')
        self._undock_cli = self.create_client(Trigger, '/docking_node/start_undock')

    def start_docking(self):
        req = Trigger.Request()
        future = self._start_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        print(future.result().message)

    def undock(self):
        req = Trigger.Request()
        future = self._undock_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        print(future.result().message)
```

### 充电收尾 (charge.*) — DOCKED 后 静止(锁定)→阻尼

停泊成功 (`DOCKED`) 时狗站在充电桩上方，`ChargeMode`
（`tagdocking/charge_mode.py`）在 DOCKED 态以 20Hz 非阻塞轮询推进两步收尾：

| 步骤 | 服务 | SDK 命令 | 完成确认 |
|------|------|----------|----------|
| 1. 静止锁定 | `~/static_stand` | CMD_LOCK_MODE | `posture_state==static_stand`（posture 开启时本就锁着，幂等直过） |
| 2. 阻尼/泄力 | `~/passive` | CMD_EMERGENCY_STOP (0x5A，电机泄力保电) | 服务成功响应后等待 `passive_settle_sec`；非硬件/充电确认 |

（服务前缀 `base.l1w_prefix`，默认 `/l1w_control`。）

- **与 posture.enable 完全无关**：呼吸抑制只管停-看循环里的量测稳定；
  即使中途的锁定/解锁全程关闭，DOCKED 后这两步照做。
- **阻尼步可单独关掉**：`charge.passive=false`（运行期可调，锁定完成那一刻
  读参）→ 收尾止于锁定，狗站立但不泄力。
- **跳过锁定直接阻尼**：`charge.static_stand=false` → DOCKED 后直接切阻尼，
  狗站立泄力，不做 static_stand。
- **失败只告警不翻车**：对接已成功，收尾是"锦上添花"——重试
  `charge.retries` 次耗尽、或无 l1w_control 桥（纯台架）超过
  `service_wait_sec` 宽限 → 告警一次，DOCKED 仍是成功终态；STATIC 步
  失败也会继续尝试阻尼（更接近目标）。
- **泊出衔接**：锁定/阻尼后 cmd_vel 被桥门控（`motion_enabled=False`），
  盲退发不出速度——`~/start_undock` 时先自动 stand_up 等恢复运动模式
  再盲退；stand_up 恢复失败则落 MOTION_FAILED（狗已泄力，不干等超时）。
  收尾进行中收到泊出请求 → 立即中断序列转 stand_up 恢复。
- 台架验证：`scripts/mock_l1w_control` 已镜像 static_stand/passive（posture →
  static_stand、门控关），日志可见 `STATIC → DAMPING`。

---

### 泊出 (Undock)

停泊完成后 (`DOCKED`)，调用 `~/start_undock` 把车退出停靠位：

```bash
ros2 service call /docking_node/start_undock std_srvs/srv/Trigger
```

泊出是一个固定的两段式盲动序列，纯里程计闭环、不看二维码：

1. **盲退** `undock.backup_distance` (默认 0.5 m，同重试倒车的 negative jog)
2. **原地转 180°** (`undock.turn_angle_deg`，正=CCW / 负=CW)

两段到位后进入 `UNDOCKED` 终态。`~/state` 依次发布 `undocking` → `undocked`，
webapp 据此弹"泊出完成"提示。泊出可从任意非停泊态触发 (IDLE / DOCKED / 错误终态)，
停泊进行中 (`SEARCH_TAG` / `APPROACH` …) 调用会被拒绝。

从 `DOCKED` 泊出时狗往往已锁定/阻尼、电机泄力（充电收尾，§3.2）——盲退前
自动先 stand_up 等 `motion_enabled==True` 恢复运动模式再动；恢复失败落
`MOTION_FAILED`。

> 泊出是盲动 (后退 + 转向)，**调用方需确保车后方无障碍**。

---

## 4. 状态机详解

### 4.1 正常流程状态

| 状态 | 枚举值 | 说明 | 超时 |
|------|--------|------|------|
| `IDLE` | 0 | 空闲，等待启动命令 | — |
| `SEARCH_TAG` | 2 | 角度步进扫描搜索 AprilTag | 60s (`search.timeout_sec`) |
| `ALIGN` | 3 | 遗留态，现行流程直接跳过（已并入 APPROACH 走停式） | — |
| `APPROACH` | 4 | 走停式两阶段逼近（法线机动对准 + 纯直行） | 60s (`approach_timeout_sec`) |
| `FINAL_SERVO` | 5 | 到位确认（距离±5cm + 方位≤15° 单帧 DOCKED） | 30s (`final_servo_timeout_sec`) |
| `DOCKED` | 6 | 停靠成功 | — |
| `RETRYING` | 8 | 失败自动重试：盲退一段距离后回 SEARCH_TAG 重锁 | 15s (`retry.timeout_sec`) |
| `UNDOCKING` | 9 | 泊出: 盲退 + 原地转 180° | 30s (`undock.timeout_sec`) |
| `UNDOCKED` | 14 | 泊出成功 | — |

全局超时 `timeout_sec`（120s）覆盖所有活动态；同一次停泊内的多次重试共享该
窗口（每次**用户触发** start 才重置）。

### 4.2 异常状态

| 状态 | 枚举值 | 触发条件 |
|------|--------|----------|
| `TAG_LOST` | 10 | 预留终态。现行实现中视觉状态丢 tag 超 1s 会**先回 SEARCH_TAG 重锁**而非直接报错；只有搜索本身超时才失败（落 `TIMEOUT`） |
| `TIMEOUT` | 11 | 全局超时 (120s) 或各阶段超时（搜索 60s / 接近 60s / 精停确认 30s） |
| `MOTION_FAILED` | 12 | 直行入口对准过差且重试耗尽（`retry.max_retries`=2）；或重试倒车超时、泊出超时（里程计不走时兜底） |
| `CANCELLED` | 13 | 用户主动取消 |

> 单帧丢检（低帧率/转向后模糊）不会终止停泊：连续丢失约 1s 才回 SEARCH_TAG
> 重新锁定，机动（盲动）期间不计丢失。

### 4.3 各状态行为详解

#### SEARCH_TAG — 角度步进扫描

```
原地转 step_angle_deg(30°, 里程计闭环盲转) → 停稳 settle → 检测停留
pause_time_sec(1.5s) → 未见到 → 再转 30° → …  (12 步转满 360°)
```

- **转动期冻结检测**：盲转中 `tag_visible` 恒为 False，不会误触发锁定
- **初始方向**：偏向最后见到二维码的一侧（`_last_seen_lat` 符号），从未见过
  则用 `search.search_direction`（+1=CCW / -1=CW）；之后始终同向
- **Tag 锁定**：检测期持续可见 `search.hold_time_sec`（0.5s）→ APPROACH
- `search.rotate_time_sec` 已弃用（角度步进化后不再读取，保留兼容旧配置）

#### APPROACH — 走停式两阶段逼近

**阶段 1（带角度修正，默认）**：`GeometryPlanner.plan_sequence` 从单次新鲜
测量算出完整**法线机动**路径 `[转①, 前进, 转②]`，一次走完（盲动，中途不看
相机）：
- 几何：站位点 A = tag 位置 + 目标距离沿 tag 法线；转①对准 A，直行到 A，
  转②正对 tag（-法线方向）
- 迭代精化：每轮最多走 `stopgo.jog_max`（0.15m）就停下重测；tag 法线
  （normal）远距时不可靠，靠近后逐渐收敛
- **已对准直行捷径（限近场）**：距 tag ≤ `final_straight.tighten_distance`
  且方位误差 ≤ 入口门槛、横偏 ≤ `final_straight.lateral_threshold_m` 时不再机动，
  直接直行（避免近场 normal 抖动引起的无谓摆头）
- **远/近两级修正**：阶段 1 的修正门槛与动作集按 `final_straight.tighten_distance`
  （1.3m）分两级（全向底盘逐帧规划 plan() 与差速底盘机动门槛共用此分界）：
  - **远场**（dist > tighten_distance）：**粗对准 + 纯前进**。方位/横向修正
    门槛放宽到 `far_yaw_threshold_deg`（15°）/ `far_lateral_m`（0.20m），且
    **不做法线对准、不横移**（全向底盘 plan() 传 normal=None 走纯方位
    aim-and-go：对准 bearing → 前进）。1.5m 处 AprilTag normal 噪声 ±10°
    被 dist 放大成 ±0.3m 横移噪声，是远场反复/反向横移的根源——远场只进
    不平移，先走近再精调
  - **近场**（dist ≤ tighten_distance）：**收紧到入口包络**（方位 3° / 横向
    `lateral_threshold_m`=2cm）才做"先对齐法线再横移"。否则"入口判不合格、
    规划器却不修"（方位 3~10°/横向 3~5cm 区间）会把误差原样带进直行区再
    被入口拒绝，陷入前进-重试乒乓。法线门槛不收紧（normal 近场抖动 4~5°，
    收紧会摆头）；跨区前进步长钳到恰好停在区界，入口检查拿到最新鲜量测

**阶段 2（纯直行，不修角）**：`final_straight` 启用时，距 tag ≤
`final_straight.start_distance`（0.85m）即**无条件直行**，像停车入库一样
不再调角——再往前已无空间调位姿，直行中横向误差保持到最终位姿。

- **直行失败检查（一次性，入口包络）**：首次进入直行距离时，方位误差 >
  `final_straight.yaw_threshold_deg`（默认 3°）**或** |横偏| >
  `final_straight.entry_lateral_m`（默认 5cm）说明对准过差、入库会撞偏 →
  报导航失败（进 RETRYING）。只判一次，防止直行中方位自然漂动误触发
- `final_straight.enable: false` 恢复单阶段（全程带角度修正）
- `start_distance` ≤ `dock_target.distance` 视为误配置，静默回退单阶段
- 兜底：走停迭代上限 40 次（正常收敛远在之前），单帧近场噪声一次超限
  往往是抖动，重试机制给重新靠近的机会

**全向轮附加（限近场）**：航向已对齐 tag 法线且横偏超横向门槛时直接
**横移**消横偏（差速轮无此自由度，靠阶段 1 的转向机动消除）。横移量 =
可靠量测 `lat`——车轴 ∥ 法线时垂直偏距就是 lat，不再用
`dist·sin(normal) − lat·cos(normal)` 的噪声放大式；航向未对齐时先原地转
齐法线（每步 ≤ `stopgo.max_turn_step`，停稳重测）再横移——横移只在车轴
∥ 法线时才等价于"平移上法线"，否则越移越偏（"先右转再左移"的次序）。
远场（> `tighten_distance`）不横移，纯方位对准 + 前进。

#### FINAL_SERVO — 到位即 DOCKED

APPROACH 中距目标 ≤ 2×`tolerance.position_m`（0.10m）即转入。stop-and-go
下规划器已把车停在目标距离（`plan_straight` 距离到位即 `[done]` 停车），
本状态只做**到位确认**——距离在容差内 + 方位 ≤ `final_servo.yaw_tol_deg`
即**单帧判定 DOCKED**，不再累积稳定、不追角度。

**成功判定（距离 + 方位，单帧即判定）**：
1. `|dist − dock_target.distance|` < `tolerance.position_m`（0.05m）
2. 方位误差 < `final_servo.yaw_tol_deg`（15°）

> 不查横向：横向是方位角 bearing=atan2(横向,距离) 的投影，15° 已隐含
> |横向| ≤ dist·tan15° ≈ 14cm。不累积 1s 稳定：呼吸/平衡摆动（posture 关闭时）
> 会让刻把已停好的泊位误判失败、连打十几秒日志后超时重试。

**安全保护**：
- 距离 < `safety.minimum_distance_m`（0.15m）时强制判定 DOCKED，防止碰撞
- 超时 30s 兜底：位姿合格则接受，超差则 fail() 走重试

#### RETRYING — 失败自动重试

`fail()` 触发（目前仅直行入口对准过差）：若重试次数 < `retry.max_retries`
（2），盲退 `retry.backup_distance`（0.5m，里程计闭环不看 tag）→ 回
SEARCH_TAG 重新锁定靠近；退满次数仍失败 → `MOTION_FAILED` 终态。
倒车超时 15s（里程计不走）直接落 MOTION_FAILED，防止无限倒车。

### 4.4 状态发布

```bash
# 监听状态变化
ros2 topic echo /docking_node/state
# 输出: data: "approach"
```

---

## 5. 底盘适配器

### 5.1 统一接口

```python
class BaseAdapter(ABC):
    def publish_jog(self, linear_rate: float): ...      # 前进/后退 (m/s, 负=后退)
    def publish_turn(self, angular_rate: float): ...    # 原地转 (rad/s, 正=CCW)
    def publish_arc(self, linear_rate, angular_rate): ...  # 直行 + 微小角速度 (双码纯直行区的行进中航向保持)
    def publish_stop(self): ...                          # 零速 Twist

# 仅 omni / quadruped:
    def publish_lateral(self, lateral_rate: float): ... # 纯横移 (m/s, 正=左)
```

接口是**常速离散指令**而非连续速度伺服：`ActionExecutor` 以恒定速率发布并
用里程计位移闭环掐断，规划器（而非底盘）负责几何。状态机/执行器与底盘解耦，
同一套停泊逻辑对三种底盘通用。

### 5.2 DiffDriveAdapter (差速轮)

- 输出 `Twist.linear.x`（前进）+ `Twist.angular.z`（原地转）到
  `base.cmd_vel_topic`（默认 `cmd_vel`）
- **无横移能力**：横向误差不靠 vy 消除，由规划器的法线机动
  （转向→前进→转向）或"瞄准即走"（先对准 tag 再直行）消除
- `publish_arc` **不是已弃用的接口**，但被弃用过的那**一种用法**仍然禁止。
  两件事方向相反，必须分清：
  - **禁止（历史弃用）**：*转向时叠加前进速度*。规划器要的是原地转 θ 度，
    叠上 vx 会让车沿弧驶出目标横向范围。`turning` 分支因此固定走
    `publish_turn`；角度精度靠里程计校准的全量盲转保证。若差速轮原地转需
    克服静摩擦，加大 `stopgo.jog_angular_rate`，不要叠加前向速度。
  - **允许（当前在用）**：*直行时叠加微小角速度*，即双码纯直行区的行进中
    航向保持（§6.5b）。那条是"本该只转、却混进了走"，这条是"本该只走、
    要守住不歪"。单周期 0.34°、整程不超过 `heading_hold_budget_deg`，符号
    规则保证只让 `|err|` 下降，不产生横向机动；未接通时走 `publish_jog`，
    与该功能上线前逐位相同。
  - ⚠️ `BaseAdapter.publish_arc` 的**默认实现**回落成纯原地转（会把前进
    速度整个丢掉）。四个具体 adapter 都已覆写；节点侧另有 `if angular:`
    分支保证未接通时根本不碰它。

### 5.3 OmniAdapter (全向轮 / Mecanum)

- 同差速的 jog/turn/stop，额外提供 `publish_lateral`（`Twist.linear.y`，正=左）
- 规划器在横偏超 `stopgo.lateral_threshold` 时直接下横移指令一步消偏，
  无需差速那种多段机动——全向底盘停泊更快更直接

### 5.4 QuadrupedAdapter (四足机器人)

- **不发布 cmd_vel**，所有指令转发给 SDK 回调 `move_callback(vx, vy, yaw_rate)`
- 支持横移（`vy`），规划逻辑与 omni 相同

```python
from tagdocking.base_adapter import QuadrupedAdapter

robot = YourQuadrupedSDK()
adapter = QuadrupedAdapter(
    move_callback=robot.move,   # SDK 的移动接口 (vx, vy, yaw_rate)
    node=ros_node,              # 可选, 仅用于日志
)
# 运行期也可换: adapter.set_move_callback(new_cb)
```

> **注意**：`docking_node` 的适配器工厂构造 `QuadrupedAdapter(node=self)` 时
> **未注入 move_callback**（回调为空操作），因此直接 `base_type:=quadruped`
> 起停泊命令不会到达任何地方——SDK 集成需在代码里完成上例注入。
> **cmd_vel 驱动的机器狗直接用 `base_type:=omni` 即可**（见 §2.6）。

---

## 6. 参数配置

全部参数在 `config/docking.yaml` 中；launch 启动时会用 launch 参数覆盖其中
一部分（`tag.*`、`base.*`、`odom_topic`、`dock_distance`→`dock_target.distance`、
`final_straight_distance/start_distance`、`final_straight_yaw_deg`→
`final_straight.yaw_threshold_deg` 等，launch 默认值见
`ros2 launch tagdocking docking.launch.py --show-args`；
`entry_lateral_m` 等空默认参数不传时以 yaml 为权威）。

### 6.1 Tag 与停靠目标

```yaml
tag.family: "36h11"                   # AprilTag 家族
tag.size: 0.16                        # Tag 边长 (m) — 不准则测距不准!
tag.id: 0                             # 要停靠的 Tag ID
tag.frame: "tag36h11:0"               # TF 中的 Tag 坐标系名
tag.fresh_timeout_sec: 2.0            # 超过 N 秒无检测 → 视为丢失 (放宽以适配 6fps)
tag.ema_alpha: 0.5                    # EMA 平滑 (0=重滤波, 1=原始值)
tag.max_pose_jump_m: 0.3              # 单帧位姿跳变阈值 (防误识别; 连续10帧跳变才接受新值)

dock_target.distance: 0.55            # 最终底盘距 Tag 多远停下 (m)
dock_target.lateral_offset: 0.0       # 横向偏移 (m), 0=居中
dock_target.yaw_offset_deg: 0.0       # 朝向偏移 (°), 0=正对 Tag
```

**场景示例**:

| 场景 | distance | lateral_offset | yaw_offset_deg | 说明 |
|------|----------|---------------|----------------|------|
| 正对接充电桩 | 0.55 | 0.0 | 0.0 | 停在 Tag 正前方 55cm |
| 横向靠边卸货 | 0.80 | 0.3 | 0.0 | 停在 Tag 前方 80cm 偏左 30cm |

### 6.2 走停参数 (stopgo.*) — 核心调参区

```yaml
stopgo.lateral_threshold: 0.04        # m — 横偏容许 ±4cm, 超过才修正(全向→横移/差速→机动)
stopgo.yaw_threshold_deg: 3.0         # deg — 角度容许 ±3°, 超过才转向
stopgo.jog_min: 0.05                  # m — 单步最小前进距离
stopgo.jog_max: 0.50                  # m — 单步最大前进距离 (**只 clamp 单码 GeometryPlanner**, 对双码无效)
stopgo.jog_linear_rate: 0.08          # m/s — 前进速度
stopgo.jog_angular_rate: 0.3          # rad/s — 转向速度
stopgo.min_angular_rate: 0.12         # rad/s — 下发角速度下限, 必须 > l1w_control 死区 0.10 (见下)
stopgo.lateral_rate: 0.12             # m/s — 横移速度 (omni/quadruped), 同样必须 > 死区 0.10
stopgo.jog_odom_scale: 1.0            # 前进通道里程计低估系数 (判停目标 = 距离/系数)
stopgo.jog_backward_odom_scale: 1.0   # 后退通道同上
stopgo.lateral_odom_scale: 1.0        # 横移通道同上
stopgo.turn_settle_sec: 1.5           # s — 转向后等图像清晰 (RTSP 相机可加大)
stopgo.turn_undershoot: 0.75          # 只转指令角的 75%, 防过冲 (legacy 步进转向用; 法线机动/搜索用全量盲转)
stopgo.turn_lead_per_speed: 0.30      # rad/(rad/s) — 盲转停止滞后补偿的提前量
stopgo.turn_slow_rad: 0.14            # rad — 距目标此角度内减速到半速
stopgo.max_turn_step: 0.3             # rad (~17°) — 单次转向硬上限
stopgo.small_turn_rad: 0.1            # rad — 小于此角度的转向减半速
stopgo.theta_shrink_ratio: 2.0        # 动态方位容差 = max(yaw_threshold, dist/ratio)
stopgo.drift_tol: 0.15                # rad — 前进中方位漂移上限 (走弧时提前停重规划)

# ── 行进中航向保持 (只在双码纯直行区那一步"一次走完"的长直行里武装) ──
stopgo.heading_hold_enable: true      # 现场一键回滚 (ros2 param set, 下一趟生效)
stopgo.heading_hold_rate: 0.12        # rad/s — 接通时下发的角速度幅值, 必须 > 死区 0.10
stopgo.heading_hold_engage_deg: 2.0   # deg — 接通门槛 (0.5×sin2° = 17.5mm < dock_tolerance 20mm)
stopgo.heading_hold_release_deg: 0.7  # deg — 断开门槛, **绝不取 0** (瞄 0 必过冲换符号 → 抖振)
stopgo.heading_hold_min_engage_sec: 0.15   # s — 最短接通 (3 周期 ≈ 一个步态相位)
stopgo.heading_hold_cooldown_sec: 0.30     # s — 断开后冷却, 覆盖命令→odom 报告滞后
stopgo.heading_hold_budget_deg: 15.0  # deg — 单程累计下发角上限 (诊断闸, 不是安全边界)
```

**大白话对照表**（把它想成：狗直着走，你在旁边盯着它的头，歪了就伸手掰一下）

| 参数 | 值 | 大白话 |
|---|---|---|
| `heading_hold_enable` | `true` | 总开关。要不要伸这只手 |
| `heading_hold_rate` | `0.12` | 掰的力道。0.12 是刚好越过底盘死区 0.10 的最小值——够用就行，大了一按就过头 |
| `heading_hold_engage_deg` | `2.0` | 歪到多少才管。歪 2° 以内不理会 |
| `heading_hold_release_deg` | `0.7` | 掰回到多少就松手。注意**不是掰到 0 才松**——瞄 0 必过冲，过冲就来回抖 |
| `heading_hold_min_engage_sec` | `0.15` | 一旦按下，至少按住 0.15 秒。按一下就松，腿还没反应过来 |
| `heading_hold_cooldown_sec` | `0.30` | 松手后歇 0.3 秒再做下一次判断（等 odom 把刚才那下报上来） |
| `heading_hold_budget_deg` | `15.0` | 一趟总共最多掰 15°。超了就报警 + 后半程纯直行 |

> **`budget_deg` 有个下限，配低了会被拒绝**：一次最短接通就要花掉
> `rate × min_engage_sec`，默认 `0.12 × 0.15 = 0.018 rad = 1.03°`。若
> `budget_deg ≤` 这个值，一次都接不通、`used` 恒为 0，完成日志打的
> "航向保持已用=0.00deg" 与"压根没漂够门槛"逐字相同——**最坏的失效形态是
> 看着开着、其实关着**。所以 `_heading_hold_params()` 直接 `warn` + 返回
> `None`（本趟退回纯直行），日志里会写清算出来的门槛和建议值。要真开，
> `budget_deg` 至少给到门槛的 3 倍以上。校验按**抬底后**的 rate 算
> （`max(abs(heading_hold_rate), min_angular_rate)`），否则把 `rate` 配成
> 0.02 就能绕过守卫、而实际下发的仍是 0.12。

> **为什么是"继电"(0 或 ±rate) 而不是比例律**：l1w_control 有
> `min_angular_z = 0.10` 死区，`|wz| < 0.10` 被 clampAxis 直接截成 0。比例式
> 命令在误差小的时候恰好落进死区，物理上发不出去——不是调参能救的。代码另有
> `max(rate, min_angular_rate)` 结构性抬底，配进死区也不会静默失效。
>
> **为什么参考量取 odom/IMU yaw 增量而不是墙码 bearing**：① 盲动期检测冻结，
> 行程中根本没有 bearing；② 近场 bearing 带相机横向力臂增益 `1+t_x/z`，最不
> 可信——那正是区内禁离散转向的理由；③ bearing 把横偏与航向混在一起，追它
> 等于 pure pursuit 横向控制器；④ 这个底盘只有 IMU yaw 可信。

**调参建议**（走停式，无 PID 增益）：
- **转向过冲**：`jog_angular_rate` 太快或里程计 yaw 漂移 → 降速率；用
  `scripts/test_turn_angle` 实测底盘转角精度
- **tag 在转向中丢失**：`max_turn_step` 太大或相机 FOV 窄 → 减小步长；
  `turn_settle_sec` 太短会采到模糊帧 → 加大
- **直行距离不准**：`jog_linear_rate` 太快 → 降速率；用
  `scripts/test_jog_distance` 实测
- **停泊位置系统性偏差**：几乎总是 `tag.size` 不对 / 相机内参不准 /
  相机安装 TF（mount.*）不准，用 `scripts/test_apriltag --known-distance` 验证

### 6.3 静止站立 (posture.*) — 走停 × 呼吸抑制

```yaml
base.l1w_prefix: "/l1w_control"       # 狗模式服务/状态话题前缀 (zsibot_l1_control)
posture.enable: false                 # 总开关 (默认关, launch posture_enable:=true 开启); 无桥时运行期自动降级停用
posture.static_settle_sec: 1.2        # s — 停→量测最短间隔 (RTSP 延迟+锁定过渡+呼吸衰减)
posture.lock_ack_timeout_sec: 2.0     # s — 等 posture_state==static_stand; 超时本次停降级
posture.unlock_ack_timeout_sec: 2.0   # s — 等 motion_enabled==True (含固件退出 LOCK 滞后)
posture.unlock_retries: 2             # 次 — stand_up 重发上限, 耗尽 → MOTION_FAILED
posture.min_stable_frames: 3          # 帧 — 规划前所需连续 accepted 新鲜帧 (1=关闭)
posture.stable_frame_timeout_sec: 2.5 # s — 等不满 N 帧就带当前位姿规划 (防闪烁卡死)
posture.service_wait_sec: 1.0         # s — 服务发现宽限, 超过判定无桥并停用
```

行为细节见 §1 "静止站立 × 呼吸抑制"。实机调参：若解锁偶尔超时（狗退出
LOCK 慢），加大 `unlock_ack_timeout_sec`；若每停耗时可接受且想更快，把
`min_stable_frames` 降到 2 或调小 `static_settle_sec`。

### 6.4 充电收尾 (charge.*)

```yaml
charge.enable: true                   # 充电收尾总开关 (与 posture.enable 无关)
charge.passive: true                  # 阻尼(泄力)步开关; false = 仅锁定 (运行期可调)
charge.static_stand: false            # DOCKED 后先 static_stand 锁定再阻尼; false=跳过锁定, 直接阻尼
charge.static_ack_timeout_sec: 3.0    # s — static_stand 等待确认
charge.lie_down_settle_sec: 4.0       # s — (已弃用) 趴下动作时长, 当前流程不再使用 lie_down
charge.passive_settle_sec: 2.0        # s — passive 受理后等待，非硬件确认
charge.retries: 1                     # 次 — 每步服务重试
charge.service_wait_sec: 1.0          # s — 无桥宽限, 超时降级告警 (DOCKED 仍成功)
```

行为细节见 §3.2 "充电收尾"。

### 6.5 两阶段停泊 (final_straight.*)

```yaml
final_straight.enable: true           # 两阶段开关; false 恢复单阶段
final_straight.start_distance: 0.85   # m — 直行阶段起点 (须 > dock_target.distance, 否则回退单阶段)
final_straight.yaw_threshold_deg: 3.0   # deg — 入口方位门槛: 进入直行距离时超此值报导航失败;
                                       #      近场方位修正门槛同此值 (launch 默认 3.0 会覆盖)
final_straight.entry_lateral_m: 0.05  # m — 入口横向门槛: 进入直行距离时 |横向| 超此值报导航失败 (一次性入口检查)
final_straight.lateral_threshold_m: 0.02  # m — 近场横移修正/捷径横向门槛 (比入口更紧, 须 < stopgo.lateral_threshold 保证规划器会滑)
final_straight.tighten_distance: 1.3  # m — 远/近分界: dist ≤ 此值近场精调(法线对准+横移),
                                       #      > 此值远场粗对准+纯前进不横移
                                       #      (建议 ≥ start_distance + jog_max; 误配小于 start 时钳到 start)
final_straight.far_yaw_threshold_deg: 15.0  # deg — 远场粗对准方位门槛 (dist > tighten_distance 时阶段1生效)
final_straight.far_lateral_m: 0.20    # m — 远场粗对准横向门槛 (dist > tighten_distance 时阶段1生效)
```

进入 `start_distance` 后无条件纯直行不调角；首次进入时方位或横向超入口
包络报导航失败（自动重试）。阶段 1 按 `tighten_distance` 分两级：远场
（> `tighten_distance`）**粗对准 + 纯前进**（方位/横向门槛放宽到 `far_*`，
全向底盘不做法线对准、不横移——远场 normal 噪声被 dist 放大，横移只会
反复）；近场（≤ `tighten_distance`）修正门槛收紧到 `lateral_threshold_m`
（比入口更紧），保证"入口判不合格的误差一定先被修掉"。见 §4.3 APPROACH。

### 6.5a 相机安装横向偏移补偿 (camera.lateral_offset_m)

```yaml
camera.lateral_offset_m: 0.0          # m — 相机光学中心相对底盘中心线的横向偏移 (+ = 相机偏左)
```

相机光学中心若装在底盘中心线左/右某偏移处，而 base→camera 静态 TF 的
`mount.y` 未含此偏移，量测 lat 会系统性偏小该值——节点认为"正对"时底盘
中心实际偏到 tag 法线另一侧，停泊整体偏位、近侧腿撞充电桩。此值**加回**
`raw_lat` 后，规划器在直行前自动左移修正到真实对准。

- **必须量测驱动、每停重测**，不可一次性盲移该值——SEARCH 抖动重入直行
  阶段时盲移会叠加成两倍；量测驱动则由里程计欠冲在下一停收敛。
- 补偿后近场 bearing 会增大 ~2°（3cm/0.85m），直行入口方位门槛的余量
  更真实。
- 该值运行期可调：`ros2 param set <节点> camera.lateral_offset_m 0.05`。
- **默认是 0**：当前相机（odin1 RTSP）光心与底盘中心线齐平。曾经配过的
  `0.03` 是为**上一个相机**量的，留在配置里等于凭空注入一条假横偏——
  `raw_lat` 被加大 3cm → 节点以为底盘偏左 → 持续往右修 → 停泊系统性偏右
  3cm。换装偏心相机时按实测填。
- **作用域只有单码路径**（`_lookup_tag_pose` 里 `raw_lat` 的唯一赋值点）；
  双码路径（§6.5b）完全不经过它。

### 6.5b 双二维码光学走停 (dual.*)

墙码 `36h11:0 / 0.15m`、桩码 `36h11:51 / 0.05m`。两码中心须处于同一
进桩中心线竖直平面；不是两码左右位置平均居中，而是**每个码各自水平方向居中**。
`dual.enable=false` 旁路此控制器；显式 launch `dual_enable:=false` 会覆盖 YAML true。

0. **全程站立（默认 profile）**：`dual.crouch_enable=false`（默认）下整个双码
   流程不趴下 —— 不调 lie_down，也没有 30cm 主动锁定/切站立（`request_stand`
   恒 False）。对准在距桩码 ~0.9m（≈ 距墙码 ~1.8m）的站位处一次完成，随后
   locked 直行到 0.50m 触发阻尼，直行阶段仍允许墙码 bearing 微调（见 4）。
   匍匐代码整段保留：`dual.crouch_enable=true` 恢复"30cm 主动锁定 → 趴下 →
   匍匐精调"的旧路径（含 2b 的趴下前粗对准），等底盘单轮转向问题修好再开。

**为什么站立（2026-09 现场结论）**：匍匐步态靠旋转单个轮/腿转弯，每次转向都
附带 4.5~7cm 的前移（见 3b），且小角度下误差与命令同量级，无法精确调整 ——
现场一次对准从墙码 1.683m "纠偏"爬到 1.247m，全程朝前净移 0.44m。因此暂时
放弃匍匐搜索/调整，全程站立。代价是相机高度：站立视角下 5cm 桩码的可见下限
`z_min = r + fy·(y+r)/(H−cy−required)`（r=0.05/√2；Odin 实标定 fy=731.99、
H−cy−required=660.76），即 **`z_min ≈ 0.0745 + 1.108·y`**（y = 相机光心高于
桩码中心）：y=0.30 → 0.41m、y=0.45 → 0.57m、y=0.60 → 0.74m；反解 z_min=0.9
得 y=0.745m —— 相机高出桩码 0.745m 以内，0.9m 处桩码清晰可见，而桩码要等到
墙码 z≈1.5~1.65m 才丢失。这决定了角度必须在 ~1.8m 站位一次对准完毕，也决定了
`dual.straight_start_distance` 必须从 1.0 抬到 1.70（丢失闭锁的资格门，见 4）。

**提交直行处的死区（2026-09-11 现场，已修）**：狗在墙码 z≈1.78m 完成对准
（墙/桩 bearing ≈ 1°、theta ≈ 1.5°、e ≈ 10mm、J ≈ 0.03），此时**直行即可成功**，
却报 `no visible translation candidate; independent stable-window budget
exhausted` 停在原地。桩码底边余量已被吃到 <10px（`visibility_margin_px` 8 +
`visibility_sample_pad_px` 2），于是每条前进候选都被严格 `visible()` 否掉。
而本该放行它的"桩码垂直离场"豁免有两把锁：

- `progress`（已**完成**一步合格直行）—— 首步之前它恒为 false；
- 墙距 ≤ `straight_start_distance`(1.70) —— 提交发生在观察窗内任意处（1.70~1.90）。

两把锁都只能靠"先前进一步"打开，而前进正是被否掉的那件事 —— 互为前提的死锁，
三个独立稳定窗口耗尽即 MOTION_FAILED。修法两条：

1. 放行的航向证据改为**两个来源任一**（`heading_committed`）：① 已完成的合格
   直行（`progress`+`qualified_ns`，最强）；② **两码持住对准**的直行承诺
   （`committed_ns`，即 aligned + `align_hold_sec`，正是 observe→approach
   那一跳所依据的同一份证据）。首步只有 ②。两者都由任何纠偏撤销
   （`action_started` / `observe`），**预测本身永远不产生证据**，纠偏候选
   （yaw/横移）也永远不享受放行。
2. 距离门改用**直行包络** `observation_distance + observation_tolerance`(1.90)，
   且丢失闭锁必须**同步抬**（见 4）—— 否则 1.70~1.90 的合法离场会掉进
   `pile missing outside qualified final entry` 硬失败，等于用一个新失效模式
   换掉旧的。仍保留硬距离上界：站位窗外（远场）的陈旧 stage 不得借此放行。

诊断补强：`margin_report()` 现在附带 `桩码垂直离场=放行 / 不放行(缺 …)`，逐条
列出四个前提里缺哪个。余量表只说桩码底边剩几 px，不说那几 px 该不该拦人 ——
现场日志里余量一路 61.6→49.3→39.0→25.8→+8px 然后三窗判死，缺的其实是放行。

**同一死锁的复发与治本（2026-09-12 现场，已修）**：把包络从 1.70 抬到 1.90
只是**把墙往外挪**，狗停在 **z=1.923m** 时它又撞上了一次 —— 离观察窗上沿只差
**23mm**，站位步 0.10m 被 `visible()` 否掉，而放行需要 `stage=approach` 与
`z ≤ 包络`，这两条又都只有走完这一步才拿得到（`stage` 在 observe 分支的
`d ≤ obs+tol` 之后才置位）。要进窗才能进窗。当趟其余条件全是好的：bearing
+0.58°/+0.96°、J 已收到 0.227。

真正的不对称在于：`_correction` 的纠偏候选早就是一张菜单（`cap / cap÷2 /
cap÷4 / 残差 / 下限`），**前进却只发一个方案**，不可见就直接计窗口，三窗判死。
修法是给纯前进步同一把刻度尺 —— `_emit_forward()` 的**收缩阶梯**：

| | |
|---|---|
| 档位 | `d / d÷2 / d÷4`，与 `_correction` 同一口径，现场只需记一套 |
| 下限 | `dual.dock_tolerance`(20mm) —— 比停泊容差还短的前进在终点判定里本就算"已到位"，发它换不回任何东西 |
| 窗口 | **整条阶梯算一次决策**，逐档都否才消耗一个窗口（不是每档一个） |
| 不适用 | 回退修剪与 `relaxed` 脱困后退：后退让两码退回画面中心、余量单调变好，它被否是另一种病（近平面/丢码），缩短退距不治；且脱困要的是 `improving` 那把宽松尺子 |
| 接线 | observe 站位步、acquire 匍匐逼近、`_advance` 的逐步走三处；纯直行区"一次走完"整段不可见时落回的也是这条 |

日志：收缩成功打 `dual 前进收缩 10.0cm → 2.5cm (原步不可见, 阶梯第 3 档)`；
逐档都否打 `dual 前进收缩阶梯全否: 试过 10.0cm/5.0cm/2.5cm (下限
dock_tolerance=2.0cm) —— 不是步长的问题`。

配套的另一处诊断缺口也补了：`no visible translation candidate` 那行原先只带
`margin_report()`，报的是**当前位姿**的余量，规划被否时它常常条条宽裕
（2026-09-12 现场最小的 pile B 还有 26px、required 才 10px），读起来就是
"每条边都够却一个候选都没有"，查无可查 —— 真正被否的是那一步**走过去之后**
的余量，而它在 `visible()` 里算出来就被丢了。现在 `_rejected_report()` 把它
接回日志尾部：被否的是哪一步、预测路径上最坏的四条边各剩几 px、破的是哪一条。

1. **找双码**：无墙码执行有界原地搜索（最多一圈且受搜索总超时）；只有墙码先停看，
   在持续新鲜墙码检测中确认缺小码后，每次退 0.05m。全轮后退最多 0.30m/6 次，
   获取双码默认限时 60s，获取后 observe 独立限时 120s；仍受总体任务超时限制。
   只有小码、无可信墙距或外参不许盲进。
2. **观察位**：双码有效、先小转/横移使其居中，再在相机墙码深度 1.8m ±0.1m 附近
   建立进桩阶段。1.8m 的由来：站立视角下 5cm 桩码在 ~0.9m 处仍清晰可见
   （"为什么站立"的 z_min 公式），而角度必须在桩码可见的最后窗口一次对准完毕
   —— 再近桩码出视野就没有参照了。太近仍受后退预算限制；预算不足报失败，
   需人工重新布置起点。
2b. **趴下前的单码粗对准 (`dual.prealign_*`)**：双码是**精调器不是收敛器** ——
   它枚举的单步上限只有几度，每步还要停稳-重测 ~2.4s。锁定那一刻残留多少方位误差，
   双码就得一步几度地啃回来（现场实测锁定时 20.4°，双码需 ~30 步 ≈ 70s，顶着
   `dual.observe_timeout_sec` 90s 走，必然失败）。所以在趴下之前、站立姿态下先用
   **墙码单码** bearing = `atan2(横向, 距离)` 做纯转向粗对准，收进
   `dual.prealign_tolerance_deg`(5°) 再移交双码。单步 ≤
   `min(dual.prealign_step_deg, stopgo.max_turn_step)`，步数预算
   `dual.prealign_max_steps`(12)。
   - 为什么在趴下前：墙码 0.15m 挂在墙上，站立视角看最清楚；桩码本来就看不见，
     粗对准也不需要它。
   - 为什么用单码：粗对准只收一个自由度（车头朝墙码），单码 bearing 是直接量测，
     不依赖两码基线、不会因桩码缺失而无解。精度不够正是移交双码的理由。
   - 粗对准步**不进**双码的动作预算/合格状态/视觉反馈账（不走 `action_started`），
     但**照装**里程计看门狗 —— 底盘不动（死区/锁定）必须当场报错，中止信息带
     `单码粗对准阶段 —` 前缀，与双码精调故障区分。
   - 预算耗尽**只告警后移交**，不判失败：收敛与失败判定的责任统一在双码
     (`observe_timeout` / `max_actions` / 看门狗)，两处都判会让同一故障有两种说法。
   - 只在 `acquire` 相位生效；进入 observe/approach/locked 后方向盘归双码。
3. **双码进桩**：每窗联合评估正负转向和正负横移，只发一个可见且显著改善的动作；
   横移硬上限 0.03m、前进 0.05m。转向上限按墙码光学 z **分远近两档可调**：
   `z > dual.straight_start_distance` 用 `dual.yaw_step_deg`(远场粗步，默认 8°)，
   否则用 `dual.yaw_fine_step_deg`(近场细步，默认 3°)，均限在 [0.5°, 15°] 且
   fine ≤ coarse（启动校验）。原先写死的 3° 硬上限已移除 —— 3° 在近场合适，在
   远场是灾难：底盘停止滞后约 2°，与命令同量级，`stopgo.turn_lead_per_speed` 的
   `0.5×目标` 钳位因此生效，每步只转目标的一半。放大上限不牺牲安全：`visible()`
   逐段采样整条转向轨迹，会把把码甩出视野的大步直接否掉，且半步/四分之一步候选
   一直在枚举里兜底。
   两码独立 bearing、航向 `min(tol,1°)`、横偏 2cm 均通过后才允许前进；
   航向门不再禁止横移。每 jog 后重新双码纠偏，不在 1.5m 或 1m 直接关闭纠偏。
3b. **站位守卫（observe 阶段）**：墙码 z 一旦掉到观察窗下沿
   (`dual.observation_distance - dual.observation_tolerance`，默认 1.7m) 以下，
   **无论是否已对准**，先后退把站位拉回再继续纠偏。
   - 补的缺口：原先距离只在「已对准 **且** held」之后才查，而整场几乎都在纠偏，
     等于**纠偏期间站位完全失管**。现场墙码 z 从 1.68m 一路爬到 1.25m 穿过整个
     观察窗，规划器一次都没察觉，直到矮桩码被压出画面下沿（底边余量
     61→49→39→26→+8px）、可见性包络被突破才暴露。
   - 为什么会爬：匍匐步态原地转向时机体整体前移，每个动作 4.5~7cm，**与动作
     类型和大小无关**（执行器侧 `_action_linear=0`，两码光学 z 同步减小＝纯平移）。
     那是底盘侧的扰动，守卫只负责把站位拉回来，不假装能消除它。
   - **只守近端**：太近会把矮桩码压出画面下沿，是唯一真实的失效模式；太远无害
     （两码都还在视野里，对准后 held 分支自会推进）。纠偏期间主动前进则相反 ——
     航向还没对，前进就是沿错误方向走远。
   - **只在 observe**：approach 是有意逼近 `dock_distance`，locked 是锁定直行，
     在那两个阶段挂 1.4m 门槛会把接近永久堵死。
   - 守卫排在 `aligned()` 之前，因此也接管了「已对准但站位过近」——该路径原属
     held 分支里的 `d < obs-tol` 后退，前置后那行永不可达，已删。
   - 后退记账**分两本**：站位守卫走独立的 `dual.standoff_reverse_limit`(1.0m)/
     `standoff_reverse_count`(20)，与找桩码/脱困的 `dual.reverse_limit`(0.8m)/
     `reverse_count`(16) 互不侵占 —— 把站位拉回观察窗是常态操作，与脱困共账
     会在真脱困时误报耗尽。耗尽分别报 `dual reverse standoff budget exhausted`
     与 `... acquisition ...`，现场可一眼分清是哪种退不出来。步长改 10cm 后
     **一步退 10cm 对一步爬 4.5~7cm 不再是打平**，守卫每轮净赚 3~5cm；但两本账
     都按**距离**守，步数上限随之退化为不起作用的天花板（站位 1.0m → 10 步，
     主账 0.8m → 8 步）。（上文"全轮后退最多 0.30m/6 次"与 config 的 0.8m/16
     的出入仍待定，此处不重写那句。）
   - 包络若已被突破，守卫的后退改走 `improving()` 宽松门：此时严格 `visible()`
     会否掉一切候选（fraction 0 就是那个已出界的当前位姿），守卫不换门就会
     亲手堵死唯一的出路。
4. **锁向与直行末段**：本轮已经双码对齐并前进，最近合格观测未过资格时限
   （`dual.qualification_sec`=12s，每次合格前进刷新），且墙距
   ≤ **直行包络** `observation_distance + observation_tolerance`(1.90m)，在
   持续新鲜墙码帧中确认小码缺失 ≥1.5s 才锁向。远处/未对齐丢小码停车等待后
   失败；不降级墙码纠角。锁向后小码再现也不转/横移，墙码丢失或流停就停。

   闭锁窗口曾用 `dual.straight_start_distance`(1.70 = 观察窗**下**沿)，与
   `visible()` 的桩码垂直离场放行同用一个数。两者一起抬到窗上沿是同一个
   修复的两半（2026-09-11 现场，见"提交直行处的死区"）：直行的提交发生在
   观察窗内任意处，若放行只认 ≤1.70 而闭锁也只认 ≤1.70，1.70~1.90 就成了
   死区；只抬放行不抬闭锁，则那段合法离场会掉进
   `pile missing / invalid outside qualified final entry` 硬失败。

   **直行阶段的墙码 bearing 微调（设计决定 1）**：locked 不再是纯前进 ——
   每停把墙码 bearing 归零就是对墙码做 **pure pursuit**：1.8m 处横偏 y₀ 的狗
   若每次都瞄着墙码走，走的是一条指向墙码的直线，到停泊面 (0.5m) 横偏为
   y₀·0.5/1.3 ≈ 0.38·y₀ —— 不止阻止偏航误差增长，还主动收敛直行阶段本来
   观测不到的横向误差（站立下唯一可观测量就是墙码 bearing，桩码早已丢失）。
   |bearing| ≤ `dual.straight_yaw_tol_deg`(1.5°) 不动；候选沿用"枚举-预测-
   过门"（两个符号都试，符号与步长交给 predict() 裁决 —— 相机横向力臂增益
   1+t_x/z 在 z=0.6 时达 1.33，手写 `min(|b|, step)` 会系统性过冲 15~40%）。
   无改善或不可见的候选**不失败、照直行**：把锦上添花的微调变成整场健康直行
   的中止是错的交易，墙码仍受保护 —— 不可见的转向根本不会发出。连续转向达
   `dual.straight_yaw_max_turns`(3) 强制前进，兜住底盘 ~2° 停止滞后导致的
   预测-现实偏差；两条路径都不判失败。转向发 `aligned=False`（转向不得产生
   行进资格）。已知副作用：归零的是**相机** bearing，偏离中线安装的相机会把
   该偏移带到接触点 —— 与 dual 其余部分一致（e 同为相机参照），不叠加
   `camera.lateral_offset_m`（已有 3cm 单码问题不在此修）。

   **纯直行区（`dual.steering_stop_distance`=1.0m）**：墙码光学 z ≤ 1.0m 之后
   **禁横移**；转向不再整体停摆，而是交给一条随接近收紧的门槛
   （`_bearing_tol`）裁决。approach 的 `_correction` 在此仍整体停摆，未对准也
   只前进、**不判失败**（近场不做完整纠偏是取舍而非异常；把一次本可成功的
   直行变成中止正是上面那个死区的病根）。
   - 为什么近场转向要收紧：相机装在 base 前方 t_x 处，bearing 增益 `1+t_x/z`
     在 z=0.6 时已 1.33（命令 2.86° 实变 3.30°），而底盘停止滞后 ~2° 与单步
     命令同量级 —— 越近，一步转向的不确定度越大、剩余行程越不足以把过冲
     收回来。现场表现就是近场摆头与来回横移。
   - **为什么不是一律禁**（2026-09-12 现场推翻）：那趟区内 `wall_bearing`
     +1.88 → +3.75 → +6.02 → +11.22° 全程无人纠，终点横偏 ~91mm，而
     `dock_tolerance` 只有 20mm。上面那条论证本身没错，错在拿一个 ~2° 量级的
     不确定度去否决一个 11° 量级的误差。
   - **门槛怎么来的**（`dual.straight_yaw_lag_deg`=2.0°）：
     ```
     收益 = y·(1 − dock_distance/z)        # 归零 bearing 即 pure pursuit 收缩横偏
     代价 = dock_distance·sin(lag)          # 本次转向自身的停止滞后残留成航向误差
     收益 > 代价  ⟺  tan|b| > dock_distance·sin(lag)/(z − dock_distance)
     ```
     右边就是门槛。它自己会做对两件事：z → `dock_distance` 时分母 → 0、门槛
     发散到 90°，**最后一截仍然禁转**（旧结论被保留，只是落在物理正确的位置
     而不是一条 1.0m 的硬悬崖）；z 远离时门槛降到 `straight_yaw_tol_deg` 以下
     由后者兜底，**区内门槛永远不比区外松**。
     典型值：z=1.00→2.0°，0.85→2.8°，0.75→4.1°，0.65→6.7°，0.55→11.1°。
   - ⚠️ **不要用 `steering_stop_distance` 来关区内转向**。它同时管着四件事：
     本条转向门槛、`_correction` 停摆、`visible()` 的 `allow_exit`、以及下面
     "一次连续直行"。调低它会把一次停到位一并关掉。要更保守只调
     `straight_yaw_lag_deg`（调到 ~8° 即近似恢复旧的整体停摆）。
   - 代价（明说）：区内 pure pursuit 的**横向**收缩不再被整体放弃，但仍受
     门槛限制 —— 门槛以下的小横偏原样带到接触点。
     **航向误差另有一层** —— 见下面"行进中连续航向保持"：区内行进中新产生的
     yaw 漂移被夹在 ±`heading_hold_engage_deg` 内。两层互补：这一层在**停稳
     时**纠视觉看得见的残余（航向 + 横偏混在 bearing 里），那一层在**行进中**
     守住视觉物理上看不见的漂移。现场那 91mm 里两者都有份。
   - `visible()` 的桩码垂直离场放行在此区内**不再要求"两码对准"**：航向已被
     策略冻结，未对准既不可被采纳为纠偏，拦下这一步前进也换不回任何东西 ——
     只会把一场合法的纯直行变成三窗耗尽（同一个失效模式的近场版本）。航向
     证据（`heading_committed`）与包络上界仍是硬前提，绝不按一个从未验证过
     的航向盲走。
   - **区内动作形态：一次连续直行到停泊处**。转向是停稳时的原地动作，**不**
     把前进切碎 —— 纠完一次，下一个窗口仍发全量剩余距离。区内 `_advance`
     不再按 `forward_step` 封顶（0.95→0.5 = 一段 ~0.45m 连续前进，对照此前
     5 步 × 2.9s）。终点精度交给停稳重测的量测闭环：容差内即 done；不足由
     收缩步补齐；**冲过则回退修剪**（负 jog，单步封顶 `reverse_step`，显式
     日志"冲过停泊点 Xcm，回退修剪"）——里程计尺度误差由此兜底，无需预先
     标定 `jog_odom_scale`。配套两处：`ActionWatch` 对连续直行按实际行程放宽
     deadline（0.45m @ 0.08m/s ≈ 5.6s，原 6s 动作超时会掐死它）；冲过硬失败
     对"合格终局"（progress + 资格新鲜 + 直行包络内）豁免，否则 approach
     阶段的冲过会在丢失闭锁之前被判死。
     `visible()` 对**终局整段**（`continuous`）的桩码**整体免检**，不只是
     下沿：狗是骑跨在桩上充电的（机体下方电极片对准桩上电极片），到停泊处
     桩码必然位于机体下方、必然出画——要求它在路径终点仍可见，等于要求一个
     "成功时必然不成立"的条件。桩码贴近时保守包围立方体的投影还会横向炸开
     （z→半径 时发散），连 `allow_exit` 保留的左右边也会把整段否掉，那不是
     "这步走错了"而是"到位了"。这一段结束即停泊，下一窗口只做容差判定或
     修剪，不再需要桩码，所以放弃的也不是任何后续要用的量测。现场主线上
     `observe` 在 locked 里直接把桩码置 `None`（重现不得改写已锁命令），
     本就没有桩码采样点。
     整段仍不可见则**退回 `forward_step` 逐步走**：此时被否的只可能是墙码，
     而墙码是修剪与终点判定唯一的依据，它必须在整段路径上都留在画里，否则
     宁可逐步走、每停重测——那是长期现场验证的老行为。
   - **行进中连续航向保持**：区内一次走完与行进中守住航向是同一个决定的两半
     —— 既然不停下来纠方向，就得在走的过程中不让它歪掉。现场实测机器狗在这
     一段**机身朝左歪（有真实航向角）**，残余航向 ε 的终点横向代价 =
     `dock_distance·sin ε`：5-10° 漂移 → 35-70mm，而 `dual.dock_tolerance`
     只有 20mm。
     做法：以 odom/IMU yaw 相对起步的增量为误差，跑一个**带迟滞的继电**
     （输出只有 0 或 ±`heading_hold_rate`，因为 l1w_control 死区 0.10 让比例
     律根本发不出去，见 §6.2）。接通要五条全满足（超 engage 门槛 / 读数
     合理 / 冷却已过 / 预算够 / 尾段留得下一次最短接通+沉降），断开点是
     `release` 而非 0，接通期内符号锁定。单周期 0.34°，整程通常 3-6 段、
     每段 0.15-0.35s。**只有区内那一步 `continuous=True` 的长直行会武装它**
     ——回退修剪、区外逐步走、泊出/重试盲腿、单码通道一律拿不到。
   - ⚠️ **这与上面的 bearing 门槛是互补的两层，不是重复**。两者参考量、闭环性、
     纠的对象都不同，看到区内既发离散 `yaw` 又发连续 `angular.z` 不要以为其中
     一条是冗余的：

     | 停稳时的 bearing 微调 | 行进中连续保持 |
     |---|---|
     | 参考量是**墙码 bearing**，带相机横向力臂增益 `1+t_x/z`（z=0.6 时 1.33） | 参考量是 **odom/IMU yaw 增量**，旋转不产生力臂误差，增益恒为 1 |
     | **单步开环**：发一次 3° 命令，下一个停看点才知道结果 | **闭环**：20Hz 每周期重算，单周期命令量 0.34°，过冲下一周期就被看到 |
     | 不确定度来自 **~2° 停止滞后**（= `straight_yaw_lag_deg`，正是门槛的代价项） | 单次命令 0.34° ≪ 滞后；滞后被迟滞带 + 冷却结构性吸收 |
     | 纠的是**上一停视觉已看见**的残余误差（bearing 把航向与横偏混在一起） | 纠的是**行程中新产生**的 yaw 漂移 —— 上一停的视觉物理上看不见它 |

     一句话：一层在**停稳时**用视觉纠"已经歪了多少"，一层在**行进中**用 IMU
     守住"别再歪下去"。现场那 91mm 里两者都有份，少任何一层都补不齐。
   - 失效时的退化终点**全部是 `wz≡0` = 不做这件事的老行为**：odom 过期/非
     有限由既有 `ActionWatch` 掐掉整个动作（排在 `update()` 之前）；`|err|`
     > 20° 判读数故障；预算耗尽告警后本程纯直行。没有任何失效模式比不做这
     件事更差。
   - 取值须 `dock_distance < 本值 <= straight_start_distance`（启动校验）：
     落在终点之后等于从不生效，越过站位则把 observe 的双码对准一起禁掉。
     设成略大于 `dock_distance` 即等效关闭。
5. **到位**：合法进桩后的新鲜停稳墙距 0.50m ±0.02m 才先零速、cancel，再 DOCKED。
   最后 jog 可小于通用 jog_min；容差外的少量不足由收缩步补齐，少量冲过
   （> 0.02m）由回退修剪（单步 ≤ `reverse_step`，走主账）收回——这是纯直行区
   连续直行的配套语义；无资格的异常越界仍失败。随后一次性
   `ChargeMode.begin()` 请求 passive；请求被接受、实际阻尼、充电电流建立是三回事。

**参考系与安装约束**：使用标定主点对应的光学 x/z 水平视线；距离是 optical z，
不是欧氏距离、base x，也不叠加 `camera.lateral_offset_m`（已有 3cm 单码问题不改）。
完整 `base_frame -> camera_frame` 外参将不同高度的码中心变换到机身地平面求进桩线。
光轴水平投影须近似平行机身前进、roll 近零（默认 2°安装门），pitch 由完整旋转处理。
最后一步用机身前向在光轴方向的投影换算；无可信外参、错误 ID 或不支持横移底盘会拒绝，
**不再用 0 偏移/单码降级偷偷继续**。外参配置必须与真实安装一致，软件无法验证标定真实性。

**候选与反馈**：完整 SE(2) 预测含相机绕机身转动的位移、pitch 与码高差。
候选包括上限、半步、四分之一步、限幅残差和最小可执行量；3mm 横移仅是离线起点，
不是已验证的硬件能力。评分为
`J=0.5Σ(bearing/tol)²+(max|bearing|/tol)²+0.25(theta/3°)²+0.25(e/0.03m)²`，
近同分选小步，固定次序决胜；不是全局路径规划或收敛证明。连续三个独立稳定窗无安全
改善候选就失败，不把 timer 重试当新窗。动作成功启动才扣预算，完成合格前进才提交末段
资格；明显反向/过期 odom/无响应/超时 stop-cancel-abort，停稳后核验实际 J 改善，连续
三次无可分辨净改善或振荡失败。不会自动倒转命令符号。

**真实相机视野**：`dual.camera_info_topic=/camera_sync/camera_info` 必须对应检测图像的
原始 stamp、frame、输出分辨率与内参；缺失有限等待，非法模型明确失败。`dual.projection_mode`
显式选择 `raw`（K + plumb_bob，D 为零个或五个系数）或 `rectified`（单目 P、单位 R）。
已知 Odin `/odin1/image/undistorted` launch 默认选择 rectified，其余保留 YAML raw；
自定义图像源须显式传 `dual_projection_mode`。把订阅 remap 为 image_rect **不会去畸变**。
ROI/binning 非平凡值不支持，须供应已按输出图像归一化、缩放的 K/P；不能拿原分辨率标定
配缩小图。标定光轴是 x/z=0，对应主点 cx，不一定是几何中线 width/2；本轮不补偿该差异。

用码尺寸包围体和区间畸变检查四边余量，默认 12 段中间采样、8px 边距另加 2px 采样余量。
采样仅为模型预测，不保证连续轨迹绝不丢码，不是避障、遮挡或制动净空验证。前后小步也检查；
只有近距、已完成合格前进且当前仍对准的纯前进可允许小码垂直退出，墙码始终受保护。
预测退出不锁向，仍须真实停稳持续缺码；转向/横移绝不借此甩掉小码。

**当前位姿已出包络时的后退脱困**：包络被突破后，每条转/横移候选的 fraction 0
就是那个已出界的当前位姿，于是候选恒不可行，唯一出路是后退（后退单调增大两码
光学 z，把两码带回主点附近）。这条后退经 `improving()` **宽松门**放行——只要求
不恶化任何一条边，而非严格 `visible()`。
**放行门与复检门必须是同一把尺子**：`pending_valid()` 曾一律用严格 `visible()`
复检，于是宽松放行的后退必然被自己否掉，形成**活锁而非报错**——规划发出后退 →
复检否掉 → `stopped()` 重置滤波并把 settle 再推 1.5s → 重新采帧 → 规划同一条
后退……动作计数不增，现场只看到「后退脱困」告警每 2s 刷一屏而底盘一步不动，直到
外层 `approach_timeout_sec` 兜底。现改为在 `_emit` 记录该计划走的是哪道门
(`pending_relaxed`)，复检沿用同一道。宽松门不会渗漏成默认：正常候选仍走严格门。

**日志**：双码发现/有效固定 key 默认每 2s 输出，首条及内部阶段变化立即输出，附 suppressed
计数，不减少检测处理。动作启动记录双码 xyz、独立 bearing、theta/e、预测 J 和图像余量；
完成记录 signed odom，停稳后记录实际 J。内部 observe 不等于外层 APPROACH 已对准。

**时间与帧门**：每个动作停车至少 `max(1.5, dual.settle_sec, posture.static_settle_sec)` 秒，
不依赖 posture.enable。清空旧观测后要求至少 `max(3, dual.min_frames)` 个不同时间戳新帧。
双码使用同一检测 stamp 的非阻塞 TF 查询；TF stamp 差限 `dual.tf_skew_sec=0.02`，
新鲜限 `dual.fresh_sec=0.6` 秒。重复、过期、运动期、停稳前或缺码帧不推进双码稳定；
没有“等不够帧超时照走”。解锁期间待发计划过期会丢弃并停车重测。
流整体中断不等于小码正常退出视野。真实相机/TF 必须同钟且发布采集时间。

参数在 `config/docking.yaml` 的 dual 区集中维护；launch 可传 `dual_observation_distance:=1.8`、
`dual_forward_step:=0.10`、`dual_settle_sec:=1.5`、`dual_dock_distance:=0.50` 等同名下划线参数
（默认空值保留 YAML）；保留已验证的 stopgo 速度和里程计比例。
`dual.forward_step`/`dual.reverse_step`/`dual.lateral_step` 此前都是**装饰品**：
前进/后退五处发出点写死 `min(.05, ...)`（`reverse_step` 早已配成 0.10 却从未
生效），横移候选枚举写死 `min(.03, ...)`（现场把 YAML 调到 0.05 跑出来仍是
0.03），与 `yaw_cap` 那次是同一个病。硬上限已全部拆除，现在真正生效：
前进/后退默认 **0.10**，1.8→0.5m 的 1.3m 由 26 步减半到 13 步（每步起停+停稳+
重测 ~2.9s，开销远大于位移本身）；横移默认 **0.05**，校验区间 `[0.005, 0.10]` ——
横移是盲走，执行器侧没有 `jog_max` 那样的硬闸，上限比前进保守。精度不变 ——
终点步仍按剩余距离收缩，`dock_tolerance` 仍是 0.02；`stopgo.jog_max` **只 clamp
单码 GeometryPlanner，对双码没有任何 clamp**——双码的真实上限就是
`dual.forward_step` 本身，途中安全由 `visible()` 逐段采样保护。`dual.straight_start_distance` **兼容旧名字但改变含义**：只剩 yaw_cap 远/近
分档与参数排序校验两处用法；桩码垂直离场放行与丢失闭锁改用直行包络
`observation_distance + observation_tolerance`(1.90)。
`dual.pile_fresh_timeout_sec` 已弃用，统一改用 `dual.fresh_sec`。
其它重点：`missing_confirm_sec=1.5`、`missing_timeout_sec=8`、`qualification_sec=12`、
`max_actions=160`、`dock_tolerance=0.02`、`straight_yaw_tol_deg=1.5`/`straight_yaw_max_turns=3`/`straight_yaw_lag_deg=2.0`、
`forward_step=0.10`/`reverse_step=0.10`/`lateral_step=0.05`、`steering_stop_distance=1.0`、
`reverse_limit=0.8`/`reverse_count=16`、`standoff_reverse_limit=1.0`/`standoff_reverse_count=20`、
`acquire_timeout_sec=60`/`observe_timeout_sec=120`。这些阈值尤其小码退出距离必须实测，不能仅从
两码前后 0.9m 间距推断。检测应使用全分辨率（`camera_downscale:=1`）与正确逐码尺寸。

**无机器人自动化测试**（不启动 ROS 节点、服务或 launch）：

```bash
cd /home/nvidia/whale-nav/src/tagdocking
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q test
python3 -m compileall -q tagdocking launch
```

BUILD_TESTING 通过 ament_cmake_pytest 注册此测试目录。禁用 pytest 插件自动加载用于避开
系统 pytest 6.2 与用户 anyio 插件不兼容；不是跳过本项目测试。旧静态 TF/mock 教程只用于
单码；双码测试输入必须有真实递增检测时间戳及匹配动态 TF，不能以 latest 静态假码验收。

**已验证的离线包络**：独立固定世界传感器（不把规划器预测当实际反馈）测试了
左右 ±0.12m、航向 ±0.12rad 的九种组合，1600×1296、fx=fy=700、水平相机、
0.9m 前后码间距，计入动作/姿态等待/停稳/采帧后 90s 内开始合格前进且少于 30 动作。
另有独立 0.6m 码间距、fx=400 的退出兼容布局，在 180s 内真实垂直失去小码、锁向并
到达墙深度容差；这不是实际 0.9m 布局一定能完成末段的证明。pitch ±0.15/0.2rad
由几何预测对照测试覆盖，不代表该 pitch 下全程闭环已验证。实际视场、标签大小、退出位置
或执行死区不兼容时预期有限停车失败，不盲恢复。噪声/振荡/过冲反馈、反向/无响应 odom、
虚假 odom 完成均有失败路径测试；它们不替代真实动力学验证。

**实机验收需要另行授权**：先仅观察核验 ID/尺寸、完整外参、两个独立中心误差、光学距离；
后方及桩内净空可控、有人持急停时分别验单步后退/横移/转向及 odom 比例；最后完整走停，
记录正常小码退出深度、墙码新鲜度、0.50m 停车误差与独立阻尼/电流反馈。
`docking.launch.py` 有清杀旧相机/检测进程的历史副作用，**不要为了测试参数而执行 launch_setup**。
本自动化测试不证明实际制动距离、侧移死区、地面打滑或充电触点可靠性。

### 6.6 搜索参数 (search.*)

```yaml
search.angular_speed: 0.3             # rad/s — 每步旋转速度
search.step_angle_deg: 30.0           # deg — 每步旋转角度 (≥10°, 太小会半速拖慢)
search.pause_time_sec: 1.5            # s — 每步之间的检测停留时长
search.hold_time_sec: 0.5             # s — tag 持续可见此时长才锁定
search.search_direction: 1            # +1=CCW, -1=CW (从未见过 tag 时的起始方向)
search.timeout_sec: 60.0              # 整体搜索超时
# search.rotate_time_sec 已弃用 (角度步进化后不再读取)
```

### 6.7 容差 / 安全 / 超时

```yaml
tolerance.position_m: 0.05            # m — 前后/横向 ±5cm 视为到位
tolerance.yaw_deg: 10.0               # deg — 方位 ±10° 视为对准
tolerance.stable_time_sec: 1.0        # s — 已弃用 (FINAL_SERVO 改为到位即 DOCKED, 不再累积稳定)
# final_servo 到位确认: stop-and-go 把车停在目标距离后只判距离+方位,
# 单帧就 DOCKED (见 §4.3 FINAL_SERVO)。
final_servo.distance: 0.20            # m — 距目标此值内转入 FINAL_SERVO
final_servo.yaw_tol_deg: 15.0         # deg — 到位即 DOCKED 的方位门槛 (直行到位后不再追角)

safety.minimum_distance_m: 0.15       # m — 距 tag 更近直接判 DOCKED, 防碰撞
timeout_sec: 180.0                    # 全局停泊超时 (静止站立每停 +1.5-2.5s, 120→180)
approach_timeout_sec: 90.0            # APPROACH 阶段超时 (60→90)
final_servo_timeout_sec: 30.0         # FINAL_SERVO 阶段超时
# final_servo.max_linear_speed / max_yaw_speed: 遗留, 走停式下未使用
```

### 6.8 重试与泊出 (retry.* / undock.*)

```yaml
retry.max_retries: 2                  # 失败自动倒车重试次数 (仅直行对准失败会触发)
retry.backup_distance: 0.5            # m — 重试盲退距离
retry.linear_rate: 0.08               # m/s — 重试倒车速度
retry.timeout_sec: 15.0               # 倒车超时(里程计不走兜底落 MOTION_FAILED)

undock.backup_distance: 0.5           # m — 泊出盲退距离
undock.linear_rate: 0.08              # m/s — 泊出倒车速度
undock.turn_angle_deg: 180.0          # deg — 泊出转向 (正=CCW, 负=CW)
undock.angular_rate: 0.3              # rad/s — 泊出转向速度
undock.timeout_sec: 30.0              # 泊出超时
```

### 6.9 相机与话题

```yaml
camera.max_latency_ms: 150            # ms — 位姿时效窗口下限
camera.latency_interval_margin: 3.0   # 窗口 = max(下限, 实测检测间隔 × 此值) — 自适应帧率
pose_buffer.size: 30                  # 缓冲位姿数
detection_topic: "/detections"        # apriltag_ros 检测话题
odom_topic: "/odom_combined"          # 里程计话题 (launch 参数可覆盖)
```

---

## 7. 调试方法

### 7.1 监控状态

```bash
# 实时状态 (1 Hz 日志)
ros2 run tagdocking docking_node

# 状态话题
ros2 topic echo /docking_node/state

# 误差话题
ros2 topic echo /docking_node/error
# x=距离误差, y=横向误差, z=yaw误差
```

### 7.2 绘图观察误差收敛

```bash
# 安装 rqt_plot (若未安装)
sudo apt install ros-humble-rqt-plot

# 绘制距离误差和 yaw 误差
ros2 run rqt_plot rqt_plot \
    /docking_node/error/x \
    /docking_node/error/z
```

### 7.3 查看 TF 树

```bash
# 检查 TF 连通性
ros2 run tf2_tools view_frames.py
# 查看 frames.pdf

# 实时查看 tag→base_link 的变换
ros2 run tf2_ros tf2_echo base_link tag36h11:0
```

### 7.4 检查 Tag 检测

```bash
# 确认 apriltag_ros 在发布检测结果
ros2 topic echo /detections --once

# 确认 TF 广播 (应该看到多帧 tag36h11:X)
ros2 topic echo /tf | grep tag36h11
```

### 7.5 RViz 可视化

1. 添加 **TF** 显示：确认 `base_link → 相机光学系 → tag36h11:0` 连线
2. 添加 **Image** 显示：查看相机画面，确认 Tag 在视野内
3. 添加 **Odometry** 显示：确认里程计箭头随机器人运动

### 7.6 调试脚本（scripts/）

```bash
# 转角精度: 指令转 90°, 对比里程计实测角度 (走停式的转向全靠里程计闭环)
python3 scripts/test_turn_angle --angle -90            # 顺时针 90°
python3 scripts/test_turn_angle --angle 180 --speed 0.2

# 直行精度: 指令前进 0.5m, 对比里程计实测位移
python3 scripts/test_jog_distance --distance 0.5
python3 scripts/test_jog_distance --distance 0.3 --speed 0.05

# 相机直测: 起 apriltag_node 实时打印 tag 距离/横向/方位 (验证内参+TF+tag_size)
# 已知距离下放 tag, 实测值应一致 (±2cm):
python3 scripts/test_apriltag --known-distance 1.0
# 机器狗 RTSP 模式 (先单独起 rtsp_camera):
python3 scripts/test_apriltag --image-topic /camera_sync/image_raw \
    --measure-frame base_link --known-distance 1.0

# 相机标定:
python3 scripts/calibrate_camera   # ROS 相机话题模式 (需 ssh -X, GUI)
python3 scripts/calibrate_rtsp --url rtsp://...   # RTSP 模式 (纯 ssh 无 GUI)
```

### 7.7 常见问题排查

| 问题 | 原因 | 解决 |
|------|------|------|
| 停在 SEARCH_TAG 一直转圈 | Tag 未检测到 | `ros2 topic hz /camera_sync/image_raw`；检查 `dock_tag_id`、光照、`tag.size` |
| 检测频率极低 / `Synchronized pairs: 0` | image 与 camera_info 时间戳不配对 | 由 camera_info_bridge 解决；确认 apriltag 订阅的是 `/camera_sync/image_raw` 而非裸话题 |
| `TF lookup failed` / TF 查不到 | 坐标系链路断 | 检查 `base_link → 相机光学系 → tag` 链路；ROS 相机模式依赖机器人 URDF，RTSP 模式由 rtsp_camera 发静态 TF（`mount.*` 参数） |
| 有检测但 TF 查询全失败（TF 树无 tag 帧） | apriltag_ros 3.4.0+（上游 ROS2 重写，节点名 `/apriltag`）参数为嵌套 `tag.ids`/`tag.frames`/`tag.sizes` 且无 `publish_tf`（TF 无条件发布），旧 fork 为扁平 `tag_ids`/`tag_frames`/`publish_tf`；传错的一套被静默忽略 → tag 不在配置里 → 不解算位姿、不广播 TF | launch/脚本已两套同传兼容两代版本；`ros2 param list /apriltag` 核对参数是否生效，`ros2 run tf2_ros tf2_echo camera_color_optical_frame tag36h11:0` 验证 TF |
| 转向后丢 tag 回 SEARCH_TAG | 转太快出 FOV / settle 太短 | 减小 `stopgo.max_turn_step` 或 `jog_angular_rate`；加大 `stopgo.turn_settle_sec` |
| 直行失败报导航失败（自动重试） | 进入直行距离时方位 > `final_straight.yaw_threshold_deg` 或 \|横向\| > `final_straight.entry_lateral_m` | 看"直行失败"日志里的方位/横向误差；近场修正门槛已与入口包络同步（`tighten_distance`），仍不足可收紧 `entry_lateral_m`/`yaw_threshold_deg`，或放宽入口门槛 |
| 重试耗尽落 MOTION_FAILED | 多次直行失败 / 倒车时里程计不走 | 看日志定位具体原因；检查底盘是否响应 `/cmd_vel`、里程计话题是否正确 |
| 停泊位置系统性偏前/偏后 | `tag.size` 不对 / 相机内参不准 | `test_apriltag --known-distance` 验证实测距离，重标定 |
| 停泊位置横向偏 | 相机安装 TF（`mount.*`/URDF）不准 / 相机光学中心偏离底盘中心线 | RTSP 模式校准 `camera_mount_y`；若量测 lat 系统性偏小且近侧腿撞桩，设 `camera.lateral_offset_m` 补偿（+ = 相机偏左） |
| 双码停泊终点系统性偏左/右 | 纯直行区行进中 yaw 漂移，或航向保持没生效 | 读 `dual action COMPLETE` 日志的 `Δyaw`：持续 > 2° 说明保持没起作用——查 `stopgo.heading_hold_enable`，再 `ros2 topic echo /cmd_vel --field angular.z` 看 wz 是否被 clampAxis 截零（截零就调大 `heading_hold_rate` 到 0.15~0.18）。`航向保持已用` 打印 0° = 从未接通；打印"预算耗尽"= 底盘不响应 wz 或 odom yaw 异常。若 `Δyaw≈0` 却仍偏，那是**横向平移**不是航向漂移，本参数组管不了（看上一行） |
| 区内 `wall_bearing` 一路变大却没人纠 | bearing 没超 `_bearing_tol` 的收紧门槛，或转向候选全被 `visible()` 否掉 | 看 `纯直行区 … bearing=X ≤ 收紧门槛 Y` 这行：X<Y 是设计行为（纠它不划算）；要更早介入就调小 `dual.straight_yaw_lag_deg`。若打的是 `无可行转向候选` 则是墙码要被转出画面——那是余量问题，抬 `observation_distance` 或相机下俯，不要动门槛。**注意 `Δyaw` 现在每一步都有真实读数**（含横移步与走停退化路径；转向步额外附指令角以便读欠转），不再是"只有 `continuous=True` 那一步才测" |
| 区内仍在 10cm 走停、拿不到一次停到位 | 整段连续直行被 `visible()` 否掉，静默退化成 `forward_step` 逐步走 | 看该窗口日志里的 `min_margin`：整段长跑会把墙码推到画面边缘，余量见底就会被否。抬 `observation_distance` 或相机下俯增加余量；`dual.visibility_margin_px` 只买到 ~2mm，是最后手段。退化路径本身是安全的（每停重测 + bearing 微调仍在工作），但拿不到 `continuous` 的航向保持 |
| 双码近场摆头/角速度段数 >10 | 航向保持抖振（迟滞带偏窄或底盘报告滞后偏大） | `heading_hold_engage_deg`→2.5、`heading_hold_release_deg`→0.5 拉宽迟滞带，或 `heading_hold_cooldown_sec`→0.45 |
| `dual visual correction no progress / oscillation` 中止 | 两条判据之一触发，失败串已写明是哪条 | **连败判据**（连续 N 步每步改善都 < 门槛）→ 指令根本没起作用：查 `/cmd_vel` 有没有发出去、底盘响不响应、里程计标定。**窗口判据**（N 步净改善 < 门槛，单步可以很好）→ 振荡，两个通道在互相破坏：读 `dual visual feedback` 那行的 theta/e 分解，看是哪一项与预测背离；若 e 与预测相符而 theta 大幅变差，再看那一步的 `Δyaw` —— 横移步打出 `(寄生!)` 就是底盘在横移中带出了转动，见下一行 |
| `no visible translation candidate; independent stable-window budget exhausted` | 前进被 `visible()` 否掉，三个独立稳定窗口耗尽 | 先读同一行尾部的**两段**余量：`wall/pile L/R/T/B` 是**当前位姿**的，`被否的一步 … 预测路径最坏 L/R/T/B … 破的是 X 边` 才是**被否那一步**的 —— 前者宽裕后者破了是正常的，只看前者会以为自相矛盾。再看 `桩码垂直离场=不放行(缺 …)`：缺 `stage=approach`/`z<=直行包络` 且墙距只差几十 mm，就是"要进窗才能进窗"的死锁（见 §6.5b）；此时日志里应当先出现 `dual 前进收缩 …cm → …cm`，没出现说明阶梯没生效。若打的是 `前进收缩阶梯全否 … 不是步长的问题`，那就别再调 `dual.forward_step` —— 去查相机外参/`visibility_margin_px`/桩码是否真的该被免检 |
| 横移步 `Δyaw` 打出 `(寄生! 平移步不该转)` | 底盘执行横移时带出转动（步态耦合，omni 腿式常见） | 单次 >1° 就足以吃掉那一步的修正收益（0.5×sin1°=8.7mm，`dock_tolerance` 才 20mm），累起来会触发窗口判据。先用 `ros2 topic echo /cmd_vel --field angular.z` 确认这段 wz 确实是 0（是 0 = 底盘自己转的，不是我们发的）；再减小 `dual.lateral_step` —— 它按比例缩整个候选菜单（`cap / cap÷2 / cap÷4`），单步横移短了带出的寄生角也小。**注意 `dual.min_lateral_m` 不是这个旋钮**：它只是菜单下限，抬高它只删掉低于门槛的小候选，中段候选照选不误；yaw 与 lateral 是各自独立枚举、最后按 J 一起排序，没有"通道优先级"可调 |
| 转角/直行不准（车没走够量） | 里程计漂移或打滑 | `test_turn_angle` / `test_jog_distance` 实测误差；降速率 |
| 停靠后机器人"锁死"无法遥控 | 旧版本持续发布零速 | 已修复：静默态刹车 0.3s 后释放 `/cmd_vel`（底盘看门狗接管停止） |

---

## 8. 仿真测试

### 8.1 Gazebo + 静态 Tag

```bash
# 1. 启动 Gazebo 仿真世界 (含机器人和 Tag 模型)
ros2 launch my_sim world.launch.py

# 2. 启动停靠
ros2 launch tagdocking docking.launch.py

# 3. 将机器人手动放到距 Tag ~2m 处
#    (在 Gazebo 中拖动机器人模型)

# 4. 触发停靠
ros2 service call /docking_node/start_docking std_srvs/srv/Trigger

# 5. 观察机器人自动驶向 Tag 并停下
```

### 8.2 无 GPU 仿真 (纯 mock)

如果不想启动完整仿真，可以手动发布假检测和 TF。注意 `docking_node` 取位姿
靠 TF（`base_link→tag`），所以 mock 需要**两条腿**：检测消息触发查询，
静态 TF 提供位姿：

```bash
# 腿 1: 假 TF — tag 固定在 base_link 前方 1.0m (x=1.0, y=0)
ros2 run tf2_ros static_transform_publisher \
    --x 1.0 --y 0 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 \
    --frame-id base_link --child-frame-id tag36h11:0
```

```python
# 腿 2: mock_detection.py — 发布假 Tag 检测 (触发 TF 查询)
import rclpy
from rclpy.node import Node
from apriltag_msgs.msg import AprilTagDetectionArray, AprilTagDetection

class MockDetector(Node):
    def __init__(self):
        super().__init__('mock_detector')
        self._pub = self.create_publisher(AprilTagDetectionArray, '/detections', 10)
        self._timer = self.create_timer(0.1, self._publish)  # 10Hz

    def _publish(self):
        msg = AprilTagDetectionArray()
        det = AprilTagDetection()
        det.id = 0
        det.family = "36h11"
        msg.detections = [det]
        self._pub.publish(msg)

rclpy.init()
rclpy.spin(MockDetector())
```

（另需假的 `/odom` 发布节点；距离随 mock TF 固定不变，主要用于验证状态机
流转与话题/服务联通。）

### 8.3 台架走停闭环 (mock_l1w_control)

无狗桌面上验证"停→静止站立→稳定帧→规划→解锁→走"完整闭环与模式时序。
`scripts/mock_l1w_control` 节点名就叫 `l1w_control`，docking 侧默认前缀
`/l1w_control` 免配置；它镜像真桥的语义（锁定拒绝非零 cmd_vel、latched
状态回传），并积分 `/cmd_vel` 发布 `/dog/odom` 让 `ActionExecutor` 里程计
判停闭环：

```bash
# 终端 1: 停靠栈 (无 rtsp_url → 走 ROS 相机桥路径; 也可直接起 docking_node)
ros2 launch tagdocking docking.launch.py

# 终端 2: 模式接口替身 (integrates /cmd_vel → /dog/odom)
ros2 run tagdocking mock_l1w_control

# 终端 3: 假检测 + 假 TF (§8.2 的两条腿) → 触发停靠
ros2 service call /docking_node/start_docking std_srvs/srv/Trigger
```

日志断言：

1. 每停依序出现：`停稳 → 请求静止站立` → `静止站立已锁定` → `已清空旧位姿`
   → `走停 规划` → `请求恢复运动模式` → `运动模式已恢复` → `子步`
2. mock 统计（退出时打印）：`static_stand` 每停恰 1 次、`stand_up` 每次起步
   恰 1 次（逐 tick 重发即有 bug）
3. 锁定窗内无非零 cmd_vel —— 出现 `!!! VIOLATION` 即失败
4. DOCKED 后无 stand_up（保持锁定）；`start_undock` → stand_up 早于首条非零
   cmd_vel；锁定窗内 `cancel_docking` → ~0.5s 内 stand_up
5. 故障注入：`fail_lock:=true` → 一条降级 warn 仍完成；`reject_unlock:=true`
   → ~6s 内 MOTION_FAILED；机动中手动调 `/l1w_control/static_stand` →
   立即 `外部锁定打断机动`
6. 完全不起 mock → 一条"服务不可用"warn，行为与无此功能时一致

---

## 9. 注意事项

### 9.1 坐标系约定 (REP-103)

本系统严格遵循 REP-103 坐标系：

```
       x 前向
         ^
         |
         |
         +------> y 左向
```

- **error_x > 0**: 机器人需要往前走 (Tag 在正前方)
- **error_y > 0**: Tag 偏向机器人左边，需要左移 (全向轮) 或左转 (差速轮)
- **error_yaw > 0**: 机器人需要逆时针 (CCW) 旋转来正对 Tag

### 9.2 TF 依赖

系统通过 **tf2** 查询 Tag 位姿（`lookup_transform(measure_frame 或 base_frame, tag.frame)`），
不直接使用 apriltag_ros 检测消息里的坐标。必须满足：

```
odom → base_link → 相机光学系(image header.frame_id) → tag36h11:0
```

- `apriltag_ros` 的 `publish_tf` 必须为 `true`（launch 已默认设置）
- TF 父系取**图像 header 的 frame_id**（即相机光学系）；双码模式必须将
  `camera_frame` 配为该标定光学系，并提供到 `base_frame` 的完整可信静态 TF。
  ROS 相机模式来自机器人 URDF/robot_state_publisher，RTSP 模式由
  `rtsp_camera` 按 `mount.*` 安装参数发布
- `measure_frame` 为空（默认）时用 `base_frame`（REP-103，x=前 y=左）查询；
  设为其他坐标系则直接从该系查询（结果须同为 REP-103 朝向）

### 9.3 视觉延迟与自适应时效窗口

- 双码模式不使用下述单码自适应时效放行：始终检查 `dual.fresh_sec`、检测/TF
  时间一致性和停稳后至少三帧；锁向运动期间仍独立监视墙码。上游时间戳必须
  真实代表采集时间，RTSP 到达时间戳无法证明画面年龄，需另行实测延迟。
- 单码位姿时效窗口是**自适应**的：`max(camera.max_latency_ms=150ms, 实测检测
  间隔 × latency_interval_margin=3.0)`——按相机真实帧率自调（6Hz→约 500ms，
  30Hz→150ms 下限），容忍偶发丢帧
- 走停式架构对**传输延迟**（如 RTSP 的 0.1~0.5s）天然容忍：机动后 settle
  0.8s 才重新测量；若停稳后位姿仍滞后，加大 `stopgo.turn_settle_sec`
- `tag.fresh_timeout_sec`（2.0s）决定"tag 可见"判定；低帧率相机不需要再放宽
- PoseBuffer 保留最近 30 帧，控制器始终取最新有效帧
- **检测冻结**：机动期间（盲转/盲走）所有检测帧直接丢弃，不进滤波/缓冲——
  规划只用停稳后的新鲜帧

### 9.4 差速轮的横向误差

差速轮 **没有侧移能力（vy=0）**，横向误差由**规划器**消除，不需要底盘层做
任何特殊处理：

- 远距：法线机动（转向→前进→转向）走到 tag 法线上的站位点
- 近距（已对准）：方位+横偏在容差内直接直行，横偏保持在容差内即可
- 转向的本质是"瞄准即走"（aim-and-go）：先对准 tag 再直行，横偏在接触时
  单调收敛到 0

如果差速轮精停效果不好，按序检查：`stopgo.lateral_threshold`（横偏容差，
默认 5cm）→ `tolerance.position_m`（到位容差，5cm）→ 里程计直行精度
（`test_jog_distance`）。

### 9.5 信号安全

系统注册了 **SIGINT 和 SIGTERM** 信号处理器：
- 收到 Ctrl+C 时，阻塞式发布 **约 1 秒零速指令** (100Hz × 1s) 覆盖底盘看门狗
- ROS context 关闭时也会触发 `_safe_stop()`
- **不会出现 Ctrl+C 后机器人还继续前进的情况**

### 9.6 性能要求

- 控制循环: **20 Hz** (50ms 周期)
- 相机检测频率: **≥ 2 Hz 即可完成停泊**（走停式逐帧规划），建议 ≥ 6Hz；
  apriltag_ros 在 CPU 上约 6~30 Hz（取决于分辨率）
- **视觉延迟必须有界且可验证**：settle 不能修复旧画面伪装成新时间戳；双码
  默认新鲜窗口 0.6s，过期帧不放行，TF 迟到则丢弃该帧并等下一帧。
- 里程计质量比相机帧率更关键——所有盲动（含泊出/重试倒车）全靠它闭环

### 9.7 构建说明

- 使用 **ament_cmake** 构建（非 ament_python），因为包含自定义 Action 接口
- `Dock.action` 由 `rosidl_generate_interfaces` 编译生成 Python 模块
- Python 代码通过 CMake 的 `install(DIRECTORY ...)` 安装到 `dist-packages`
- 入口脚本 (`docking_node` 等) 通过 CMake 的 `install(PROGRAMS ...)` 安装到 `lib/tagdocking/`

---

## 10. 附录 A：全部可调参数速查表

**170 个参数，一张表。** 值取自 `config/docking.yaml`（那是实际跑的值）；
`retry.*` 四项 yaml 里没写，列的是代码默认值，标 `°`。全部参数运行期可改：

```bash
ros2 param set /docking_node dual.forward_step 0.08     # 下一次规划即生效
ros2 param list /docking_node                           # 看全部
ros2 param get  /docking_node stopgo.heading_hold_budget_deg
```

> **读表的三条前提**
> 1. **`stopgo.*` 是底盘控制律，`dual.*` 是双码几何与策略**，两套命名空间不是
>    随意分的：`ActionExecutor` 的构造参数全部来自 `stopgo.*`；而 `dual.*` 全要
>    过正性 + 有限性校验循环（布尔过不了，所以 `dual.crouch_enable` 被单独
>    declare）。加新参数时按这条归位。
> 2. **标 ⚠ 的行是"改了会连带改掉别的东西"**，别当单一开关用。
> 3. **标 ✗ 的行是弃用参数**，代码已不读取，保留只为兼容旧 yaml —— 改它没有
>    任何效果。

| 参数 | 当前值 | 大白话释义 |
|---|---|---|
| **— 相机与话题 —** | | |
| `camera.max_latency_ms` | 150 | 位姿时效窗口的下限（ms）。高帧率时它说了算：比这更旧的位姿不许拿来规划 |
| `camera.latency_interval_margin` | 3.0 | 时效窗口 = 实测检测间隔 × 此值。3 = 容忍丢两帧还不算过期 |
| `camera.expected_fps` | 30 | 相机标称帧率，只用来算窗口初值；实测间隔一出来就以实测为准 |
| `camera.lateral_offset_m` | 0.0 | 相机光心相对底盘中心线的横向偏移（+ = 相机偏左）。**只作用于单码路径**，双码完全不经过它。当前相机齐平，故为 0；换装偏心相机时按实测填 |
| `camera_frame` | `camera_color_optical_frame` | 相机光学坐标系名（x 右 / y 下 / z 前） |
| `base_frame` | `base_link` | 底盘坐标系名，所有"距离/横偏"都是相对它说的 |
| `measure_frame` | `""` | 量测参考系覆盖；空 = 用 `base_frame`。给"以充电口而非底盘中心对准"这类需求留的口子 |
| `detection_topic` | `/detections` | AprilTag 检测结果话题 |
| `odom_topic` | `/dog/odom` | 里程计话题。走停的每一步都靠它闭环掐断，配错等于全盲 |
| `pose_buffer.size` | 30 | 时间戳位姿环形缓冲长度（帧） |
| **— 墙码与停泊目标 —** | | |
| `tag.family` | `36h11` | AprilTag 家族 |
| `tag.size` | 0.15 | 墙码物理边长（m）。填错 → 距离整体按比例错 |
| `tag.frame` | `tag36h11:0` | 墙码 TF frame 名，须与 apriltag_ros 的输出一致 |
| `tag.id` | 0 | 墙码 ID |
| `tag.fresh_timeout_sec` | 2.0 | 超过此时长没有新检测就算"陈旧"。6fps + 转向盲区下 1.0 会误判 TAG_LOST，故放宽 |
| `tag.tag_loss_timeout_sec` | 2.5 | 接近途中连续丢码超此时长才退回 SEARCH_TAG。检测流实测有 1.6~3s 空档，填 1s 必抖动 |
| `tag.ema_alpha` | 0.5 | 位姿 EMA 平滑系数（0 = 重平滑，1 = 原始）。法线方向走圆周 EMA，不会在 ±180° 处炸 |
| `tag.max_pose_jump_m` | 0.3 | 单帧跳变超过此距离直接丢弃，挡运动模糊坏帧 |
| `dock_target.distance` | 0.55 | 单码路径的停泊距离（m，底盘到码）。**双码路径用 `dual.dock_distance`**，两者互不相干 |
| `dock_target.lateral_offset` | 0.0 | 停泊点相对码中心的横向偏置（m），0 = 正对 |
| `dock_target.yaw_offset_deg` | 0.0 | 停泊时相对"正对码"的朝向偏置（°） |
| **— 底盘 —** | | |
| `base.type` | `omni` | 底盘类型：`diff_drive` / `omni` / `quadruped`。决定有没有横移能力，进而决定规划器选哪套动作 |
| `base.cmd_vel_topic` | `cmd_vel` | 速度指令话题 |
| `base.l1w_prefix` | `/l1w_control` | 狗的模式服务前缀（静止站立 / 起立 / 阻尼都挂在它下面） |
| **— 走停控制律 `stopgo.*` —** | | |
| `stopgo.lateral_threshold` | 0.05 | 横偏容许带（m）。超出才值得动手去修，否则直行 |
| `stopgo.yaw_threshold_deg` | 10.0 | 方位容许带（°），同上 |
| `stopgo.tune_angle` | 0.0 | 差速车的斜向修正角（rad），0 = 关 |
| `stopgo.jog_min` | 0.05 | 单步前进下限（m）。比这更短的步不值得起停一次 |
| `stopgo.jog_max` | 0.20 | 单步前进上限（m）。⚠ **只钳制单码 GeometryPlanner，双码完全不受它管**（双码的上限是 `dual.forward_step` 与纯直行区的"一次走完"） |
| `stopgo.jog_linear_rate` | 0.08 | 前进恒速（m/s）。走停不做加减速曲线，就是这个常速 |
| `stopgo.jog_angular_rate` | 0.3 | 转向恒速（rad/s） |
| `stopgo.min_angular_rate` | 0.12 | 下发角速度的**下限**（rad/s）。l1w_control 有 `min_angular_z = 0.10` 死区，低于它的命令被整条截成 0 —— 这个参数是结构性抬底，让死区不可达 |
| `stopgo.lateral_rate` | 0.12 | 横移恒速（m/s，仅 omni / 四足） |
| `stopgo.jog_odom_scale` | 1.0 | 前进里程计尺度标定。里程计说走了 1m 实际走了 0.95m 就填 0.95 |
| `stopgo.jog_backward_odom_scale` | 1.0 | 后退的尺度，单列是因为狗前后步态不对称 |
| `stopgo.lateral_odom_scale` | 1.0 | 横移的尺度 |
| `stopgo.turn_settle_sec` | 1.5 | 转完停稳后等图像清晰的时长（s）。加长是为了避免模糊帧被跳变拒绝误杀 |
| `stopgo.turn_undershoot` | 0.75 | 只转命令角的百分之多少。✗ 仅 legacy `full=False` 路径读；走停/泊出盲转走 `full=True`，不经过此值 |
| `stopgo.turn_lead_per_speed` | 0.30 | 转向提前量（rad per rad/s）：剩余角 ≤ 此值 × 当前角速率就提前发零速，靠滑行补足。由对接日志反推滞后 0.30-0.43s。欠转/过转量恒定时微调这里 |
| `stopgo.turn_slow_rad` | 0.14 | 距目标角此值以内减速到半速，减小惯性冲量 |
| `stopgo.max_turn_step` | 0.17 | 单次转向硬上限（rad ≈ 10°）。小步转 + 每步重看，低帧率下码不易转出视野 |
| `stopgo.small_turn_rad` | 0.1 | 小于此角度的转向全程半速 |
| `stopgo.theta_shrink_ratio` | 2.0 | 动态方位容差 = `max(yaw_threshold, 距离/此值)`：远处松、近处紧 |
| `stopgo.drift_tol` | 0.15 | 前进途中允许的方位漂移上限（rad） |
| `stopgo.turn_creep_linear` | 0.0 | ✗ 已弃用。"边走边转"叠加前进会驶出目标横向范围，现固定纯原地转 |
| **— 行进中航向保持 `stopgo.heading_hold_*`（双码纯直行区）—** | | |
| `stopgo.heading_hold_enable` | true | 总开关。现场一键回滚：`ros2 param set` 后下一趟即生效 |
| `stopgo.heading_hold_rate` | 0.12 | 接通时下发的角速度幅值（rad/s）。⚠ 代码用 `max(abs(此值), min_angular_rate)` 抬底，配进死区也不会静默失效；调大会恶化单周期粒度（0.3 → 0.86°/周期，一周期跨掉 2/3 迟滞带） |
| `stopgo.heading_hold_engage_deg` | 2.0 | 歪到多少度开始掰（°）。由来：`0.5×sin(2°) = 17.5mm < dock_tolerance(20mm)` —— 门槛就该定在"残余误差代价 < 停泊容差"这点上 |
| `stopgo.heading_hold_release_deg` | 0.7 | 掰回到多少度松手（°）。⚠ **绝不取 0**：瞄 0 断开必过冲换符号，直接抖振。迟滞带 1.3° > 最短接通粒度 1.03° |
| `stopgo.heading_hold_min_engage_sec` | 0.15 | 最短接通时长（s，3 个控制周期 ≈ 一个步态相位）。单周期脉冲底盘不一定响应 |
| `stopgo.heading_hold_cooldown_sec` | 0.30 | 断开后的冷却（s），让下一次决策基于已沉降的里程计读数 |
| `stopgo.heading_hold_budget_deg` | 15.0 | 单程累计下发角上限（°），耗尽后告警并退回纯直行。⚠ 这**不是**安全边界（那由符号规则给），是"底盘不响应 / 里程计疯了"的诊断闸。**有下限守卫**：必须 > `rate × min_engage_sec`（按抬底后的 rate 算，默认 1.03°），配低了会被 warn 掉并本趟退回纯直行 —— 不会假装开着 |
| **— 静止站立（呼吸抑制）`posture.*` —** | | |
| `posture.enable` | false | 总开关。狗站着会"呼吸"，位姿一直在抖；开了就每个停看点先锁定再量测。无桥时自动降级停用 |
| `posture.static_settle_sec` | 1.2 | 停 → 量测的最短间隔（s）：RTSP 延迟 + 锁定过渡 + 呼吸衰减。与 `turn_settle_sec` 同起点取 max |
| `posture.lock_ack_timeout_sec` | 2.0 | 等锁定确认的超时（s）；超时则本次停降级为"未锁定继续"，不判失败 |
| `posture.unlock_ack_timeout_sec` | 2.0 | 等 `motion_enabled==True` 的超时（s）。固件退出锁定有滞后，只有这个 Bool 变 True 才真的能走 |
| `posture.unlock_retries` | 2 | 起立命令重发上限。⚠ 耗尽 → MOTION_FAILED（狗还锁着，这一条不可降级） |
| `posture.min_stable_frames` | 3 | 规划前要求的连续新鲜帧数（1 = 关闭） |
| `posture.stable_frame_timeout_sec` | 2.5 | 等不满上面那 N 帧就带当前位姿硬规划（s），防闪烁卡死 |
| `posture.service_wait_sec` | 1.0 | 服务发现宽限（s），超过即判定无桥、停用并告警一次 |
| **— 充电收尾 `charge.*` —** | | |
| `charge.enable` | true | DOCKED 之后的收尾总开关 |
| `charge.passive` | true | 阻尼（泄力）步开关；false 时收尾止于锁定站立 |
| `charge.static_stand` | false | 先锁定再阻尼；false = 跳过锁定直接阻尼 |
| `charge.static_ack_timeout_sec` | 3.0 | 等锁定确认的超时（s） |
| `charge.lie_down_settle_sec` | 4.0 | ✗ 已弃用，当前收尾流程不再使用趴下动作 |
| `charge.passive_settle_sec` | 2.0 | 阻尼服务受理后的等待（s）。注意这是等时间，不是等硬件反馈 |
| `charge.retries` | 1 | 每步服务的重试次数 |
| `charge.service_wait_sec` | 1.0 | 无桥宽限（s）。超时只降级告警，**DOCKED 仍算成功** |
| **— 单码两阶段直行 `final_straight.*` —** | | |
| `final_straight.enable` | true | 两阶段停泊开关；false 恢复单阶段 |
| `final_straight.start_distance` | 0.85 | 直行阶段起点（m），须 > `dock_target.distance` |
| `final_straight.yaw_threshold_deg` | 3.0 | 入口方位门槛（°）：进入直行距离时超此值报导航失败。近场方位修正也用同一值 |
| `final_straight.normal_yaw_threshold_deg` | 2.0 | 近场法线对准门槛下限（°）。它低于噪声，实际生效的是下面那条 |
| `final_straight.normal_turn_min_deg` | 6.0 | 近场法线对准的**实际**最小出手角（°）：小于它转了也是白转 |
| `final_straight.entry_lateral_m` | 0.07 | 入口横向门槛（m），一次性检查，超了报导航失败 |
| `final_straight.lateral_threshold_m` | 0.02 | 近场横移修正 / 捷径的横向门槛（m），比入口更紧。⚠ 须 < `stopgo.lateral_threshold`，否则规划器不会滑 |
| `final_straight.tighten_distance` | 1.3 | 远/近分界（m）：≤ 它进近场精调（法线对准 + 横移 + 直行），> 它只做远场粗对准 + 纯前进。建议 ≥ `start_distance + jog_max` |
| `final_straight.far_yaw_threshold_deg` | 15.0 | 远场粗对准的方位门槛（°）。远场法线不可靠，不宜微调 |
| `final_straight.far_lateral_m` | 0.20 | 远场粗对准的横向门槛（m）。远场只要求大方向对，横偏走到近场再说 |
| `final_servo.distance` | 0.20 | 进入 FINAL_SERVO 的距离（m） |
| `final_servo.yaw_tol_deg` | 15.0 | 到达停泊距离即判定的方位门槛（°）。直行阶段不再追角度，15° 内都接受 |
| `final_servo.max_linear_speed` | 0.05 | ✗ legacy 伺服速度，走停流程不读 |
| `final_servo.max_yaw_speed` | 0.2 | ✗ 同上 |
| **— 双码几何与策略 `dual.*` —** | | |
| `dual.enable` | false | 双码路径总开关。开了才走墙码 + 桩码那一整套，否则是单码流程 |
| `dual.crouch_enable` | false | 靠近时趴下再起立的开关 |
| `dual.camera_info_topic` | `/camera_sync/camera_info` | 内参话题。双码的可见性预测全靠它，没有就不动 |
| `dual.projection_mode` | `raw` | 用原始 K+D 还是去畸变后的 P。⚠ 显式配置：话题重映射**不等于**已去畸变 |
| `dual.wall_tag_size` | 0.15 | 墙码边长（m） |
| `dual.pile_tag_id` | 51 | 桩码 ID，必须与 `tag.id` 不同 |
| `dual.pile_tag_size` | 0.05 | 桩码边长（m）。5cm 决定了它的可见半径只有 ~0.9m，也就决定了站位必须在 1.8m |
| `dual.dock_distance` | 0.50 | 双码停泊距离（m，墙码光学 z） |
| `dual.dock_tolerance` | 0.02 | 停泊容差（m）。差在此内就算到位；不足由收缩步补，冲过由回退修剪。也是前进收缩阶梯的下限 |
| `dual.mount_tolerance_deg` | 2.0 | 外参安装角容差（°），超了启动就报，不让带着错外参上场 |
| `dual.observation_distance` | 1.8 | 观察站位（m）。由来：站立视角下 5cm 桩码在 ~0.9m 处仍清晰可见，而角度必须在桩码可见的最后窗口一次对准完毕 |
| `dual.observation_tolerance` | 0.1 | 站位容差（m）。⚠ 它和上面那条一起构成**直行包络** 1.90m —— 桩码垂直离场放行与丢失闭锁都用这个和 |
| `dual.straight_start_distance` | 1.70 | = obs − tol。现只剩两处用法（排序校验、转向步长远近分档）。⚠ 放行与闭锁已改用直行包络，别混 |
| `dual.steering_stop_distance` | 1.0 | 纯直行区入口（m）。⚠ **不要拿它当"关掉区内转向"的开关** —— 它同时管着"一次连续直行"与 `visible()` 的放行，调低会把一次停到位一并关掉。要更保守地限制区内转向请调 `straight_yaw_lag_deg` |
| `dual.forward_step` | 0.10 | 区外单步前进上限（m）。被否时按 `d / d÷2 / d÷4` 逐档收缩再试（见 §6.5b 的收缩阶梯） |
| `dual.reverse_step` | 0.10 | 单步后退上限（m），也是冲过停泊点后回退修剪的单步封顶 |
| `dual.lateral_step` | 0.05 | 横移候选菜单的**刻度尺**（m）：菜单是 `cap / cap÷2 / cap÷4 / 残差 / 下限`，调它等于整体缩放。底盘横移带出寄生 yaw 时，调小的是这个 |
| `dual.min_lateral_m` | 0.003 | 横移候选的**下限**（m）：比它短的候选直接删掉。⚠ 它不是"优先走 yaw 通道"的旋钮 —— 抬高只删小候选，中段照选不误 |
| `dual.lateral_tolerance_m` | 0.02 | 横偏容差（m），进到此内就不再横移 |
| `dual.yaw_step_deg` | 8.0 | 远场（z > 1.70）单步转向上限（°） |
| `dual.yaw_fine_step_deg` | 3.0 | 近场（z ≤ 1.70）单步转向上限（°）。⚠ 3° 在远场是灾难：底盘停止滞后 ~2° 与命令同量级，提前量钳位会让每步只转一半 |
| `dual.straight_yaw_tol_deg` | 1.5 | 区外 bearing 微调门槛（°），小于它只直行 |
| `dual.straight_yaw_max_turns` | 3 | 连续转向次数上限，前进一步即清零。防"光转不走" |
| `dual.straight_yaw_lag_deg` | 2.0 | 底盘单步转向的停止滞后（°）。它是纯直行区内那条盈亏平衡门槛的**代价项** —— 要更保守地禁区内转向就调大它 |
| `dual.align_tolerance_deg` | 3.0 | 两码对准门槛（°）。⚠ 是**各自**的光学 bearing，不是两者平均 |
| `dual.align_hold_sec` | 0.5 | 对准要持住多久才算数（s），同一个沉降窗口内的不重复帧 |
| `dual.prealign_tolerance_deg` | 5.0 | 趴下前单码粗对准的收敛门槛（°）。双码是**精调器不是收敛器**：锁定时残留 20° 它得一步几度啃 30 步 ≈ 70s，必然超时 |
| `dual.prealign_step_deg` | 8.0 | 粗对准单步转向上限（°） |
| `dual.prealign_max_steps` | 12 | 粗对准步数预算，耗尽后告警并移交双码（不判失败） |
| `dual.settle_sec` | 1.5 | 双码停稳等待（s） |
| `dual.min_frames` | 3 | 规划前所需的最少稳定帧 |
| `dual.fresh_sec` | 0.6 | 两码共用的新鲜度窗口（s） |
| `dual.tf_skew_sec` | 0.02 | 两码时间戳允许的错开（s）。超了就不是"同一时刻的两码"，视差解算无意义 |
| `dual.stable_position_m` | 0.03 | 相邻帧光心位移上限（m），用来判"真的停稳了" |
| `dual.missing_confirm_sec` | 1.5 | 桩码连续不可见多久才确认丢失（s） |
| `dual.missing_timeout_sec` | 8.0 | 确认丢失后多久判失败（s） |
| `dual.pile_fresh_timeout_sec` | 2.0 | ✗ 已弃用，现在两码统一由 `dual.fresh_sec` 管 |
| `dual.qualification_sec` | 12.0 | 航向证据的保鲜期（s）。超过就不算"持住对准"，放行作废 |
| `dual.pile_lock_distance` | 0.30 | 桩码光学 z 到此值切回站立（m）。再近桩码就离开趴姿视野了；⚠ 要求先对准好航向 |
| `dual.crouch_settle_sec` | 3.0 | 趴下动作的等待（s）。没有专门的趴姿状态量，只能靠"服务成功 + 计时" |
| `dual.posture_retries` | 2 | 趴下/起立的服务重试次数，耗尽则双码中止 |
| `dual.reverse_limit` | 0.8 | 获取阶段后退的累计上限（m） |
| `dual.reverse_count` | 16 | 获取阶段后退的步数上限 |
| `dual.standoff_reverse_limit` | 1.0 | 站位调整后退的累计上限（m），与上面分开记账 |
| `dual.standoff_reverse_count` | 20 | 站位调整后退的步数上限 |
| `dual.visibility_margin_px` | 8.0 | 码的包围框离画面边缘至少要留几个像素 |
| `dual.visibility_sample_pad_px` | 2.0 | 在上面基础上再加的安全垫。两者之和（10px）才是实际门槛，日志里打的 `required` 就是它 |
| `dual.visibility_samples` | 12 | 沿整条预测轨迹采样几个点做可见性检查。⚠ 检查的是**整条路径**，不只终点 —— 这就是大步长不牺牲可见性的原因 |
| `dual.score_improvement` | 0.002 | 一个候选至少要把代价 J 改善这么多才配被采纳 |
| `dual.feedback_min_improvement` | 0.01 | 视觉反馈窗口的净改善门槛。⚠ 达不到就判"原地踏步"中止 —— 别拿它当报警器的关闭开关 |
| `dual.feedback_fail_windows` | 3 | 连续多少步修正没改善就中止（连败判据），同时也是净改善窗口的长度 |
| `dual.no_candidate_windows` | 3 | 连续多少个**独立稳定窗口**一个候选都发不出就判失败。定时器重试不计数 |
| `dual.max_actions` | 160 | 整趟动作数硬上限，兜住病态振荡 |
| `dual.acquire_timeout_sec` | 60.0 | 获取双码的超时（s） |
| `dual.observe_timeout_sec` | 120.0 | 获取之后 observe 阶段的独立超时（s） |
| `dual.camera_wait_sec` | 5.0 | 等 CameraInfo 的宽限（s），等不到就不上场 |
| `dual.action_timeout_sec` | 6.0 | 单个动作的超时（s）。⚠ 纯直行区"一次走完"会按实际行程放宽，否则 6s 掐死 6.3s 的长直行 |
| `dual.response_timeout_sec` | 2.0 | 发了命令多久里程计还没动就算底盘没响应（s） |
| `dual.action_startup_sec` | 0.6 | 起步瞬态宽限（s），期间不判反向 —— 四足换步时先退一点是正常的 |
| `dual.odom_fresh_sec` | 0.5 | 起步前与行程中要求的里程计新鲜度（s） |
| `dual.odom_noise_m` | 0.001 | 里程计位移噪声地板（m），门槛推导的基准 |
| `dual.odom_noise_rad` | 0.002 | 里程计角度噪声地板（rad ≈ 0.11°），同上 |
| `dual.log_period_sec` | 2.0 | 双码状态日志的打印周期（s） |
| **— 搜索 / 重试 / 泊出 / 超时 / 安全 —** | | |
| `search.angular_speed` | 0.3 | 搜索旋转速度（rad/s） |
| `search.step_angle_deg` | 30.0 | 每步旋转角度（°），建议 ≥ 10°（太小会被半速逻辑拖慢） |
| `search.pause_time_sec` | 3.0 | 每步之间的检测停留（s）。⚠ 检测流有 1.6~3s 空档，填 1.5s 会整个错过 |
| `search.initial_look_sec` | 4.0 | 第 0 步的停留（s），开局要多等 RTSP / 检测流冷启动 |
| `search.hold_time_sec` | 0.5 | 码持续可见多久才算锁定（s） |
| `search.search_direction` | 1 | 从没见过码时的起始转向：+1 = 逆时针，−1 = 顺时针 |
| `search.timeout_sec` | 120.0 | 搜索整体超时（s） |
| `search.rotate_time_sec` | 0.8 | ✗ 已弃用，角度步进化之后不再读取 |
| `retry.max_retries` | 2 ° | 失败后盲退重锁的次数上限 |
| `retry.backup_distance` | 0.5 ° | 每次重试盲退的距离（m） |
| `retry.linear_rate` | 0.08 ° | 重试盲退的速率（m/s） |
| `retry.timeout_sec` | 15.0 ° | 单次重试的超时（s） |
| `undock.backup_distance` | 0.8 | 泊出盲退距离（m，正值表示后退） |
| `undock.linear_rate` | 0.2 | 泊出后退速率（m/s） |
| `undock.turn_angle_deg` | 90.0 | 泊出转向角（°，正 = 逆时针） |
| `undock.angular_rate` | 0.3 | 泊出转向速率（rad/s） |
| `undock.timeout_sec` | 30.0 | 泊出整体超时（s），里程计不走时靠它兜底落 MOTION_FAILED |
| `tolerance.position_m` | 0.05 | 单码路径判到位的前后容差（m） |
| `tolerance.yaw_deg` | 10.0 | 单码路径判对准的角度容差（°） |
| `tolerance.stable_time_sec` | 1.0 | 到位状态要保持多久才算数（s） |
| `safety.minimum_distance_m` | 0.15 | 安全下限（m）：比这更近就停 |
| `timeout_sec` | 300.0 | 整趟停泊的总超时（s） |
| `align_timeout_sec` | 15.0 | ALIGN 阶段超时（s，遗留态，当前流程跳过） |
| `approach_timeout_sec` | 180.0 | APPROACH 阶段超时（s）。⚠ 静止站立每停 +1.5-2.5s，双码 APPROACH 要跨 1.8→0.6m 约 24 步且纠偏全程同钟，所以从 90 抬到 180 |
| `final_servo_timeout_sec` | 30.0 | FINAL_SERVO 阶段超时（s） |

---

## 依赖

```
rclpy, std_msgs, std_srvs, geometry_msgs, nav_msgs
action_msgs, apriltag_msgs
tf2_ros, tf2_geometry_msgs
apriltag_ros (运行时)
python3-opencv, python3-numpy, python3-yaml   # rtsp_camera / calibrate_rtsp
```

## License

Apache-2.0
