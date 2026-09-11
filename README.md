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
    def publish_arc(self, linear_rate, angular_rate): ...  # 边走边转 (保留接口, 主流程未用)
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
- `publish_arc`（边走边转）已弃用：叠加前进速度会让车驶出目标横向范围；
  角度精度靠里程计校准的全量盲转保证。若差速轮原地转需克服静摩擦，
  加大 `stopgo.jog_angular_rate`，不要叠加前向速度

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
stopgo.lateral_threshold: 0.05        # m — 横偏容许 ±5cm, 超过才修正(全向→横移/差速→机动)
stopgo.yaw_threshold_deg: 10.0        # deg — 角度容许 ±10°, 超过才转向
stopgo.jog_min: 0.05                  # m — 单步最小前进距离
stopgo.jog_max: 0.15                  # m — 单步最大前进距离 (小步: 运动期短, tag 不易模糊丢失)
stopgo.jog_linear_rate: 0.08          # m/s — 前进速度
stopgo.jog_angular_rate: 0.3          # rad/s — 转向速度
stopgo.lateral_rate: 0.08             # m/s — 横移速度 (omni/quadruped)
stopgo.turn_settle_sec: 0.8           # s — 转向后等图像清晰 (RTSP 相机可加大到 1.0~1.5)
stopgo.turn_undershoot: 0.75          # 只转指令角的 75%, 防过冲 (legacy 步进转向用; 法线机动/搜索用全量盲转)
stopgo.max_turn_step: 0.17            # rad (~10°) — 单次转向硬上限
stopgo.small_turn_rad: 0.1            # rad — 小于此角度的转向减半速
stopgo.theta_shrink_ratio: 2.0        # 动态方位容差 = max(yaw_threshold, dist/ratio)
stopgo.drift_tol: 0.15                # rad — 前进中方位漂移上限 (走弧时提前停重规划)
```

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
camera.lateral_offset_m: 0.03         # m — 相机光学中心相对底盘中心线的横向偏移 (+ = 相机偏左)
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

### 6.5b 双二维码光学走停 (dual.*)

墙码 `36h11:0 / 0.15m`、桩码 `36h11:51 / 0.05m`。两码中心须处于同一
进桩中心线竖直平面；不是两码左右位置平均居中，而是**每个码各自水平方向居中**。
`dual.enable=false` 旁路此控制器；显式 launch `dual_enable:=false` 会覆盖 YAML true。

1. **找双码**：无墙码执行有界原地搜索（最多一圈且受搜索总超时）；只有墙码先停看，
   在持续新鲜墙码检测中确认缺小码后，每次退 0.05m。全轮后退最多 0.30m/6 次，
   获取双码默认限时 45s，获取后 observe 独立限时 90s；仍受总体任务超时限制。
   只有小码、无可信墙距或外参不许盲进。
2. **观察位**：双码有效、先小转/横移使其居中，再在相机墙码深度 1.5m ±0.1m 附近
   建立进桩阶段。太近仍受同一后退预算限制；预算不足报失败，需人工重新布置起点。
3. **双码进桩**：每窗联合评估正负转向和正负横移，只发一个可见且显著改善的动作；
   硬上限转 3°、横移 0.03m、前进 0.05m（配置不能放大硬上限）。
   两码独立 bearing、航向 `min(tol,1°)`、横偏 2cm 均通过后才允许前进；
   航向门不再禁止横移。每 jog 后重新双码纠偏，不在 1.5m 或 1m 直接关闭纠偏。
4. **单向锁向末段**：本轮已经双码对齐并前进，最近合格观测未过资格时限，且墙距
   ≤1.0m，在持续新鲜墙码帧中确认小码缺失 ≥1.5s 才锁向。远处/未对齐丢小码
   停车等待后失败；不降级墙码纠角。锁向后小码再现也不转/横移，墙码丢失或流停就停。
5. **到位**：合法进桩后的新鲜停稳墙距 0.50m ±0.02m 才先零速、cancel，再 DOCKED。
   最后 jog 可小于通用 jog_min；严重越界失败，绝不自动倒退或回搜索。随后一次性
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

**日志**：双码发现/有效固定 key 默认每 2s 输出，首条及内部阶段变化立即输出，附 suppressed
计数，不减少检测处理。动作启动记录双码 xyz、独立 bearing、theta/e、预测 J 和图像余量；
完成记录 signed odom，停稳后记录实际 J。内部 observe 不等于外层 APPROACH 已对准。

**时间与帧门**：每个动作停车至少 `max(1.5, dual.settle_sec, posture.static_settle_sec)` 秒，
不依赖 posture.enable。清空旧观测后要求至少 `max(3, dual.min_frames)` 个不同时间戳新帧。
双码使用同一检测 stamp 的非阻塞 TF 查询；TF stamp 差限 `dual.tf_skew_sec=0.02`，
新鲜限 `dual.fresh_sec=0.6` 秒。重复、过期、运动期、停稳前或缺码帧不推进双码稳定；
没有“等不够帧超时照走”。解锁期间待发计划过期会丢弃并停车重测。
流整体中断不等于小码正常退出视野。真实相机/TF 必须同钟且发布采集时间。

参数在 `config/docking.yaml` 的 dual 区集中维护；launch 可传 `dual_observation_distance:=1.5`、
`dual_forward_step:=0.05`、`dual_settle_sec:=1.5`、`dual_dock_distance:=0.50` 等同名下划线参数
（默认空值保留 YAML）；保留已验证的 stopgo 速度和里程计比例。
`dual.straight_start_distance` **兼容旧名字但改变含义**：只作近距缺小码资格门，非距离锁向。
`dual.pile_fresh_timeout_sec` 已弃用，统一改用 `dual.fresh_sec`。
其它重点：`missing_confirm_sec=1.5`、`missing_timeout_sec=8`、`qualification_sec=8`、
`max_actions=160`、`dock_tolerance=0.02`。这些阈值尤其小码退出距离必须实测，不能仅从
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
