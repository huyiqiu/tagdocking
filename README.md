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
              停稳确认 (误差在容差内持续1s)      盲退 0.5m 重新锁定 (≤2次)
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
停稳 → settle 等图像清晰 → 采一帧新鲜测量 → 规划器算出完整小段路径
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

### 模块结构

```
tagdocking/
├── action/Dock.action           # ROS2 Action 定义
├── config/docking.yaml          # 全部参数
├── config/rtsp_camera_info_example.yaml  # RTSP 相机内参示例
├── launch/docking.launch.py     # 启动文件 (相机源: ROS话题桥 / RTSP桥 二选一)
├── scripts/
│   ├── docking_node             # 停泊主节点入口
│   ├── camera_info_bridge       # ROS 相机时间戳同步桥入口
│   ├── rtsp_camera              # RTSP 相机桥入口 (机器狗)
│   ├── calibrate_camera         # 棋盘格标定 (ROS 相机, ssh -X)
│   ├── calibrate_rtsp           # 棋盘格标定 (RTSP, 纯 ssh 无 GUI)
│   ├── test_apriltag            # 相机直测 tag 距离/横向 (验证内参+TF)
│   ├── test_turn_angle          # 转角精度测试 (cmd vs 里程计)
│   └── test_jog_distance        # 直行精度测试 (cmd vs 里程计)
├── tagdocking/
│   ├── docking_node.py          # 主节点 — 20Hz 控制循环, 集成所有子系统
│   ├── state_machine.py         # 状态机 + 超时/重试/泊出管理
│   ├── geometry_planner.py      # 几何规划器 — 法线机动(turn-drive-turn)/两阶段直行
│   ├── action_executor.py       # 动作执行器 — 里程计航位推算闭环
│   ├── camera_info_bridge.py    # 相机话题时间戳同步桥 (ROS 相机模式)
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

相机侧（二选一，由 launch 的 `rtsp_url` 参数决定）：

```
ROS 相机模式:  相机驱动(/image_raw+/camera_info) ─► camera_info_bridge ─► /camera_sync/*
RTSP 模式:     rtsp_camera (拉流+内参合成+静态TF) ─────────────────────► /camera_sync/*
                              └► /camera_sync/image_raw + /camera_sync/camera_info
                                        └► apriltag_node ─► /detections + TF
```

---

## 2. 快速开始

### 2.1 前置条件

- ROS2 Humble 已安装
- `apriltag_ros` 已安装并能正常检测 Tag
- 相机接入二选一：
  - **ROS 相机模式**（差速车等）：相机驱动发布 `/image_raw` 和 `/camera_info`，且已标定
  - **RTSP 模式**（机器狗）：相机提供 RTSP 流，先用 `scripts/calibrate_rtsp` 标定（见 2.6）
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
| `camera_info_file` | 内参 YAML（`calibrate_rtsp` 生成，rtsp 模式必填） |
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
| `FINAL_SERVO` | 5 | 停稳确认（位置/朝向在容差内且静止，持续 1s） | 30s (`final_servo_timeout_sec`) |
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
- **已对准直行捷径**：方位误差 ≤ 直行门槛且横偏 ≤ `stopgo.lateral_threshold`
  时不再机动，直接直行（避免近场 normal 抖动引起的无谓摆头）

**阶段 2（纯直行，不修角）**：`final_straight` 启用时，距 tag ≤
`final_straight.start_distance`（0.85m）即**无条件直行**，像停车入库一样
不再调角——再往前已无空间调位姿，直行中横向误差保持到最终位姿。

- **直行失败检查（一次性）**：首次进入直行距离时，方位误差 >
  `final_straight.yaw_threshold_deg`（launch 默认 5°，docking.yaml 10°）
  说明对准过差、入库会撞偏 → 报导航失败（进 RETRYING）。只判一次，防止
  直行中方位自然漂动误触发
- `final_straight.enable: false` 恢复单阶段（全程带角度修正）
- `start_distance` ≤ `dock_target.distance` 视为误配置，静默回退单阶段
- 兜底：走停迭代上限 40 次（正常收敛远在之前），单帧近场噪声一次超限
  往往是抖动，重试机制给重新靠近的机会

**全向轮附加**：横偏超 `stopgo.lateral_threshold` 时直接**横移**消横偏
（差速轮无此自由度，靠阶段 1 的转向机动消除）。

#### FINAL_SERVO — 停稳确认

APPROACH 中距目标 ≤ 2×`tolerance.position_m`（0.10m）即转入。此阶段不再
有独立控制律——走停循环在容差内即停，本状态只做**多帧稳定确认**。

**成功判定（三个条件同时满足，持续 `tolerance.stable_time_sec` 1s）**：
1. `|dist − dock_target.distance|` < `tolerance.position_m`（0.05m）
2. `|lat − lateral_offset|` < `tolerance.position_m`
3. 方位误差 < `tolerance.yaw_deg`（10°）且无运动指令（走停式下停稳即静止）

> 这里的 yaw 是**方位角**（bearing = atan2(lat, dist)，即机器人指向 tag 的
> 方向偏差），不是 tag 自身朝向。

**安全保护**：
- 距离 < `safety.minimum_distance_m`（0.15m）时强制判定 DOCKED，防止碰撞
- 超时 30s 后若误差 < 3×容差则接受当前位姿（尽力而为），否则 TIMEOUT

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
`ros2 launch tagdocking docking.launch.py --show-args`）。

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

### 6.3 两阶段停泊 (final_straight.*)

```yaml
final_straight.enable: true           # 两阶段开关; false 恢复单阶段
final_straight.start_distance: 0.85   # m — 直行阶段起点 (须 > dock_target.distance, 否则回退单阶段)
final_straight.yaw_threshold_deg: 10.0  # deg — 直行失败门槛 (launch 默认 5.0 会覆盖此值)
```

进入 `start_distance` 后无条件纯直行不调角；首次进入时方位误差超门槛报
导航失败（自动重试）。见 §4.3 APPROACH。

### 6.4 搜索参数 (search.*)

```yaml
search.angular_speed: 0.3             # rad/s — 每步旋转速度
search.step_angle_deg: 30.0           # deg — 每步旋转角度 (≥10°, 太小会半速拖慢)
search.pause_time_sec: 1.5            # s — 每步之间的检测停留时长
search.hold_time_sec: 0.5             # s — tag 持续可见此时长才锁定
search.search_direction: 1            # +1=CCW, -1=CW (从未见过 tag 时的起始方向)
search.timeout_sec: 60.0              # 整体搜索超时
# search.rotate_time_sec 已弃用 (角度步进化后不再读取)
```

### 6.5 容差 / 安全 / 超时

```yaml
tolerance.position_m: 0.05            # m — 前后/横向 ±5cm 视为到位
tolerance.yaw_deg: 10.0               # deg — 方位 ±10° 视为对准
tolerance.stable_time_sec: 1.0        # s — 稳定持续此时长才判 DOCKED

safety.minimum_distance_m: 0.15       # m — 距 tag 更近直接判 DOCKED, 防碰撞
timeout_sec: 120.0                    # 全局停泊超时 (覆盖所有活动态)
approach_timeout_sec: 60.0            # APPROACH 阶段超时
final_servo_timeout_sec: 30.0         # FINAL_SERVO 阶段超时
# final_servo.max_linear_speed / max_yaw_speed: 遗留, 走停式下未使用
```

### 6.6 重试与泊出 (retry.* / undock.*)

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

### 6.7 相机与话题

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
| 转向后丢 tag 回 SEARCH_TAG | 转太快出 FOV / settle 太短 | 减小 `stopgo.max_turn_step` 或 `jog_angular_rate`；加大 `stopgo.turn_settle_sec` |
| 直行失败报导航失败（自动重试） | 进入直行距离时方位误差 > 门槛 | 看"直行失败"日志里的方位误差；阶段 1 对准不足可收紧 `stopgo.yaw_threshold_deg`，或放宽 `final_straight.yaw_threshold_deg` |
| 重试耗尽落 MOTION_FAILED | 多次直行失败 / 倒车时里程计不走 | 看日志定位具体原因；检查底盘是否响应 `/cmd_vel`、里程计话题是否正确 |
| 停泊位置系统性偏前/偏后 | `tag.size` 不对 / 相机内参不准 | `test_apriltag --known-distance` 验证实测距离，重标定 |
| 停泊位置横向偏 | 相机安装 TF（`mount.*`/URDF）不准 | RTSP 模式校准 `camera_mount_y`；差速车检查 URDF 相机外参 |
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
- TF 父系取**图像 header 的 frame_id**（即相机光学系），与 `camera_frame`
  参数无关（该参数仅信息用途）；关键是该光学系到 `base_link` 有静态 TF——
  ROS 相机模式来自机器人 URDF/robot_state_publisher，RTSP 模式由
  `rtsp_camera` 按 `mount.*` 安装参数发布
- `measure_frame` 为空（默认）时用 `base_frame`（REP-103，x=前 y=左）查询；
  设为其他坐标系则直接从该系查询（结果须同为 REP-103 朝向）

### 9.3 视觉延迟与自适应时效窗口

- 位姿时效窗口是**自适应**的：`max(camera.max_latency_ms=150ms, 实测检测
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
- **视觉延迟无严格要求**：走停式在停稳后才测量，延迟被 settle 窗口吸收
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
