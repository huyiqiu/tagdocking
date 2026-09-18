# tagdocking 终端命令速查

tagdocking 包的全部终端命令:启动、参数设置、标定、台架测试、调试、停止。

参数的**语义与调参依据**见包内 README(上一级目录的 `README.md`);本文只回答
"哪条命令、怎么传参"。所有路径以仓库根(= colcon 工作区)`~/whale-nav` 为准。

## 1. 构建 / 环境

```bash
cd ~/whale-nav
colcon build --symlink-install --packages-select tagdocking   # 改代码/配置后重建
source install/setup.bash                                     # 每个新终端都要 source
```

`--symlink-install` 下 `config/docking.yaml` 与 launch 文件就是源码同一份(软链),
改完**重启节点**即生效,不用重编。

## 2. 启动入口

### 2.1 start_docking.sh 四种模式

```bash
cd ~/whale-nav/src/tagdocking

./scripts/start_docking.sh              # all: 后台 supervisor + 前台 Web (人工常用)
./scripts/start_docking.sh supervisor   # 只起按需启停 supervisor (满栈用时才拉起)
./scripts/start_docking.sh docking      # 前台起常驻满栈 (排查/标定用)
./scripts/start_docking.sh web          # 只起 Web 控制台 http://0.0.0.0:8090
```

- `docking` 模式**刻意不传参数**:实测最优的那组值已固化为 `docking.launch.py`
  的默认值(见 §3)。临时改值在模式后追加,余下参数原样转给 `ros2 launch`:

  ```bash
  ./scripts/start_docking.sh docking dual_dock_distance:=0.45
  ```

- Web 端口用环境变量覆盖:`TAGDOCKING_WEB_PORT=8090`
- `all` 模式下 supervisor 与 Web 任一退出/Ctrl-C 都会把另一个一并收掉;
  supervisor 退出时自己把满栈收干净

### 2.2 systemd 现状

`start_docking.sh` 注释里提到的
`whale-nav-tagdocking-supervisor.service` / `whale-nav-tagdocking-web.service`
**尚未安装**(检查 `/etc/systemd/system`),目前全靠脚本手动起。

### 2.3 日志位置

| 内容 | 位置 |
| --- | --- |
| 满栈三进程输出 (supervisor 重定向) | `/tmp/tagdocking_stack.log` |
| `ros2 launch` 自身日志 | `~/.ros/log/` |

## 3. launch 参数:三层覆盖

```bash
ros2 launch tagdocking docking.launch.py --show-args    # 看全部参数与默认值
ros2 launch tagdocking docking.launch.py 参数:=值 ...    # 命令行覆盖
```

### 3.1 优先级(从高到低)

| 层 | 位置 | 说明 |
| --- | --- | --- |
| ① 命令行 `key:=value` | — | 最高 |
| ② launch 内置默认值 | `launch/docking.launch.py` | 相机链路几个有实值(rtsp_url/camera_info_file/odom_topic/camera_downscale/tag_size);所有 `dual_*` 默认**空串 = 回落 yaml** |
| ③ YAML 权威值 | `src/tagdocking/config/docking.yaml` | 双码全部调参语义与默认值 |

### 3.2 通用参数(部分)

```text
rtsp_url                  RTSP 拉流地址 (默认 rtsp://127.0.0.1:8555/front)
camera_info_file          rtsp 内参 YAML (默认包内绝对路径 rtsp_camera_info.yaml)
family / tag_size         AprilTag 族 (36h11) 与墙码边长 (0.15)
dock_tag_id / camera_frame / base_frame
cmd_vel_topic / l1w_prefix / odom_topic   (odom 默认 /dog/odom)
camera_downscale          0 (关降采样 —— 远距离小 tag 检测下限, launch 注释有说明)
camera_backend            rtsp 解码后端 (gstreamer/ffmpeg)
camera_mount_x/y/z/yaw_deg/pitch_deg/roll_deg   rtsp 相机安装位姿静态 TF
wall_tag_size / pile_tag_id / pile_tag_size     双码码边长与桩码 ID
nodes                     只能 all | camera (拼错当场报错, 不静默退回)
charge_enable / charge_passive / charge_static_stand   充电收尾 (DOCKED 后泄力)
```

### 3.3 dual_* 批量参数 (双码)

38 个由 `DUAL_TUNING_ARGS` 批量声明的 launch 参数,**空串 = 保留 yaml 权威值**。
常用的: `observation_distance`、`observation_tolerance`、`forward_step`、
`reverse_step`、`lateral_step`、`yaw_step_deg`、`yaw_fine_step_deg`、
`straight_yaw_tol_deg`、`dock_distance`、`dock_tolerance`、
`straight_start_distance`、`align_tolerance_deg`、`align_hold_sec`、
`lateral_tolerance_m`、`exit_lateral_tolerance_m`、`prealign_tolerance_deg`、
`prealign_step_deg`、`prealign_max_steps`、`reverse_limit/count`、
`standoff_reverse_limit/count`、`observe_timeout_sec`、`settle_sec`……
完整清单见 `docking.launch.py --show-args`。

```bash
# 例: 命令行覆盖看门狗门槛与站位
./scripts/start_docking.sh docking dual_feedback_min_improvement:=0.02 dual_observation_distance:=1.6
```

## 4. 单独跑节点 (ros2 run, 参数用 -p)

```bash
ros2 run tagdocking rtsp_camera --ros-args \
    -p rtsp_url:="rtsp://127.0.0.1:8555/front" \
    -p camera_info_file:=$HOME/whale-nav/install/tagdocking/share/tagdocking/config/rtsp_camera_info.yaml

ros2 run tagdocking docking_node            # 控制器 (一般经 launch/supervisor)
ros2 run tagdocking docking_supervisor      # 按需启停守护
ros2 run tagdocking docking_web --port 8090 # Web 控制台
```

## 5. 标定脚本

```bash
# RTSP 相机内参 (无 ROS 依赖, OpenCV 直接拉流 + 棋盘格)
# 输出 YAML 喂给 rtsp_camera 的 camera_info_file → 落到 config/rtsp_camera_info.yaml
python3 scripts/calibrate_rtsp --url rtsp://127.0.0.1:8555/front \
    [--size 9x6] [--square 0.025] [--frames 20] [--backend gstreamer]
```

## 6. 台架 / 执行器测试脚本 (python3 直跑)

```bash
python3 scripts/mock_l1w_control    # 桌面无狗替身: 模拟 l1w_control 模式接口
                                    # (static_stand/lie_down/passive/stand_up),
                                    # 积分 /cmd_vel 发 /dog/odom, 跑通全闭环

python3 scripts/test_apriltag --image-topic /camera_sync/image_raw --size 0.15 \
    [--tag-id 0] [--known-distance 1.0]
    # 实时量码位姿。栈在跑时复用其 apriltag_node (不重复起)。
    # 距离 = 相机光学系 z; 相机泄力后仰时量的是斜距 —— 水平距离 = 距离 × cos(俯仰)

python3 scripts/test_jog_distance --distance 0.5 [--speed 0.2]    # 前进里程精度
python3 scripts/test_lateral_distance --distance 0.3              # 横移精度 (正=左)
python3 scripts/test_turn_angle --angle -90                       # 转向精度 (正=逆时针/左转)
```

## 7. 调试

### 7.1 话题 (节点名 `docking_node`)

```bash
ros2 topic echo /docking_node/state        # 状态机状态 (20Hz)
ros2 topic echo /docking_node/outcome      # 终态锁存 JSON:
                                           #   seq/op/state/ok/code/reason/elapsed_sec
ros2 topic echo /detections                # apriltag 检测流 (detection_topic 参数, 默认 /detections)
ros2 topic hz /camera_sync/image_raw       # rtsp 桥出图 (同源: /camera_sync/camera_info)
ros2 topic echo /cmd_vel --once            # 桥门控放行后才有非零速度
```

### 7.2 supervisor 服务 (节点名 `docking_supervisor`)

栈没起时**必须走 supervisor** —— 按需启停之下 `docking_node` 平时不存在,
直接调它的服务永远是"服务不可用":

```bash
ros2 topic echo /docking_supervisor/status                            # 栈 up/starting/ready + 详情
ros2 service call /docking_supervisor/dock std_srvs/srv/Trigger
ros2 service call /docking_supervisor/undock std_srvs/srv/Trigger
ros2 service call /docking_supervisor/cancel std_srvs/srv/Trigger
ros2 service call /docking_supervisor/stack_down std_srvs/srv/Trigger  # 手动收栈
```

supervisor 内部转发的节点级服务(仅栈已在跑时可直调):
`/docking_node/start_docking`、`/docking_node/start_undock`、`/docking_node/cancel_docking`。

### 7.3 Web 控制台接口

```bash
curl -X POST http://localhost:8090/api/dock      # 停泊 (同 /api/undock /api/cancel)
curl http://localhost:8090/api/video > /dev/null # 视频流 (multipart; 有观看者时 supervisor 拉起相机链路)
# 状态推送: WebSocket /ws
```

### 7.4 通用排障

```bash
ros2 node list                # rtsp_camera / apriltag_node / docking_node 是否齐
ros2 param get /docking_node dual.dock_distance    # 核对运行时参数实际值
ros2 param dump /docking_node > /tmp/dn.yaml       # 全量导出
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame   # 相机外参
ros2 run tf2_tools view_frames                   # TF 树; 多只 apriltag_node 会
                                                 # 互相发布同名 TF 打架 (launch 已 pkill 清场)
tail -f /tmp/tagdocking_stack.log             # 实时日志; `dual stage=` / `optical_wall_z=` /
                                              # `dual action COMPLETE` 行是主线索
```

注意:`ros2 param set` 热改对多数值有效(参数逐次读取),但**跳过启动校验**
(如 `dock_distance < steering_stop < straight_start` 的排序检查)。改关键参数
建议重启节点。

## 8. 测试

```bash
cd ~/whale-nav/src/tagdocking
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest test/        # 包内全部测试 (anyio 插件会炸, 必须禁用)
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest test/ -k watchdog

cd ~/whale-nav && python -m pytest tests/    # 仓库级契约测试 (无 ROS 依赖)
```

## 9. 停止

- **Ctrl-C**:`start_docking.sh` 前台模式把 supervisor + Web 一起收,
  supervisor 退出时自己收干净满栈
- supervisor 托管时:Web「取消/停止」按钮、`/docking_supervisor/cancel`
  (停当前动作)、`/docking_supervisor/stack_down` (收栈)
- 全家桶清扫 (仓库根, 不碰无关 ROS 进程):

  ```bash
  ~/whale-nav/scripts/cleanup_whale_nav.sh --dry-run   # 预览
  ~/whale-nav/scripts/cleanup_whale_nav.sh             # 执行
  ```
