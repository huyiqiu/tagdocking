# tagdocking 对接文档（上层应用）

上层应用对接接口：启停服务、触发停泊/泊出、查状态、查日志。

## 服务

| 服务 | 作用 |
| --- | --- |
| `whale-nav-tagdocking-supervisor.service` | 按需拉起/收掉停泊栈（常驻） |
| `whale-nav-tagdocking-web.service` | HTTP API + Web 控制台，端口 8090（常驻） |

栈由 supervisor 自动管理，上层无需干预。

```bash
sudo systemctl enable whale-nav-tagdocking-supervisor.service   # 设置开机启动
sudo systemctl enable whale-nav-tagdocking-web.service

sudo systemctl start whale-nav-tagdocking-supervisor.service
sudo systemctl start whale-nav-tagdocking-web.service
```

## 触发（HTTP）

```bash
curl -X POST http://<IP>:8090/api/dock      # 停泊
curl -X POST http://<IP>:8090/api/undock    # 泊出
curl -X POST http://<IP>:8090/api/cancel    # 取消当前动作
```

返回 `{"success": true/false, "message": "..."}`。`success:true` 仅表示已受理，
成败看下方 `outcome`。

## 查状态（HTTP）

```bash
curl http://<IP>:8090/api/status     # 轮询
# ws://<IP>:8090/ws                  # 或订阅 WebSocket，同结构 JSON 实时推送
```

| 字段 | 含义 |
| --- | --- |
| `online` | supervisor 是否在线 |
| `stack` | `down`(未起,正常) / `starting` / `ready` / `up` |
| `state` | `idle` / `search_tag` / `approach` / `docked` / `undocking` / `undocked` |
| `outcome` | 上一轮终态：`{seq, op, state, ok, code, reason, elapsed_sec}` |

**成败判定**：`state` 到 `docked`/`undocked` 且 `outcome.ok==true` 为成功；
`ok==false` 读 `code`/`reason`。`seq` 用于识别是否本轮新结果。

## 查日志

```bash
journalctl -u whale-nav-tagdocking-supervisor.service -f   # 服务日志
tail -f /tmp/tagdocking_stack.log                          # 停泊栈主线索
```

## ROS 接口（可选，需对齐环境）

跑在 `ROS_DOMAIN_ID=99` + `rmw_zenoh_cpp`，对接前须 `source` 环境：

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/whale-nav/install/setup.bash
source /home/nvidia/whale-nav/scripts/whale_nav_env.sh

ros2 service call /docking_supervisor/dock   std_srvs/srv/Trigger   # 停泊
ros2 service call /docking_supervisor/undock   std_srvs/srv/Trigger # 泊出
ros2 service call /docking_supervisor/cancel   std_srvs/srv/Trigger # 取消
ros2 topic echo /docking_supervisor/status                          # 状态 JSON
```
