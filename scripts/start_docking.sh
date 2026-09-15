#!/usr/bin/env bash
# tagdocking 启动器 —— 停泊栈 + Web 控制台。
#
#   ./start_docking.sh             # supervisor + Web (人工使用, 新的常驻组合)
#   ./start_docking.sh supervisor  # 只起按需启停 supervisor
#                                  #   (whale-nav-tagdocking-supervisor.service)
#   ./start_docking.sh docking     # 只起停泊栈 (supervisor 内部调用, 或排查时
#                                  #   手工起一个常驻满栈)
#   ./start_docking.sh web         # 只起 Web   (whale-nav-tagdocking-web.service)
#
# ── 语义变化: all 不再前台起满栈 ────────────────────────────────────
# 停泊栈三个进程 (rtsp_camera/apriltag_node/docking_node) 常驻很吃 CPU, 而停泊是
# 个偶发动作 —— 空转的开销结构上约等于停泊时的开销 (rtsp_camera 的 max_fps 限的是
# 发布不是解码; apriltag 的 decimate 被刻意关掉)。所以现在的常驻组合是
# **supervisor + web**, 满栈由 supervisor 在有人真要停泊时才拉起, 终态后延迟收掉。
# 要一个不会自动收掉的常驻满栈 (排查/标定用), 显式跑 `docking` 模式。
#
# ── 关于参数 ─────────────────────────────────────────────────────────
# 这里刻意"不传参数"。实测最优的那一组值已经固化成 docking.launch.py 里的
# DeclareLaunchArgument 默认值:
#
#   rtsp_url                rtsp://127.0.0.1:8555/front
#   camera_info_file        <包share>/config/rtsp_camera_info.yaml (绝对路径)
#   odom_topic              /odin1/odometry_highfreq
#   camera_downscale        0
#   dual_enable             true
#   dual_settle_sec         1.0
#   dual_dock_distance      0.47
#   camera_lateral_offset_m 0
#   tag_size                0.15
#
# 如果这里再抄一遍, 日后改了 launch 默认值就会被脚本里的旧值悄悄盖掉, 变成
# 两个真相来源、而且是难查的那一种。要临时改某个值, 在命令行追加即可:
#
#   ./start_docking.sh docking dual_dock_distance:=0.50
#
# camera_info_file 用的是包内绝对路径, 不是原实测命令里的相对路径
# config/rtsp_camera_info.yaml —— systemd 下工作目录不确定, 相对路径会失效。
# symlink-install 让这条绝对路径就是源码同一份文件, 重新标定后立即生效。
# 启动时 launch 会打印 `[docking.launch] rtsp 内参: <路径>`, 可据此核对。

set -euo pipefail

MODE="${1:-all}"
shift 2>/dev/null || true          # 余下参数原样转给 ros2 launch

ROS_SETUP="/opt/ros/humble/setup.bash"
WS_SETUP="/home/nvidia/whale-nav/install/setup.bash"
WEB_PORT="${TAGDOCKING_WEB_PORT:-8090}"

for f in "$ROS_SETUP" "$WS_SETUP"; do
  if [[ ! -f "$f" ]]; then
    echo "找不到 $f —— 工作区没构建? 先跑:" >&2
    echo "  cd /home/nvidia/whale-nav && colcon build --symlink-install" >&2
    exit 1
  fi
done

# shellcheck disable=SC1090
# ROS 的 setup.bash 不是 `set -u` 干净的 (line 8 会引用未设置的
# AMENT_TRACE_SETUP_FILES), 开着 -u 会让 source 直接把脚本打死。
# 只在 source 期间关掉, 之后立刻恢复 —— 我们自己的代码仍受 -u 保护。
set +u
source "$ROS_SETUP"
# shellcheck disable=SC1090
source "$WS_SETUP"
set -u

start_supervisor() {
  echo "[start_docking] 启动按需启停 supervisor (满栈用时才起)"
  ros2 run tagdocking docking_supervisor
}

start_web() {
  echo "[start_docking] 启动 Web 控制台: http://0.0.0.0:${WEB_PORT}"
  ros2 run tagdocking docking_web --port "${WEB_PORT}"
}

case "$MODE" in
  docking)
    exec ros2 launch tagdocking docking.launch.py "$@"
    ;;
  supervisor)
    exec ros2 run tagdocking docking_supervisor "$@"
    ;;
  web)
    exec ros2 run tagdocking docking_web --port "${WEB_PORT}"
    ;;
  all)
    # 人工使用路径: 后台起 supervisor, 前台起 Web。任一退出或 Ctrl-C 都把另一个
    # 一并收掉。注意这里收的是 supervisor —— 满栈归它管, 它退出时会自己把栈
    # 收干净 (stack_supervisor.shutdown)。
    start_supervisor &
    DOCK_PID=$!
    cleanup() {
      trap - INT TERM EXIT
      echo
      echo "[start_docking] 收尾, 停止子进程…"
      kill -INT "$DOCK_PID" 2>/dev/null || true
      kill -INT "${WEB_PID:-}" 2>/dev/null || true
      wait "$DOCK_PID" 2>/dev/null || true
      wait "${WEB_PID:-}" 2>/dev/null || true
    }
    trap cleanup INT TERM EXIT

    start_web &
    WEB_PID=$!
    # 谁先挂就一起收场, 不留半截栈。
    wait -n "$DOCK_PID" "$WEB_PID"
    ;;
  *)
    echo "用法: $0 [all|supervisor|docking|web] [额外的 ros2 launch 参数…]" >&2
    exit 2
    ;;
esac
