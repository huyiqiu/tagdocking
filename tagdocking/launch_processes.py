"""按进程树收掉一棵 ros2 launch —— 包括不是本进程起的那些。

从 `robot_web_console/robot_web_console/navigation_processes.py` 移植而来。
那份代码在导航栈上已经跑了很久, 逻辑原样保留, 只改了命名和报错文案。

**为什么是移植而不是 import**: 那是 web console 包。让 tagdocking 依赖它是反向
依赖 —— 停泊栈不应该为了收几个进程就把一个网页控制台拖进依赖图。这里一共 70 行、
纯标准库, 抄过来比耦合过去划算。

**为什么按进程树而不是按名字 pkill**: `docking.launch.py` 里那段
`pkill -9 -f apriltag_node` 只认名字, 收不掉 `rtsp_camera` / `docking_node`,
而且 `-9` 会绕开 docking_node 自己的停车逻辑。按树收是按 ppid 一层层扒出来的,
camera 模式和满栈模式、自己起的和手工起的, 一次全收。
"""
import os
from pathlib import Path
import shlex
import signal
import time


# 这棵栈的规范匹配串。supervisor 起栈、收栈、接管、清扫游兵都用它, 只有一处定义。
# 注意它**不含** `nodes:=…` —— matches_launch 只比对 `launch <包> <文件>` 三元组,
# 所以 camera 模式和满栈模式会被同一条串一起认出来, 收场时不用关心当初以哪种
# 模式起的; 也认得 systemd 单元和 README 里那条不带任何参数的裸调用。
DOCKING_LAUNCH_CMD = 'ros2 launch tagdocking docking.launch.py'


def matches_launch(argv, command):
    """argv 是否是 `command` 描述的那条 ros2 launch。

    只比对 `launch <包> <文件>` 这个三元组, 后面跟的参数 (如 nodes:=camera)
    不参与匹配 —— 所以 camera 模式和满栈模式会被同一条 command 一起认出来,
    这正是我们要的: 收场时不用关心对方当初是以哪种模式起的。
    """
    expected = shlex.split(command)
    try:
        index = expected.index('launch')
        target = expected[index:index + 3]
        if len(target) != 3:
            return False
        return any(Path(arg).name == 'ros2' and argv[i + 1:i + 4] == target
                   for i, arg in enumerate(argv))
    except ValueError:
        return False


def process_snapshot(proc=Path('/proc')):
    """扫一遍 /proc, 返回 {pid: (状态, ppid, 启动时刻, argv)}。"""
    result = {}
    for path in proc.iterdir():
        if not path.name.isdigit():
            continue
        try:
            fields = (path / 'stat').read_text().rsplit(')', 1)[1].split()
            argv = (path / 'cmdline').read_bytes().decode(errors='replace').strip('\0').split('\0')
            # 状态、父 pid、启动时刻三者一起存: 状态用来跳过僵尸, 启动时刻用来
            # 防 pid 复用 —— 信号发出到进程退出之间, pid 可能已经被别人占了。
            result[int(path.name)] = (fields[0], int(fields[1]), fields[19], argv)
        except (OSError, ValueError, IndexError):
            continue
    return result


def launch_tree(snapshot, command, owned_pid=None):
    """从 snapshot 里扒出这棵 launch 的全部后代, 返回 {pid: 启动时刻}。

    owned_pid 是"我自己起的那个"; 传了就一并算作树根。没传也能工作 —— 靠 argv
    匹配认出别人起的那棵, 这就是接管 (adopt) 能成立的原因。
    """
    roots = {pid for pid, info in snapshot.items() if matches_launch(info[3], command)}
    if owned_pid in snapshot:
        roots.add(owned_pid)
    targets = set(roots)
    while True:
        children = {pid for pid, info in snapshot.items() if info[1] in targets}
        if children <= targets:
            break
        targets |= children
    targets.discard(os.getpid())
    return {pid: snapshot[pid][2] for pid in targets}


def _escalate_kill(targets, grace_sec):
    """对 {pid: 启动时刻} 逐级 SIGINT → SIGTERM → SIGKILL, 直到全退或放弃。

    比对启动时刻而不只是 pid: 信号发出到进程真正消失之间有个窗口, 这期间 pid
    可能已经被别的进程占了。只按 pid 判断"还活着", 就有机会朝一个无辜的新进程
    升级发 SIGKILL。
    """
    def alive():
        current = process_snapshot()
        return {pid for pid, start in targets.items()
                if pid in current and current[pid][2] == start
                and current[pid][0] not in ('Z', 'X')}

    for sig, grace in ((signal.SIGINT, grace_sec),
                       (signal.SIGTERM, 3.0),
                       (signal.SIGKILL, 1.0)):
        for pid in alive():
            try:
                os.kill(pid, sig)
            except ProcessLookupError:
                pass
        deadline = time.monotonic() + grace
        while time.monotonic() < deadline:
            if not alive():
                return set()
            time.sleep(0.1)
    return alive()


def stop_launch_tree(command, owned_pid=None, grace_sec=8.0):
    """SIGINT → SIGTERM → SIGKILL 逐级收掉整棵树, 连同被 init 收养的游兵。

    grace_sec 默认 8 秒。这个数不是拍的, 是三段时间叠起来的:
    docking_node 的信号处理器 (docking_node.py:2504-2527) 收到 SIGINT/SIGTERM 后
    **阻塞整整 1 秒**, 以 10ms 间隔把零速度刷 100 遍; 然后 rclpy.shutdown() 走
    析构; 然后 `ros2 launch` 还要等它所有子节点都退干净才自己退。既有单元给这
    整条链留的是 `TimeoutStopSec=15`, 8 秒是其中留给第一级(体面退出)的份额。
    绝不能直接上 SIGKILL —— 那会绕过刷零速度那一整套, 最后一条非零速度就一直
    挂着, 只能等底盘看门狗兜底。

    幂等: 没东西可收就直接返回, 所以每次起栈前都可以先无条件调一遍来清场。
    """
    snapshot = process_snapshot()
    targets = launch_tree(snapshot, command, owned_pid)
    # 树和游兵一起收, 一次升级走完 —— 分两轮的话第二轮又要重新等 8 秒。
    targets.update(stray_stack_processes(snapshot))
    if not targets:
        return
    remaining = _escalate_kill(targets, grace_sec)
    if remaining:
        raise RuntimeError(f'停泊栈进程未退出，PID: {sorted(remaining)}')


def tree_is_running(command, owned_pid=None) -> bool:
    """这棵 launch 现在是不是活着 —— 供接管判断和状态上报用。"""
    return bool(launch_tree(process_snapshot(), command, owned_pid))


# 游兵清扫要认的可执行文件路径尾巴。**必须带目录前缀**, 这是关键:
# 裸写 'rtsp_camera' 会命中命令行里的 `camera_info_file:=…/rtsp_camera_info.yaml`
# (README §2.3.1 和 start_docking.sh 头部注释都这么写), 于是把 `ros2 launch`
# 进程本身当成游兵 SIGKILL 掉。
#
# 名单是白名单而不是 'lib/tagdocking/' 通配: 同目录下还住着 docking_web 和
# docking_supervisor —— web 是要常驻的, supervisor 就是执行清扫的人自己。
_STRAY_SUFFIXES = (
    'lib/tagdocking/rtsp_camera',
    'lib/tagdocking/docking_node',
    'apriltag_ros/apriltag_node',
)


def stray_stack_processes(snapshot=None):
    """找出不挂在任何 launch 树下的停泊栈残兵, 返回 {pid: 启动时刻}。

    为什么按树收之外还要这一道: `launch_tree` 只能扒出**某个匹配的 launch 根**
    的后代。如果 launch 根先死了 (被 SIGKILL、或 OOM), 它的孩子会被 init 收养,
    ppid 变成 1 —— 此时树里一个都找不到, 但那个 rtsp_camera 还在发
    `/camera_sync/*` 和第二份静态 TF。下次起栈就是两个实例对着同一个 TF 帧
    双重广播, 量测在两组矛盾值之间跳 —— 正是 launch 里那段 pkill 想防的事。
    """
    snapshot = snapshot if snapshot is not None else process_snapshot()
    launched = set(launch_tree(snapshot, DOCKING_LAUNCH_CMD))
    out = {}
    for pid, (_state, _ppid, start, argv) in snapshot.items():
        if pid in launched or pid == os.getpid() or not argv:
            continue
        # 只看前两个 argv: Python 节点是 `python3 <路径> --ros-args …` (路径在
        # argv[1]), C++ 的 apriltag_node 是 `<路径> --ros-args …` (在 argv[0])。
        # 再往后就是 `--ros-args` 那一堆, 扫进去就会碰上 `:=` 参数里的路径。
        if any(a.endswith(_STRAY_SUFFIXES) for a in argv[:2]):
            out[pid] = start
    return out
