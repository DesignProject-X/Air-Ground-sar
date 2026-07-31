#!/usr/bin/env python3
"""
launcher.py
-----------
Drop-in replacement for `python3 -m http.server 8080` when serving
dashboard.html: same directory, same port, same page - plus three endpoints
so the dashboard's start buttons can actually launch things.
在给 dashboard.html 做静态服务时,用来替代 `python3 -m http.server 8080`:
目录、端口、页面都不变,只是多了三个接口,好让面板上的启动按钮真的能把
东西跑起来。

The plain http.server cannot do this at all - it only ever serves files, so
a button pressed in the page would have nothing to run `ros2 launch` with.
Going through rosbridge instead is not an option either: rosbridge is itself
started by base_station_real.launch.py, so it cannot be what starts it (and
it has no way to spawn processes regardless).
标准的 http.server 完全做不到这件事——它只会发文件,页面上的按钮按下去,
没有任何东西能去执行 `ros2 launch`。改走 rosbridge 也不行:rosbridge 本身
就是 base_station_real.launch.py 启动的,不可能由它来启动自己(何况它也
没有派生进程的能力)。

Only the three fixed entries in TARGETS below can ever be started - the
request path selects one by name and nothing from the request is passed
through to the shell.
只有下面 TARGETS 里固定的三条能被启动——请求路径只是按名字选其中一条,
请求里的任何内容都不会传进 shell。

Usage / 用法:
    cd ros2_ws/src/sar_bringup/web && python3 launcher.py
    # then open http://localhost:8080
"""

import json
import os
import shlex
import signal
import subprocess
import sys
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer

PORT = 8080
WEB_DIR = os.path.dirname(os.path.abspath(__file__))
# web -> sar_bringup -> src -> ros2_ws, then its install space. Note there is
# also a stray install/ at the repo root one level further up that only holds
# a few packages - sourcing that one instead would silently miss most of them.
# web -> sar_bringup -> src -> ros2_ws,再进它的 install。注意再往上一层的
# 仓库根目录下还有一个残留的 install/,里面只有少数几个包——错源成那个的话,
# 大部分包会悄无声息地缺失。
WS_SETUP = os.path.abspath(os.path.join(WEB_DIR, '..', '..', '..', 'install', 'setup.bash'))
ROBOT_HOST = 'tb@192.168.1.55'
LOG_DIR = os.path.join(WEB_DIR, '.launcher_logs')


def _local(launch_cmd: str) -> str:
    """A base-station launch, run under a shell that has the workspace sourced."""
    return f'source /opt/ros/humble/setup.bash && source {shlex.quote(WS_SETUP)} && {launch_cmd}'


# Start the robot before the base station: the base station's clock_sync_client
# is a one-shot that needs the robot's own clock_sync_node already up to reach
# it. (Doing it the other way round has its own hazard - a clock correction
# landing after the robot's nav2 has initialised leaves AMCL's TF buffer
# straddling the jump, which shows up as "Tf has two or more unconnected
# trees" and no map->base_link at all.) The dashboard lays the buttons out in
# this order for that reason; nothing here enforces it.
# 先启动小车再启动基站:基站的 clock_sync_client 是一次性的,需要小车自己的
# clock_sync_node 已经起来才能调到它。(反过来做也有坑——时钟纠正如果发生在
# 小车 nav2 初始化之后,AMCL 的 TF 缓冲区里就会横跨这次跳变,表现为
# "Tf has two or more unconnected trees"、map->base_link 完全建立不起来。)
# 面板上按钮就是按这个顺序排的;这里并不强制。
TARGETS = {
    'robot': {
        'label': 'Ground Robot',
        'cmd': ['ssh', '-o', 'BatchMode=yes', ROBOT_HOST,
                # -i, not -l. The robot sets ROS_DOMAIN_ID=3,
                # RMW_IMPLEMENTATION and CYCLONEDDS_URI in ~/.bashrc, which
                # opens with `case $- in *i*) ;; *) return;; esac` - so a
                # non-interactive shell leaves all three unset no matter how
                # it is invoked, and sourcing ~/.bashrc by hand hits the same
                # early return. The launch then comes up on domain 0 with the
                # default RMW while the base station is on domain 3 with
                # CycloneDDS static peers, and the two never discover each
                # other: the robot's own nodes start fine, yet every
                # cross-machine lookup from the base station fails.
                # Measured on the robot: `bash -lc` -> ROS_DOMAIN_ID=[],
                # `bash -ic` -> [3]. Typing the command over a manual ssh has
                # always worked precisely because that shell is interactive.
                # 用 -i 而不是 -l。小车把 ROS_DOMAIN_ID=3、RMW_IMPLEMENTATION、
                # CYCLONEDDS_URI 都设在 ~/.bashrc 里,而它开头就是
                # `case $- in *i*) ;; *) return;; esac`——所以不管怎么调用,
                # 非交互 shell 里这三个变量都是空的,手动 source ~/.bashrc 也
                # 一样会撞上这个提前 return。于是 launch 跑在 domain 0、用默认
                # RMW,基站却在 domain 3、用 CycloneDDS 静态 peer,两边永远发现
                # 不了对方:小车自己的节点都正常起来,但基站这边所有跨机查找
                # 全部失败。在小车上实测:`bash -lc` 得到 ROS_DOMAIN_ID=[],
                # `bash -ic` 得到 [3]。手动 ssh 进去敲命令之所以一直没问题,
                # 正是因为那个 shell 是交互式的。
                "bash -ic 'source /opt/ros/humble/setup.bash && "
                "source ~/tb_ws/install/setup.bash && "
                "ros2 launch ground_controller robot_bringup.launch.py'"],
        # Killing the local ssh is not enough: with no TTY allocated, the
        # signal never reaches the far side, so the robot's ros2 launch keeps
        # running as an orphan still holding its topics (seen before while
        # debugging - it had to be cleaned up by hand over a second ssh).
        # SIGINT rather than SIGTERM so ros2 launch runs its own shutdown and
        # brings its child nodes down cleanly.
        # 光杀本地的 ssh 不够:没有分配 TTY 时信号根本传不到对端,小车上的
        # ros2 launch 会变成孤儿继续跑、继续占着话题(之前排查时遇到过,
        # 只能另开一个 ssh 手动清理)。用 SIGINT 而不是 SIGTERM,是为了让
        # ros2 launch 走它自己的关闭流程,干净地带下子节点。
        'stop_cmd': ['ssh', '-o', 'BatchMode=yes', ROBOT_HOST,
                     "pkill -INT -f 'ros2 launch ground_controller robot_bringup'"],
    },
    'base': {
        'label': 'Base Station',
        'cmd': ['bash', '-c',
                _local('ros2 launch sar_bringup base_station_real.launch.py')],
    },
    'drone': {
        'label': 'UAV',
        'cmd': ['bash', '-c',
                _local('ros2 launch crazyflie_ros2_multiranger_bringup '
                       'wall_follower_mapper_real.launch.py')],
    },
    # No -d: rviz2 then restores whatever was last open, which is what typing
    # `rviz2` by hand already does. The configs under the source tree are not
    # a better default - real_mapping.rviz shows the UAV's own
    # /crazyflie_real/map rather than the map the ground robot navigates on,
    # and config_with_map.rviz still points at the vendored /cf231/* names.
    # Pass one explicitly here if a fixed layout is ever wanted.
    # 不带 -d:rviz2 会恢复上次打开的那套配置,跟手敲 `rviz2` 的效果一样。
    # 源码树里那几个配置并不是更好的默认值——real_mapping.rviz 显示的是无人机
    # 自己的 /crazyflie_real/map,不是地面小车导航用的那张图;
    # config_with_map.rviz 里还是 vendored 的 /cf231/* 话题名。以后要固定某套
    # 布局的话,在这里显式指定即可。
    'rviz': {
        'label': 'RViz2',
        'cmd': ['bash', '-c', _local('rviz2')],
    },
}

procs = {}


def start(name):
    proc = procs.get(name)
    if proc and proc.poll() is None:
        return False, 'already running'
    os.makedirs(LOG_DIR, exist_ok=True)
    log_path = os.path.join(LOG_DIR, f'{name}.log')
    log = open(log_path, 'wb')
    # start_new_session so the launch and everything it spawns share one
    # process group - killing the shell alone would leave the ros2 launch
    # tree orphaned and still holding its topics.
    # start_new_session 让 launch 及其派生的所有进程同属一个进程组——只杀
    # 外层 shell 的话,ros2 launch 那一整棵进程树会变成孤儿继续占着话题。
    procs[name] = subprocess.Popen(
        TARGETS[name]['cmd'], stdout=log, stderr=subprocess.STDOUT,
        stdin=subprocess.DEVNULL, start_new_session=True, cwd=WEB_DIR)
    return True, f'started (log: {log_path})'


def stop(name):
    proc = procs.get(name)
    if not proc or proc.poll() is not None:
        return False, 'not running'
    remote = TARGETS[name].get('stop_cmd')
    if remote:
        # Shut the far side down first, then drop the ssh that was carrying
        # it - the other order leaves nothing to send the signal through.
        # 先让对端关掉,再断开承载它的那条 ssh——反过来的话,信号就没有通道
        # 可以送过去了。
        try:
            subprocess.run(remote, timeout=15, stdin=subprocess.DEVNULL,
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except subprocess.TimeoutExpired:
            return False, 'remote stop timed out (robot unreachable?)'
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
    except ProcessLookupError:
        pass
    return True, 'stopping'


def status():
    return {n: ('running' if (p := procs.get(n)) and p.poll() is None else 'stopped')
            for n in TARGETS}


class Handler(SimpleHTTPRequestHandler):

    def __init__(self, *a, **kw):
        super().__init__(*a, directory=WEB_DIR, **kw)

    def _json(self, payload, code=200):
        body = json.dumps(payload).encode()
        self.send_response(code)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path == '/api/status':
            self._json(status())
            return
        super().do_GET()

    def do_POST(self):
        parts = self.path.strip('/').split('/')
        if len(parts) == 3 and parts[0] == 'api' and parts[1] in ('launch', 'stop'):
            name = parts[2]
            if name not in TARGETS:
                self._json({'ok': False, 'message': 'unknown target'}, 404)
                return
            ok, message = (start if parts[1] == 'launch' else stop)(name)
            self._json({'ok': ok, 'message': message, 'status': status()})
            return
        self._json({'ok': False, 'message': 'not found'}, 404)

    def log_message(self, fmt, *args):
        if not self.path.startswith('/api/status'):  # status is polled, too noisy
            sys.stderr.write(f'{self.address_string()} - {fmt % args}\n')


def main():
    port = int(sys.argv[1]) if len(sys.argv) > 1 else PORT
    os.chdir(WEB_DIR)
    try:
        server = ThreadingHTTPServer(('0.0.0.0', port), Handler)
    except OSError as e:
        # Almost always a leftover `python3 -m http.server` still holding the
        # port. Worth naming, because that server answers requests perfectly
        # well for the page itself while returning 404/501 for every /api
        # call - so the buttons fail in a way that looks like a bug here.
        # 基本都是之前的 `python3 -m http.server` 还占着端口。值得点明,因为
        # 那个服务器发页面本身毫无问题,只是对所有 /api 请求返回 404/501——
        # 于是按钮失效的样子会很像是这边的 bug。
        sys.exit(f'Cannot bind port {port}: {e}\n'
                 f'Is an old "python3 -m http.server {port}" still running? '
                 f'Stop it first, or pass another port: python3 launcher.py 8081')
    print(f'Serving {WEB_DIR} on http://localhost:{port}')
    print(f'Launch targets: {", ".join(TARGETS)}   logs: {LOG_DIR}')
    # The base station / UAV / rviz targets inherit this process's own
    # environment, so a launcher started from a shell that never read
    # ~/.bashrc would put them on domain 0 while the robot is on domain 3 -
    # everything would start cleanly and simply never see the other machine.
    # That failure is silent and looks nothing like a config problem, so say
    # it up front instead.
    # 基站/无人机/rviz 这几个目标继承的是本进程自己的环境,所以如果 launcher
    # 是从一个没读过 ~/.bashrc 的 shell 启动的,它们会跑在 domain 0,而小车在
    # domain 3——所有东西都能正常起来,只是永远看不见对方。这种失败是无声的,
    # 表现上也完全不像配置问题,所以干脆提前说出来。
    missing = [v for v in ('ROS_DOMAIN_ID', 'RMW_IMPLEMENTATION') if not os.environ.get(v)]
    if missing:
        print(f'WARNING: {", ".join(missing)} not set in this shell - locally '
              f'launched nodes may not see the ground robot. Start me from a '
              f'shell that has sourced ~/.bashrc.')
    else:
        print(f'ROS_DOMAIN_ID={os.environ["ROS_DOMAIN_ID"]}  '
              f'RMW={os.environ["RMW_IMPLEMENTATION"]}')
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print('\nShutting down; stopping launched processes...')
        for name in list(procs):
            stop(name)


if __name__ == '__main__':
    main()
