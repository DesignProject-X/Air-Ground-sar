#!/usr/bin/env python3
"""
Robot clock sync checker
机器人时钟同步检查工具

This robot has no battery-backed RTC, so its clock resets to a bogus
default (e.g. 2009-01-01, or whatever it happened to be left at) on every
power cycle, and this LAN has no reachable NTP server to fix it
automatically. A clock that's off by even a few seconds from this base
station breaks every cross-machine TF lookup (tf2 refuses to extrapolate
across a gap that large) - this exact symptom cost real debugging time
before the root cause (a ~79 day clock skew) was found.
这台机器人没有带电池的硬件时钟(RTC),每次断电重启后时间都会跳回一个
无意义的默认值(比如2009-01-01,或者上次断电时停留的时间),而这个局域网
里又没有能连到的NTP服务器自动校正。只要它的时钟和这台电脑差哪怕几秒,
所有跨机器的TF查询都会失败(tf2会拒绝跨过这么大的时间缺口去外推)——
这个现象曾经在找到真正原因(约79天的时钟偏差)之前,浪费了不少排查时间。

Run this once after every robot reboot, before starting anything else.
每次机器人重启后,在启动其它任何东西之前,先跑一次这个。

Usage / 用法:
    python3 sync_robot_clock.py --host 192.168.1.55 --user tb --password 1234
    (or set ROBOT_SSH_PASSWORD env var instead of --password, to avoid
    leaving the password in shell history)
    (也可以用环境变量 ROBOT_SSH_PASSWORD 代替 --password,避免密码留在
    shell历史记录里)
"""

import argparse
import os
import subprocess
import sys
from datetime import datetime, timezone

import paramiko

# Clock skew above this is treated as "broken, needs correcting" - a few
# seconds of slack is given for normal SSH round-trip/command overhead, not
# because small skew is considered fine for TF (ideally this would be near
# zero), just to avoid re-setting the clock on every run over noise.
# 时钟偏差超过这个值就判定为"坏了,需要校正"——留几秒余量是为了容纳SSH
# 往返/命令执行本身的耗时,不是说这么大的误差对TF来说没问题(理想情况下
# 应该趋近于0),只是为了不让每次运行都因为一点点噪声就重新设置时钟。
SKEW_THRESHOLD_SECONDS = 5.0


def get_robot_time(client) -> datetime:
    stdin, stdout, stderr = client.exec_command("date -u '+%Y-%m-%d %H:%M:%S'")
    out = stdout.read().decode().strip()
    return datetime.strptime(out, '%Y-%m-%d %H:%M:%S').replace(tzinfo=timezone.utc)


def set_robot_time(client, password: str, when: datetime) -> None:
    stamp = when.strftime('%Y-%m-%d %H:%M:%S')
    cmd = f"echo '{password}' | sudo -S date -u -s '{stamp}'"
    stdin, stdout, stderr = client.exec_command(cmd)
    stdout.channel.recv_exit_status()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', required=True, help='Robot IP or hostname')
    parser.add_argument('--user', default='tb', help='SSH username (default: tb)')
    parser.add_argument(
        '--password',
        default=os.environ.get('ROBOT_SSH_PASSWORD'),
        help='SSH/sudo password (default: $ROBOT_SSH_PASSWORD env var)')
    args = parser.parse_args()

    if not args.password:
        print('No password given (use --password or $ROBOT_SSH_PASSWORD).', file=sys.stderr)
        sys.exit(1)

    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(args.host, username=args.user, password=args.password, timeout=10)

    local_now = datetime.now(timezone.utc)
    robot_now = get_robot_time(client)
    skew = (local_now - robot_now).total_seconds()

    print(f'This machine (UTC): {local_now:%Y-%m-%d %H:%M:%S}')
    print(f'Robot (UTC):        {robot_now:%Y-%m-%d %H:%M:%S}')
    print(f'Skew: {skew:.1f}s')

    if abs(skew) <= SKEW_THRESHOLD_SECONDS:
        print('Clocks already in sync - nothing to do. / 时钟已经同步,不需要处理')
        client.close()
        return

    print(f'Skew exceeds {SKEW_THRESHOLD_SECONDS}s threshold - correcting robot clock...')
    correction_time = datetime.now(timezone.utc)
    set_robot_time(client, args.password, correction_time)

    robot_now_after = get_robot_time(client)
    new_skew = (datetime.now(timezone.utc) - robot_now_after).total_seconds()
    print(f'Robot clock set. New skew: {new_skew:.1f}s')

    client.close()

    if abs(new_skew) > SKEW_THRESHOLD_SECONDS:
        print('WARNING: skew still large after correction - check manually.', file=sys.stderr)
        sys.exit(1)


if __name__ == '__main__':
    main()
