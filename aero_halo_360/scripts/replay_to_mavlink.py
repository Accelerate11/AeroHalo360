#!/usr/bin/env python3
import argparse
import os
import pathlib
import signal
import subprocess
import sys
import time


def parse_bool(value):
    normalized = value.strip().lower()
    if normalized in ('1', 'true', 'yes', 'on'):
        return True
    if normalized in ('0', 'false', 'no', 'off'):
        return False
    raise argparse.ArgumentTypeError(f'无法解析布尔值: {value}')


def validate_bag_path(raw_path):
    path = pathlib.Path(raw_path).expanduser().resolve()
    if not path.exists():
        raise ValueError(f'rosbag 路径不存在: {path}')
    if path.is_dir() and not (path / 'metadata.yaml').is_file():
        raise ValueError(f'rosbag 目录缺少 metadata.yaml: {path}')
    if path.is_file() and path.suffix not in ('.db3', '.mcap'):
        raise ValueError(f'rosbag 文件必须是 .db3 或 .mcap: {path}')
    return str(path)


def node_is_ready(node_name):
    result = subprocess.run(
        ['ros2', 'node', 'list'],
        check=False,
        capture_output=True,
        text=True,
        timeout=3,
    )
    return result.returncode == 0 and node_name in result.stdout.splitlines()


def stop_process_group(process):
    if process is None or process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGTERM)
    try:
        process.wait(timeout=5)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        process.wait(timeout=2)


def main():
    parser = argparse.ArgumentParser(
        description='等待处理节点 ready 后播放 rosbag，并传播播放退出状态')
    parser.add_argument('bag_path')
    parser.add_argument('--no-launch', action='store_true')
    parser.add_argument('--base-config', default='')
    parser.add_argument('--profile', default='mission_planner_demo')
    parser.add_argument('--installation-config', default='')
    parser.add_argument('--connection', default='udpout:127.0.0.1:14550')
    parser.add_argument('--wait-node', default='/cloud_processor_node')
    parser.add_argument('--ready-timeout', type=float, default=15.0)
    parser.add_argument('--start-delay', type=float, default=1.0)
    parser.add_argument('--clock', type=parse_bool, default=True)
    parser.add_argument('--loop', type=parse_bool, default=False)
    args = parser.parse_args()

    try:
        bag_path = validate_bag_path(args.bag_path)
    except ValueError as exc:
        parser.error(str(exc))
    if args.ready_timeout <= 0.0 or args.start_delay < 0.0:
        parser.error('ready-timeout 必须大于 0，start-delay 不能为负数')

    launch_process = None
    if not args.no_launch:
        launch_command = [
            'ros2', 'launch', 'aero_halo_360', 'aero_halo_360.launch.py',
            'start_mavlink:=true',
            'timestamp_mode:=replay',
            f'profile:={args.profile}',
            f'mavlink_connection:={args.connection}',
        ]
        if args.base_config:
            launch_command.append(f'base_config:={args.base_config}')
        if args.installation_config:
            launch_command.append(
                f'installation_config:={args.installation_config}')
        launch_process = subprocess.Popen(launch_command, start_new_session=True)

    try:
        deadline = time.monotonic() + args.ready_timeout
        while time.monotonic() < deadline:
            if launch_process is not None and launch_process.poll() is not None:
                print('错误：AeroHalo360 launch 在 ready 前退出', file=sys.stderr)
                return launch_process.returncode or 1
            if node_is_ready(args.wait_node):
                break
            time.sleep(0.2)
        else:
            print(
                f'错误：等待节点 {args.wait_node} 超时 '
                f'({args.ready_timeout:.1f} s)',
                file=sys.stderr,
            )
            return 3

        time.sleep(args.start_delay)
        bag_command = ['ros2', 'bag', 'play', bag_path]
        if args.clock:
            bag_command.append('--clock')
        if args.loop:
            bag_command.append('--loop')
        print('回放命令:', ' '.join(bag_command), flush=True)
        return subprocess.call(bag_command)
    finally:
        stop_process_group(launch_process)


if __name__ == '__main__':
    raise SystemExit(main())
