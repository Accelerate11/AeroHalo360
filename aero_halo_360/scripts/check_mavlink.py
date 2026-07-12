#!/usr/bin/env python3
import argparse
import os
import sys
import time

os.environ['MAVLINK20'] = '1'

try:
    from pymavlink import mavutil
except ImportError:
    mavutil = None


def is_target_autopilot(message, target_system, target_component):
    system_id = int(message.get_srcSystem())
    component_id = int(message.get_srcComponent())
    if target_system > 0 and system_id != target_system:
        return False
    if target_component > 0 and component_id != target_component:
        return False
    if int(getattr(message, 'type', -1)) == int(mavutil.mavlink.MAV_TYPE_GCS):
        return False
    if int(getattr(message, 'autopilot', -1)) == int(
            mavutil.mavlink.MAV_AUTOPILOT_INVALID):
        return False
    return True


def wait_for_heartbeat(master, timeout_s, target_system, target_component):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        message = master.recv_match(type='HEARTBEAT', blocking=True, timeout=0.5)
        if message is None:
            continue
        if is_target_autopilot(message, target_system, target_component):
            return message
    return None


def main():
    parser = argparse.ArgumentParser(
        description='只读检查 MAVLink 飞控 heartbeat 或 OBSTACLE_DISTANCE')
    parser.add_argument(
        '--connection', default='udpin:0.0.0.0:14550',
        help='pymavlink 连接字符串')
    parser.add_argument('--baud', type=int, default=921600)
    parser.add_argument('--timeout', type=float, default=10.0)
    parser.add_argument('--target-system', type=int, default=1)
    parser.add_argument('--target-component', type=int, default=1)
    parser.add_argument(
        '--heartbeat-only', action='store_true',
        help='只等待来自目标飞控的 heartbeat')
    parser.add_argument(
        '--count', type=int, default=0,
        help='收到指定数量 OBSTACLE_DISTANCE 后退出；0 表示持续运行')
    args = parser.parse_args()

    if mavutil is None:
        print(
            '错误：未安装 pymavlink，请执行：'
            'python3 -m pip install --user pymavlink',
            file=sys.stderr,
        )
        return 2
    if args.timeout <= 0 or args.baud <= 0 or args.count < 0:
        print('错误：timeout/baud 必须为正数，count 不能为负数', file=sys.stderr)
        return 2

    master = mavutil.mavlink_connection(args.connection, baud=args.baud)
    try:
        if args.heartbeat_only:
            heartbeat = wait_for_heartbeat(
                master,
                args.timeout,
                args.target_system,
                args.target_component,
            )
            if heartbeat is None:
                print(
                    f'错误：{args.timeout:.1f} 秒内未收到目标飞控 '
                    f'{args.target_system}/{args.target_component} heartbeat',
                    file=sys.stderr,
                )
                return 3
            print(
                '飞控 heartbeat 验证通过：'
                f'sysid={heartbeat.get_srcSystem()} '
                f'compid={heartbeat.get_srcComponent()}')
            return 0

        received = 0
        start = time.monotonic()
        last = start
        print(f'正在监听 {args.connection}')
        while True:
            message = master.recv_match(
                type='OBSTACLE_DISTANCE',
                blocking=True,
                timeout=args.timeout,
            )
            if message is None:
                print(
                    f'错误：{args.timeout:.1f} 秒内未收到 OBSTACLE_DISTANCE',
                    file=sys.stderr,
                )
                return 4
            received += 1
            now = time.monotonic()
            rate = received / max(0.001, now - start)
            distances = list(message.distances)
            print(
                f'频率={rate:4.1f}Hz '
                f'前={distances[0]}cm 右={distances[18]}cm '
                f'后={distances[36]}cm 左={distances[54]}cm '
                f'间隔={now-last:0.3f}s frame={message.frame}')
            last = now
            if args.count > 0 and received >= args.count:
                return 0
    finally:
        close_method = getattr(master, 'close', None)
        if callable(close_method):
            close_method()


if __name__ == '__main__':
    sys.exit(main())
