#!/usr/bin/env python3
import argparse
import sys
import time

import rclpy
from rclpy.node import Node

from aero_halo_360.msg import SectorDistances

DIRECTION_SECTORS = {
    'front': 0,
    'right': 18,
    'rear': 36,
    'left': 54,
}


class DirectionChecker(Node):
    def __init__(self, direction: str, max_distance_cm: int, topic: str):
        super().__init__('aero_halo_360_direction_checker')
        self.direction = direction
        self.expected_sector = DIRECTION_SECTORS[direction]
        self.max_distance_cm = max_distance_cm
        self.result = None
        self.create_subscription(SectorDistances, topic, self.on_message, 10)

    def on_message(self, message):
        distances = list(message.distances)
        if len(distances) != 72:
            self.result = (False, f'扇区数量错误: {len(distances)}')
            return
        nearest_sector = min(range(72), key=lambda index: distances[index])
        nearest_distance = distances[nearest_sector]
        sector_error = min(
            (nearest_sector - self.expected_sector) % 72,
            (self.expected_sector - nearest_sector) % 72,
        )
        if sector_error <= 1 and nearest_distance <= self.max_distance_cm:
            self.result = (
                True,
                f'{self.direction} 方向通过: sector={nearest_sector}, '
                f'distance={nearest_distance} cm',
            )


def main():
    parser = argparse.ArgumentParser(
        description='用单个近距离障碍物校验 AeroHalo360 四方向扇区映射')
    parser.add_argument('--direction', required=True, choices=DIRECTION_SECTORS)
    parser.add_argument('--topic', default='/aero_halo_360/sector_distances')
    parser.add_argument('--max-distance-cm', type=int, default=300)
    parser.add_argument('--timeout', type=float, default=10.0)
    args = parser.parse_args()

    rclpy.init()
    node = DirectionChecker(args.direction, args.max_distance_cm, args.topic)
    deadline = time.monotonic() + args.timeout
    try:
        while rclpy.ok() and time.monotonic() < deadline and node.result is None:
            rclpy.spin_once(node, timeout_sec=0.2)
        if node.result is None:
            print(
                f'失败: {args.timeout:.1f} 秒内未观察到 {args.direction} '
                '方向的唯一近障碍物',
                file=sys.stderr,
            )
            return 2
        passed, message = node.result
        print(message)
        return 0 if passed else 3
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
