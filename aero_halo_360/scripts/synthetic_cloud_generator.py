#!/usr/bin/env python3
import math
from typing import Iterable, List, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


class SyntheticCloudGenerator(Node):
    def __init__(self):
        super().__init__('synthetic_cloud_generator')
        self.topic = self.param('topic', '/livox/lidar')
        self.frame_id = self.param('frame_id', 'base_link')
        self.wall_direction = self.param('wall_direction', 'front')
        self.wall_distance_m = float(self.param('wall_distance_m', 2.0))
        self.wall_width_m = float(self.param('wall_width_m', 2.0))
        self.wall_height_m = float(self.param('wall_height_m', 1.0))
        self.noise_points = int(self.param('noise_points', 0))
        self.rate_hz = float(self.param('rate_hz', 10.0))
        self.publisher = self.create_publisher(PointCloud2, self.topic, 10)
        self.create_timer(1.0 / max(1.0, self.rate_hz), self.publish_cloud)
        self.get_logger().info(f'合成点云已启动: 方向={self.wall_direction} 距离={self.wall_distance_m:.2f} m 话题={self.topic}')

    def param(self, name, default):
        if not self.has_parameter(name):
            self.declare_parameter(name, default)
        return self.get_parameter(name).value

    def wall_points(self) -> Iterable[Tuple[float, float, float]]:
        samples_side = 25
        samples_z = 8
        half_width = self.wall_width_m / 2.0
        z_min = -self.wall_height_m / 2.0
        z_step = self.wall_height_m / max(1, samples_z - 1)
        side_step = self.wall_width_m / max(1, samples_side - 1)
        direction = self.wall_direction.lower()

        for i in range(samples_side):
            side = -half_width + side_step * i
            for j in range(samples_z):
                z = z_min + z_step * j
                if direction == 'right':
                    yield (side, -self.wall_distance_m, z)
                elif direction == 'rear':
                    yield (-self.wall_distance_m, side, z)
                elif direction == 'left':
                    yield (side, self.wall_distance_m, z)
                else:
                    yield (self.wall_distance_m, side, z)

    def publish_cloud(self):
        points: List[Tuple[float, float, float]] = list(self.wall_points())
        for i in range(self.noise_points):
            angle = i * 2.399963229728653
            radius = 0.5 + (i % 20) * 0.05
            points.append((math.cos(angle) * radius, math.sin(angle) * radius, 0.1))

        cloud_header = Header()
        cloud_header.stamp = self.get_clock().now().to_msg()
        cloud_header.frame_id = self.frame_id
        msg = point_cloud2.create_cloud_xyz32(cloud_header, points)
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = SyntheticCloudGenerator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
