import math
from typing import cast

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from custom_interfaces.msg import ObstacleDetection


class LaserObstacleDetector(Node):

    def __init__(self):
        super().__init__('laser_obstacle_detector')

        # Parameters
        self.declare_parameter('angle_min', -0.52)
        self.declare_parameter('angle_max', 0.52)
        self.declare_parameter('distance_threshold', 0.8)
        self.declare_parameter('confidence_threshold', 0.10)
        self.declare_parameter('scan_topic', 'scan')
        self.declare_parameter(
            'output_topic', '/laser_obstacle/detection'
        )
        self.declare_parameter('min_valid_rays', 5)

        self.angle_min: float = self.get_parameter('angle_min').get_parameter_value().double_value
        self.angle_max: float = self.get_parameter('angle_max').get_parameter_value().double_value
        self.distance_threshold: float = self.get_parameter('distance_threshold').get_parameter_value().double_value
        self.confidence_threshold: float = self.get_parameter('confidence_threshold').get_parameter_value().double_value
        self.min_valid_rays: int = self.get_parameter('min_valid_rays').get_parameter_value().integer_value

        if self.distance_threshold <= 0.0:
            raise ValueError('distance_threshold must be positive')

        if not 0.0 <= self.confidence_threshold <= 1.0:
            raise ValueError('confidence_threshold must be between 0 and 1')

        if self.min_valid_rays < 1:
            raise ValueError('min_valid_rays must be at least 1')

        # ROS interfaces
        scan_topic = str(self.get_parameter('scan_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)

        self.publisher = self.create_publisher(
            ObstacleDetection,
            output_topic,
            10
        )

        self.subscription = self.create_subscription(
            LaserScan,
            scan_topic,
            self.scan_callback,
            10
        )

        self.get_logger().info(
            f'Laser obstacle detector started. '
            f'Scan: {scan_topic}, output: {output_topic}'
        )

    @staticmethod
    def normalize_angle(angle: float):
        return math.atan2(math.sin(angle), math.cos(angle))

    def angle_in_range(self, angle: float):
        angle = self.normalize_angle(angle)
        lower = self.normalize_angle(self.angle_min)
        upper = self.normalize_angle(self.angle_max)

        # Supports angular ranges crossing -pi/pi.
        if lower <= upper:
            return lower <= angle <= upper

        return angle >= lower or angle <= upper

    def scan_callback(self, scan):

        obstacle_ray_count = 0
        total_ray_count = 0

        for index, distance in enumerate(scan.ranges):

            angle = scan.angle_min + index * scan.angle_increment

            if not self.angle_in_range(angle):
                continue

            # Ignore NaN, infinity and readings outside sensor limits.
            if not math.isfinite(distance):
                continue

            if distance < scan.range_min or distance > scan.range_max:
                continue

            total_ray_count += 1

            if distance < self.distance_threshold:
                obstacle_ray_count += 1

        # Calculate confidence.
        confidence = (
            obstacle_ray_count / total_ray_count
            if total_ray_count > 0
            else 0.0
        )

        valid = total_ray_count >= self.min_valid_rays

        obstacle_present = (
            valid and confidence >= self.confidence_threshold
        )

        # Publish result.
        msg = ObstacleDetection()

        # Preserve the original LiDAR scan timestamp.
        msg.header = scan.header

        msg.valid = valid
        msg.obstacle_present = obstacle_present
        msg.confidence = float(confidence)
        msg.obstacle_ray_count = obstacle_ray_count
        msg.total_ray_count = total_ray_count

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = LaserObstacleDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()