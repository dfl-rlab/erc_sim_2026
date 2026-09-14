#!/usr/bin/env python3

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


def quaternion_to_yaw(orientation):
    """Convert a quaternion orientation into yaw in radians."""
    sin_yaw = 2.0 * (
        orientation.w * orientation.z
        + orientation.x * orientation.y
    )
    cos_yaw = 1.0 - 2.0 * (
        orientation.y * orientation.y
        + orientation.z * orientation.z
    )
    return math.atan2(sin_yaw, cos_yaw)


def normalize_angle(angle):
    """Normalize an angle to the range [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


class Rotate90(Node):
    TARGET_ROTATION = -math.pi / 2.0  # Negative means clockwise
    MAX_SPEED = 0.4                  # rad/s
    MIN_SPEED = 0.07                 # rad/s
    KP = 1.2
    TOLERANCE = math.radians(1.0)
    TIMEOUT = 120.0

    def __init__(self):
        super().__init__('rotate_90')

        self.velocity_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10,
        )

        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10,
        )

        self.current_yaw = None
        self.target_yaw = None
        self.start_time = None
        self.finished = False
        self.log_counter = 0

        self.control_timer = self.create_timer(
            0.05,
            self.control_callback,
        )

        self.get_logger().info('Waiting for /odom...')

    def odom_callback(self, message):
        self.current_yaw = quaternion_to_yaw(
            message.pose.pose.orientation
        )

        if self.target_yaw is None:
            self.target_yaw = normalize_angle(
                self.current_yaw + self.TARGET_ROTATION
            )
            self.start_time = time.monotonic()

            self.get_logger().info(
                f'Initial yaw: {math.degrees(self.current_yaw):.1f} deg'
            )
            self.get_logger().info(
                f'Target yaw: {math.degrees(self.target_yaw):.1f} deg'
            )

    def control_callback(self):
        if self.finished or self.target_yaw is None:
            return

        if time.monotonic() - self.start_time > self.TIMEOUT:
            self.get_logger().error('Rotation timed out')
            self.stop()
            self.finished = True
            return

        error = normalize_angle(
            self.target_yaw - self.current_yaw
        )

        if abs(error) <= self.TOLERANCE:
            self.stop()
            self.finished = True

            self.get_logger().info(
                f'Rotation complete. Error: '
                f'{math.degrees(error):.2f} deg'
            )
            return

        angular_speed = self.KP * error

        angular_speed = max(
            -self.MAX_SPEED,
            min(self.MAX_SPEED, angular_speed),
        )

        if abs(angular_speed) < self.MIN_SPEED:
            angular_speed = math.copysign(
                self.MIN_SPEED,
                angular_speed,
            )

        command = Twist()
        command.angular.z = angular_speed
        self.velocity_publisher.publish(command)

        self.log_counter += 1
        if self.log_counter >= 10:
            self.log_counter = 0
            self.get_logger().info(
                f'Remaining: {math.degrees(error):.1f} deg, '
                f'speed: {angular_speed:.2f} rad/s'
            )

    def stop(self):
        self.velocity_publisher.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = Rotate90()

    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)

    except KeyboardInterrupt:
        node.get_logger().info('Interrupted; stopping robot')

    finally:
        # Publish several stop messages before exiting.
        for _ in range(5):
            node.stop()
            rclpy.spin_once(node, timeout_sec=0.05)

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
