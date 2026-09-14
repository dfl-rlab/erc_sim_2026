#!/usr/bin/env python3

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


def quaternion_to_yaw(orientation):
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
    return math.atan2(math.sin(angle), math.cos(angle))


class MoveDistance(Node):
    def __init__(self):
        super().__init__('move_distance')

        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('max_speed', 0.30)
        self.declare_parameter('min_speed', 0.04)
        self.declare_parameter('kp', 0.8)
        self.declare_parameter('yaw_kp', 1.5)
        self.declare_parameter('max_yaw_speed', 0.30)
        self.declare_parameter('tolerance', 0.02)
        self.declare_parameter('timeout', 180.0)

        self.requested_x = float(
            self.get_parameter('x').value
        )
        self.requested_y = float(
            self.get_parameter('y').value
        )
        self.max_speed = abs(float(
            self.get_parameter('max_speed').value
        ))
        self.min_speed = abs(float(
            self.get_parameter('min_speed').value
        ))
        self.kp = abs(float(
            self.get_parameter('kp').value
        ))
        self.yaw_kp = abs(float(
            self.get_parameter('yaw_kp').value
        ))
        self.max_yaw_speed = abs(float(
            self.get_parameter('max_yaw_speed').value
        ))
        self.tolerance = abs(float(
            self.get_parameter('tolerance').value
        ))
        self.timeout = abs(float(
            self.get_parameter('timeout').value
        ))

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

        self.start_x = None
        self.start_y = None
        self.start_yaw = None

        self.target_x = None
        self.target_y = None

        self.current_x = None
        self.current_y = None
        self.current_yaw = None

        self.start_time = None
        self.last_log_time = 0.0
        self.finished = False

        self.control_timer = self.create_timer(
            0.05,
            self.control_callback,
        )

        self.get_logger().info(
            f'Waiting for odometry. Requested movement: '
            f'x={self.requested_x:.2f} m, '
            f'y={self.requested_y:.2f} m'
        )

    def odom_callback(self, message):
        position = message.pose.pose.position

        self.current_x = position.x
        self.current_y = position.y
        self.current_yaw = quaternion_to_yaw(
            message.pose.pose.orientation
        )

        if self.start_x is not None:
            return

        self.start_x = self.current_x
        self.start_y = self.current_y
        self.start_yaw = self.current_yaw
        self.start_time = time.monotonic()

        # Convert the robot-relative request into odom coordinates.
        self.target_x = (
            self.start_x
            + self.requested_x * math.cos(self.start_yaw)
            - self.requested_y * math.sin(self.start_yaw)
        )
        self.target_y = (
            self.start_y
            + self.requested_x * math.sin(self.start_yaw)
            + self.requested_y * math.cos(self.start_yaw)
        )

        self.get_logger().info(
            f'Target in odom frame: '
            f'x={self.target_x:.3f}, '
            f'y={self.target_y:.3f}'
        )

    def control_callback(self):
        if self.finished or self.target_x is None:
            return

        if time.monotonic() - self.start_time > self.timeout:
            self.get_logger().error('Movement timed out')
            self.finish()
            return

        error_x_odom = self.target_x - self.current_x
        error_y_odom = self.target_y - self.current_y

        remaining = math.hypot(
            error_x_odom,
            error_y_odom,
        )

        if remaining <= self.tolerance:
            self.get_logger().info(
                f'Target reached. Remaining error: '
                f'{remaining:.3f} m'
            )
            self.finish()
            return

        # Convert odom-frame error into the robot's current frame.
        cos_yaw = math.cos(self.current_yaw)
        sin_yaw = math.sin(self.current_yaw)

        error_x_robot = (
            cos_yaw * error_x_odom
            + sin_yaw * error_y_odom
        )
        error_y_robot = (
            -sin_yaw * error_x_odom
            + cos_yaw * error_y_odom
        )

        velocity_x = self.kp * error_x_robot
        velocity_y = self.kp * error_y_robot

        translation_speed = math.hypot(
            velocity_x,
            velocity_y,
        )

        # Limit combined diagonal speed.
        if translation_speed > self.max_speed:
            scale = self.max_speed / translation_speed
            velocity_x *= scale
            velocity_y *= scale
            translation_speed = self.max_speed

        # Prevent the robot from stalling close to the target.
        if 0.0 < translation_speed < self.min_speed:
            scale = self.min_speed / translation_speed
            velocity_x *= scale
            velocity_y *= scale

        # Maintain the orientation that the robot started with.
        yaw_error = normalize_angle(
            self.start_yaw - self.current_yaw
        )
        angular_velocity = self.yaw_kp * yaw_error
        angular_velocity = max(
            -self.max_yaw_speed,
            min(self.max_yaw_speed, angular_velocity),
        )

        command = Twist()
        command.linear.x = velocity_x
        command.linear.y = velocity_y
        command.angular.z = angular_velocity

        self.velocity_publisher.publish(command)

        now = time.monotonic()

        if now - self.last_log_time >= 1.0:
            self.last_log_time = now

            self.get_logger().info(
                f'Remaining={remaining:.2f} m, '
                f'vx={velocity_x:.2f}, '
                f'vy={velocity_y:.2f}, '
                f'yaw error={math.degrees(yaw_error):.1f} deg'
            )

    def finish(self):
        self.velocity_publisher.publish(Twist())
        self.finished = True

    def publish_stop(self):
        self.velocity_publisher.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = MoveDistance()

    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)

    except KeyboardInterrupt:
        node.get_logger().info(
            'Interrupted; stopping robot'
        )

    finally:
        for _ in range(5):
            node.publish_stop()
            rclpy.spin_once(node, timeout_sec=0.05)

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
