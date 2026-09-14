#!/usr/bin/env python3

import time

import cv2
import numpy as np
import rclpy

from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray


class BlackMask(Node):
    def __init__(self):
        super().__init__('black_mask')

        self.declare_parameter(
            'input_topic',
            '/head_front_camera/head_front_camera/color/image_raw',
        )
        self.declare_parameter('mask_topic', '/black_mask')
        self.declare_parameter(
            'annotated_topic',
            '/black_mask/annotated',
        )
        self.declare_parameter(
            'counts_topic',
            '/black_mask/pixel_counts',
        )

        self.declare_parameter('max_value', 100)
        self.declare_parameter('min_component_pixels', 10)
        self.declare_parameter('max_component_pixels', 3000)
        self.declare_parameter('expected_components', 5)

        self.input_topic = self.get_parameter(
            'input_topic'
        ).value
        self.mask_topic = self.get_parameter(
            'mask_topic'
        ).value
        self.annotated_topic = self.get_parameter(
            'annotated_topic'
        ).value
        self.counts_topic = self.get_parameter(
            'counts_topic'
        ).value

        self.max_value = int(
            self.get_parameter('max_value').value
        )
        self.min_component_pixels = int(
            self.get_parameter('min_component_pixels').value
        )
        self.max_component_pixels = int(
            self.get_parameter('max_component_pixels').value
        )
        self.expected_components = int(
            self.get_parameter('expected_components').value
        )

        self.max_value = max(0, min(255, self.max_value))

        self.bridge = CvBridge()
        self.kernel = np.ones((3, 3), dtype=np.uint8)
        self.last_log_time = 0.0

        self.mask_publisher = self.create_publisher(
            Image,
            self.mask_topic,
            10,
        )

        self.annotated_publisher = self.create_publisher(
            Image,
            self.annotated_topic,
            10,
        )

        self.counts_publisher = self.create_publisher(
            Int32MultiArray,
            self.counts_topic,
            10,
        )

        self.image_subscription = self.create_subscription(
            Image,
            self.input_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f'Reading RGB images from {self.input_topic}'
        )
        self.get_logger().info(
            f'Publishing binary mask on {self.mask_topic}'
        )
        self.get_logger().info(
            f'Publishing annotated mask on {self.annotated_topic}'
        )
        self.get_logger().info(
            f'Publishing pixel counts on {self.counts_topic}'
        )

    def image_callback(self, message):
        try:
            bgr_image = self.bridge.imgmsg_to_cv2(
                message,
                desired_encoding='bgr8',
            )

            hsv_image = cv2.cvtColor(
                bgr_image,
                cv2.COLOR_BGR2HSV,
            )

            lower_black = np.array(
                [0, 0, 0],
                dtype=np.uint8,
            )
            upper_black = np.array(
                [180, 255, self.max_value],
                dtype=np.uint8,
            )

            mask = cv2.inRange(
                hsv_image,
                lower_black,
                upper_black,
            )

            mask = cv2.morphologyEx(
                mask,
                cv2.MORPH_OPEN,
                self.kernel,
            )
            mask = cv2.morphologyEx(
                mask,
                cv2.MORPH_CLOSE,
                self.kernel,
            )

            components = self.find_components(mask)

            pixel_counts = [
                component['area']
                for component in components
            ]

            self.publish_mask(mask, message)
            self.publish_counts(pixel_counts)
            self.publish_annotated(mask, components, message)
            self.log_components(pixel_counts)

        except Exception as error:
            self.get_logger().error(
                f'Image processing failed: {error}'
            )

    def find_components(self, mask):
        number_of_labels, _, statistics, _ = (
            cv2.connectedComponentsWithStats(
                mask,
                connectivity=8,
            )
        )

        components = []

        # Label 0 is the black background, so begin at label 1.
        for label in range(1, number_of_labels):
            x = int(
                statistics[label, cv2.CC_STAT_LEFT]
            )
            y = int(
                statistics[label, cv2.CC_STAT_TOP]
            )
            width = int(
                statistics[label, cv2.CC_STAT_WIDTH]
            )
            height = int(
                statistics[label, cv2.CC_STAT_HEIGHT]
            )
            area = int(
                statistics[label, cv2.CC_STAT_AREA]
            )

            if area < self.min_component_pixels:
                continue

            if area > self.max_component_pixels:
                continue

            components.append({
                'x': x,
                'y': y,
                'width': width,
                'height': height,
                'area': area,
            })

        # Sort by the left edge of each component.
        components.sort(key=lambda component: component['x'])

        return components

    def publish_mask(self, mask, input_message):
        output_message = self.bridge.cv2_to_imgmsg(
            mask,
            encoding='mono8',
        )
        output_message.header = input_message.header
        self.mask_publisher.publish(output_message)

    def publish_counts(self, pixel_counts):
        message = Int32MultiArray()
        message.data = pixel_counts
        self.counts_publisher.publish(message)

    def publish_annotated(
        self,
        mask,
        components,
        input_message,
    ):
        annotated = cv2.cvtColor(
            mask,
            cv2.COLOR_GRAY2BGR,
        )

        for index, component in enumerate(
            components,
            start=1,
        ):
            x = component['x']
            y = component['y']
            width = component['width']
            height = component['height']
            area = component['area']

            colour = (
                (0, 255, 0)
                if len(components) == self.expected_components
                else (0, 165, 255)
            )

            cv2.rectangle(
                annotated,
                (x, y),
                (x + width, y + height),
                colour,
                2,
            )

            cv2.putText(
                annotated,
                f'{index}: {area}px',
                (x, max(15, y - 5)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                colour,
                1,
                cv2.LINE_AA,
            )

        output_message = self.bridge.cv2_to_imgmsg(
            annotated,
            encoding='bgr8',
        )
        output_message.header = input_message.header
        self.annotated_publisher.publish(output_message)

    def log_components(self, pixel_counts):
        current_time = time.monotonic()

        # Prevent the terminal from being flooded.
        if current_time - self.last_log_time < 1.0:
            return

        self.last_log_time = current_time

        formatted_counts = ', '.join(
            f'{index}={count}px'
            for index, count in enumerate(
                pixel_counts,
                start=1,
            )
        )

        message = (
            f'Components: {len(pixel_counts)}; '
            f'left-to-right pixel counts: [{formatted_counts}]'
        )

        if len(pixel_counts) == self.expected_components:
            self.get_logger().info(message)
        else:
            self.get_logger().warning(
                message
                + f'; expected {self.expected_components}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = BlackMask()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
