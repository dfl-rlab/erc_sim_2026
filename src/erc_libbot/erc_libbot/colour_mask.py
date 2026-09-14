#!/usr/bin/env python3

import time

import cv2
import numpy as np
import rclpy

from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class ColourMask(Node):
    # OpenCV HSV ranges. Black is intentionally excluded.
    COLOUR_RANGES = {
        'red': [
            ((0, 100, 60), (10, 255, 255)),
            ((170, 100, 60), (180, 255, 255)),
        ],
        'green': [
            ((35, 80, 40), (85, 255, 255)),
        ],
        'blue': [
            ((100, 100, 40), (140, 255, 255)),
        ],
        'yellow': [
            ((20, 100, 80), (35, 255, 255)),
        ],
    }

    def __init__(self):
        super().__init__('colour_mask')

        self.declare_parameter('colour', 'blue')
        self.declare_parameter(
            'input_topic',
            '/head_front_camera/head_front_camera/color/image_raw',
        )
        self.declare_parameter(
            'mask_topic',
            '/colour_mask',
        )
        self.declare_parameter(
            'annotated_topic',
            '/colour_mask/annotated',
        )
        self.declare_parameter('min_component_pixels', 20)
        self.declare_parameter('max_component_pixels', 5000)

        self.input_topic = self.get_parameter(
            'input_topic'
        ).value
        self.mask_topic = self.get_parameter(
            'mask_topic'
        ).value
        self.annotated_topic = self.get_parameter(
            'annotated_topic'
        ).value

        self.min_component_pixels = int(
            self.get_parameter('min_component_pixels').value
        )
        self.max_component_pixels = int(
            self.get_parameter('max_component_pixels').value
        )

        self.bridge = CvBridge()
        self.kernel = np.ones((3, 3), dtype=np.uint8)
        self.last_log_time = 0.0
        self.last_colour = None

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

        self.image_subscription = self.create_subscription(
            Image,
            self.input_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            'Available colours: red, green, blue, yellow'
        )
        self.get_logger().info(
            f'Publishing mask on {self.mask_topic}'
        )
        self.get_logger().info(
            f'Publishing annotated mask on '
            f'{self.annotated_topic}'
        )

    def create_colour_mask(self, hsv_image, colour):
        ranges = self.COLOUR_RANGES[colour]
        combined_mask = np.zeros(
            hsv_image.shape[:2],
            dtype=np.uint8,
        )

        for lower_values, upper_values in ranges:
            lower = np.array(
                lower_values,
                dtype=np.uint8,
            )
            upper = np.array(
                upper_values,
                dtype=np.uint8,
            )

            range_mask = cv2.inRange(
                hsv_image,
                lower,
                upper,
            )

            combined_mask = cv2.bitwise_or(
                combined_mask,
                range_mask,
            )

        combined_mask = cv2.morphologyEx(
            combined_mask,
            cv2.MORPH_OPEN,
            self.kernel,
        )
        combined_mask = cv2.morphologyEx(
            combined_mask,
            cv2.MORPH_CLOSE,
            self.kernel,
        )

        return combined_mask

    def find_components(self, mask):
        label_count, labels, statistics, _ = (
            cv2.connectedComponentsWithStats(
                mask,
                connectivity=8,
            )
        )

        image_height = mask.shape[0]
        components = []

        for label in range(1, label_count):
            area = int(
                statistics[label, cv2.CC_STAT_AREA]
            )

            if not (
                self.min_component_pixels
                <= area
                <= self.max_component_pixels
            ):
                continue

            x_pixels = np.where(labels == label)[1]
            y_pixels = np.where(labels == label)[0]

            median_x = int(np.median(x_pixels))
            median_y = int(np.median(y_pixels))

            components.append({
                'median_x': median_x,
                'median_y': median_y,
                'height_from_bottom': (
                    image_height - 1 - median_y
                ),
            })

        components.sort(
            key=lambda component: component['median_x']
        )

        return components

    def image_callback(self, message):
        try:
            colour = str(
                self.get_parameter('colour').value
            ).lower()

            if colour not in self.COLOUR_RANGES:
                self.get_logger().error(
                    f'Unsupported colour "{colour}". '
                    'Use red, green, blue, or yellow.'
                )
                return

            if colour != self.last_colour:
                self.get_logger().info(
                    f'Now detecting: {colour}'
                )
                self.last_colour = colour

            image = self.bridge.imgmsg_to_cv2(
                message,
                desired_encoding='bgr8',
            )

            hsv_image = cv2.cvtColor(
                image,
                cv2.COLOR_BGR2HSV,
            )

            mask = self.create_colour_mask(
                hsv_image,
                colour,
            )

            components = self.find_components(mask)

            self.publish_mask(mask, message)
            self.publish_annotated(
                mask,
                components,
                colour,
                message,
            )
            self.log_components(colour, components)

        except Exception as error:
            self.get_logger().error(
                f'Colour-mask processing failed: {error}'
            )

    def publish_mask(self, mask, input_message):
        output = self.bridge.cv2_to_imgmsg(
            mask,
            encoding='mono8',
        )
        output.header = input_message.header
        self.mask_publisher.publish(output)

    def publish_annotated(
        self,
        mask,
        components,
        colour,
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
            x = component['median_x']
            y = component['median_y']
            height = component['height_from_bottom']

            cv2.circle(
                annotated,
                (x, y),
                5,
                (0, 0, 255),
                -1,
            )

            cv2.putText(
                annotated,
                f'{index}: x={x}, h={height}',
                (x + 7, y - 7),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.4,
                (0, 255, 0),
                1,
                cv2.LINE_AA,
            )

        cv2.putText(
            annotated,
            f'Detecting: {colour}',
            (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )

        output = self.bridge.cv2_to_imgmsg(
            annotated,
            encoding='bgr8',
        )
        output.header = input_message.header
        self.annotated_publisher.publish(output)

    def log_components(self, colour, components):
        now = time.monotonic()

        if now - self.last_log_time < 1.0:
            return

        self.last_log_time = now

        if not components:
            self.get_logger().warning(
                f'No {colour} regions detected'
            )
            return

        descriptions = []

        for index, component in enumerate(
            components,
            start=1,
        ):
            descriptions.append(
                f'{index}: '
                f'x={component["median_x"]}, '
                f'height={component["height_from_bottom"]}px'
            )

        self.get_logger().info(
            f'{colour.capitalize()} regions left-to-right: '
            + ' | '.join(descriptions)
        )


def main(args=None):
    rclpy.init(args=args)
    node = ColourMask()

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
