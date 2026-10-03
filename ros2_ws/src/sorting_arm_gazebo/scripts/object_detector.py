#!/usr/bin/env python3

import json

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from sensor_msgs.msg import Image
from std_msgs.msg import String

from cv_bridge import CvBridge

import cv2
import numpy as np


class ObjectDetector(Node):

    def __init__(self):
        super().__init__('object_detector')

        self.bridge = CvBridge()

        # Camera subscriber
        self.subscription = self.create_subscription(
            Image,
            '/sorting_camera/image',
            self.image_callback,
            10
        )

        # Fixed world coordinates for the four object positions.
        self.positions = {
            'position_1': (-0.45, 0.20, 0.85),
            'position_2': (-0.15, 0.20, 0.85),
            'position_3': (0.15, 0.20, 0.85),
            'position_4': (0.45, 0.20, 0.85),
        }

        # Keep the most recent detection available to nodes that
        # subscribe after the message was published.
        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.publisher = self.create_publisher(
            String,
            '/detected_objects',
            qos
        )

        self.detection_published = False

        self.get_logger().info('Object detector started.')

    def detect_color(self, roi):
        """Return the dominant object color in an image region."""

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # Red has two regions in HSV because hue wraps around.
        red_mask_1 = cv2.inRange(
            hsv,
            np.array([0, 100, 80]),
            np.array([10, 255, 255])
        )

        red_mask_2 = cv2.inRange(
            hsv,
            np.array([170, 100, 80]),
            np.array([179, 255, 255])
        )

        red_mask = red_mask_1 | red_mask_2

        # Blue hue range.
        blue_mask = cv2.inRange(
            hsv,
            np.array([100, 100, 80]),
            np.array([140, 255, 255])
        )

        red_pixels = cv2.countNonZero(red_mask)
        blue_pixels = cv2.countNonZero(blue_mask)

        if red_pixels > blue_pixels and red_pixels > 20:
            return 'red'

        if blue_pixels > red_pixels and blue_pixels > 20:
            return 'blue'

        return 'unknown'

    def image_callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )

            # Approximate pixel centers of the four fixed object positions.
            image_positions = {
                'position_1': (167, 171),
                'position_2': (263, 172),
                'position_3': (370, 172),
                'position_4': (472, 172),
            }

            half_width = 25
            half_height = 25

            detected_objects = []

            for position_name, (cx, cy) in image_positions.items():

                x1 = cx - half_width
                x2 = cx + half_width
                y1 = cy - half_height
                y2 = cy + half_height

                roi = image[y1:y2, x1:x2]

                color = self.detect_color(roi)

                # Get the fixed world coordinates for this position.
                x, y, z = self.positions[position_name]

                detected_objects.append({
                    'color': color,
                    'x': x,
                    'y': y,
                    'z': z
                })

                # Draw the detection region.
                cv2.rectangle(
                    image,
                    (x1, y1),
                    (x2, y2),
                    (255, 255, 255),
                    2
                )

                cv2.putText(
                    image,
                    color,
                    (x1, y1 - 5),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 255, 255),
                    1
                )

            # Convert the Python list into JSON.
            json_message = json.dumps(detected_objects)

            # Publish the JSON message once.
            if not self.detection_published:
                message = String()
                message.data = json_message

                self.publisher.publish(message)

                self.detection_published = True

                self.get_logger().info(
                    f'Published detected objects: {json_message}'
                )

            cv2.imshow('Sorting Camera', image)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(
                f'Image processing failed: {e}'
            )


def main(args=None):
    rclpy.init(args=args)

    node = ObjectDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()