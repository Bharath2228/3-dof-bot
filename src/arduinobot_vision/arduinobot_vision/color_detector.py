from arduinobot_vision.block_detector import BlockDetector
import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class ColorDetector(Node):
    def __init__(self):
        super().__init__('color_detector')

        self.subscription = self.create_subscription(
            Image,
            '/arduinobot/top_camera/image',
            self.image_callback,
            10
        )

        self.bridge = CvBridge()

        self.red_pub = self.create_publisher(PointStamped, '/detected_objects/red', 10)
        self.blue_pub = self.create_publisher(PointStamped, '/detected_objects/blue', 10)

        self.declare_parameter('red_range1_low', [0, 100, 100])
        self.declare_parameter('red_range1_high', [10, 255, 255])
        self.declare_parameter('red_range2_low', [170, 100, 100])
        self.declare_parameter('red_range2_high', [180, 255, 255])
        self.declare_parameter('blue_range_low', [100, 100, 100])
        self.declare_parameter('blue_range_high', [130, 255, 255])
        self.declare_parameter('ref_u', 291)
        self.declare_parameter('ref_v', 225)
        self.declare_parameter('ref_x', 0.1)
        self.declare_parameter('ref_y', 1.5)
        self.declare_parameter('scale_x', -135.0)
        self.declare_parameter('scale_y', -132.0)
        self.declare_parameter('block_height', 0.45)

        self.block_height = self.get_parameter('block_height').value

        self.detector = BlockDetector(
            red_ranges=(
                (np.array(self.get_parameter('red_range1_low').value), np.array(self.get_parameter('red_range1_high').value)),
                (np.array(self.get_parameter('red_range2_low').value), np.array(self.get_parameter('red_range2_high').value)),
            ),
            blue_range=(np.array(self.get_parameter('blue_range_low').value), np.array(self.get_parameter('blue_range_high').value)),
            ref_u=self.get_parameter('ref_u').value,
            ref_v=self.get_parameter('ref_v').value,
            ref_x=self.get_parameter('ref_x').value,
            ref_y=self.get_parameter('ref_y').value,
            scale_x=self.get_parameter('scale_x').value,
            scale_y=self.get_parameter('scale_y').value,
        )

        self.publishers_by_color = {
            'red': self.red_pub,
            'blue': self.blue_pub,
        }

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        centroids = self.detector.detect_colors(hsv)
        block_height = self.block_height

        for color, centroid in centroids.items():
            if centroid is None:
                continue
            wx, wy = self.detector.pixel_to_world(*centroid)
            point = PointStamped()
            point.header.stamp = self.get_clock().now().to_msg()
            point.header.frame_id = 'base_link'
            point.point.x = wx
            point.point.y = wy
            point.point.z = block_height
            self.publishers_by_color[color].publish(point)


def main(args=None):
    rclpy.init(args=args)
    node = ColorDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
