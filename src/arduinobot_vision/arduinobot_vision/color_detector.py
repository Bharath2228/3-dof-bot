from arduinobot_vision.block_detector import BlockDetector
import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
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

        self.detector = BlockDetector()
        self.publishers_by_color = {
            'red': self.red_pub,
            'blue': self.blue_pub,
        }

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        centroids = self.detector.detect_colors(hsv)
        block_height = 0.45

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
