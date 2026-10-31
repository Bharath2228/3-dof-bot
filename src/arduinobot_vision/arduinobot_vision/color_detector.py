import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped

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

    def find_centroid(self, hsv, lower, upper):
        mask = cv2.inRange(hsv, lower, upper)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        largest = max(contours, key=cv2.contourArea)
        M = cv2.moments(largest)
        if M['m00'] == 0:
            return None
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        return (cx, cy)

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        red_mask1 = (np.array([0, 100, 100]), np.array([10, 255, 255]))
        red_mask2 = (np.array([170, 100, 100]), np.array([180, 255, 255]))
        red_centroid = self.find_centroid(hsv, *red_mask1) or self.find_centroid(hsv, *red_mask2)

        blue_lower = np.array([100, 100, 100])
        blue_upper = np.array([130, 255, 255])
        blue_centroid = self.find_centroid(hsv, blue_lower, blue_upper)

        block_height = 0.45

        if red_centroid:
            wx, wy = self.pixel_to_world(*red_centroid)
            point = PointStamped()
            point.header.stamp = self.get_clock().now().to_msg()
            point.header.frame_id = 'base_link'
            point.point.x = wx
            point.point.y = wy
            point.point.z = block_height
            self.red_pub.publish(point)

        if blue_centroid:
            wx, wy = self.pixel_to_world(*blue_centroid)
            point = PointStamped()
            point.header.stamp = self.get_clock().now().to_msg()
            point.header.frame_id = 'base_link'
            point.point.x = wx
            point.point.y = wy
            point.point.z = block_height
            self.blue_pub.publish(point)

    def pixel_to_world(self, u, v):
        ref_u, ref_v = 291, 225
        ref_x, ref_y = 0.1, 1.5
        scale_x = -135.0
        scale_y = -132.0

        world_x = ref_x + (v - ref_v) / scale_x
        world_y = ref_y + (u - ref_u) / scale_y
        return world_x, world_y

def main(args=None):
    rclpy.init(args=args)
    node = ColorDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()