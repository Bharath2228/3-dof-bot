import cv2
import numpy as np


class BlockDetector:
    RED_RANGES = (
        (np.array([0, 100, 100]), np.array([10, 255, 255])),
        (np.array([170, 100, 100]), np.array([180, 255, 255])),
    )
    BLUE_RANGE = (np.array([100, 100, 100]), np.array([130, 255, 255]))

    REF_U, REF_V = 291, 225
    REF_X, REF_Y = 0.1, 1.5
    SCALE_X = -135.0
    SCALE_Y = -132.0

    def find_centroid(self, hsv, lower, upper):

        mask = cv2.inRange(hsv, lower, upper)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        largest = max(contours, key=cv2.contourArea)
        moments = cv2.moments(largest)
        if moments['m00'] == 0:
            return None
        cx = int(moments['m10'] / moments['m00'])
        cy = int(moments['m01'] / moments['m00'])

        return (cx, cy)

    def detect_colors(self, hsv):
        red_centroid = (
            self.find_centroid(hsv, *self.RED_RANGES[0])
            or self.find_centroid(hsv, *self.RED_RANGES[1])
        )
        blue_centroid = self.find_centroid(hsv, *self.BLUE_RANGE)
        return {'red': red_centroid, 'blue': blue_centroid}

    def pixel_to_world(self, u, v):
        world_x = self.REF_X + (v - self.REF_V) / self.SCALE_X
        world_y = self.REF_Y + (u - self.REF_U) / self.SCALE_Y
        return world_x, world_y
