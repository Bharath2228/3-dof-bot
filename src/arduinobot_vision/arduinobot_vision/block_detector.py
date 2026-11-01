import cv2
import numpy as np


class BlockDetector:
    def __init__(
        self,
        red_ranges=None,
        blue_range=None,
        ref_u=291,
        ref_v=225,
        ref_x=0.1,
        ref_y=1.5,
        scale_x=-135.0,
        scale_y=-132.0,
    ):
        self.red_ranges = red_ranges or (
            (np.array([0, 100, 100]), np.array([10, 255, 255])),
            (np.array([170, 100, 100]), np.array([180, 255, 255])),
        )
        self.blue_range = blue_range or (np.array([100, 100, 100]), np.array([130, 255, 255]))
        self.ref_u = ref_u
        self.ref_v = ref_v
        self.ref_x = ref_x
        self.ref_y = ref_y
        self.scale_x = scale_x
        self.scale_y = scale_y

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
            self.find_centroid(hsv, *self.red_ranges[0])
            or self.find_centroid(hsv, *self.red_ranges[1])
        )
        blue_centroid = self.find_centroid(hsv, *self.blue_range)
        return {'red': red_centroid, 'blue': blue_centroid}

    def pixel_to_world(self, u, v):
        world_x = self.ref_x + (v - self.ref_v) / self.scale_x
        world_y = self.ref_y + (u - self.ref_u) / self.scale_y
        return world_x, world_y
