import cv2
from math import cos, sin, pi
import numpy as np

class LidarImage:
    def __init__(self, robot_width, robot_length, max_range):
        self.image_width = 640
        self.image_height = 480
        self.floor_color = (40, 40, 40)
        self.robot_color = (180, 180, 180)
        self.scanner_radius = 3

        self.robot_width = robot_width
        self.robot_length = robot_length
        self.image = np.zeros(shape=(self.image_height, self.image_width, 3), dtype=np.uint8)
        self.max_distance = 0
        self.max_range = max_range

        self.image_center_x = int(self.image_width / 2)
        self.image_center_y = int(self.image_height / 2)

        self.robot_top_left = self._field_to_image(-(self.robot_width / 2), -(self.robot_length / 2))
        self.robot_bottom_right = self._field_to_image(self.robot_width / 2, self.robot_length / 2)

    def _field_to_image(self, field_x, field_y):
        return (
            self.image_center_x + (int((field_x / (self.max_range * 2)) * self.image_width)),
            self.image_center_y + (int((field_y / (self.max_range * 2)) * self.image_height))
        )

    def clear(self):
        self.image[:] = self.floor_color

    def drawRobot(self):
        cv2.rectangle(self.image, self.robot_top_left, self.robot_bottom_right, self.robot_color, -1)

    def drawScan(self, data, scanner_x, scanner_y, color):
        scanner_offset = self._field_to_image(scanner_x, scanner_y)

        cv2.circle(self.image, scanner_offset, self.scanner_radius, color, 1)

        for angle in range(360):
            distance = data[angle]
            if distance > 0 and distance < self.max_range:
                radians = angle * pi / 180.0
                ping_x = (distance * cos(radians)) + scanner_x
                ping_y = (distance * sin(radians)) + scanner_y

                ping_image = self._field_to_image(ping_x, ping_y)
                image_x = min(ping_image[0], self.image_width - 1)
                image_y = min(ping_image[1], self.image_height - 1)

                self.image[image_y, image_x] = color
