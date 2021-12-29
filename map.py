from PIL import Image, ImageDraw
import math
import os
import pickle
import numpy as np

ROBOT_MARKER_RADIUS = 20
ROBOT_COLOR = (0xFF, 0xAE, 0x02)
ROBOT_ANGLE_INDICATOR_COLOR = (0xB9, 0x78, 0x00)
ROBOT_PING_COLOR = (0x0, 0x80, 0x00)
ROBOT_PING_POINT_RADIUS = 2

def load_frc2019_field():
    return Map('FRC2019 LiDAR Lines.png', 'FRC2019 LiDAR Lines.png.point')

def _get_file_path(filename):
    this_module_dir = os.path.dirname(os.path.abspath(__file__))
    path = os.path.join(this_module_dir, './maps/', filename)

    if not os.path.isfile(path):
        raise Exception(f'File does not exist: {path}')

    return path

class Robot:
    def __init__(self):
        self.location_x_mm = 0.0
        self.location_y_mm = 0.0
        self.heading_degrees = 0.0
        self.ping_points = None

    def set_position(self, location_x_mm, location_y_mm, heading_degrees):
        self.location_x_mm = location_x_mm
        self.location_y_mm = location_y_mm
        self.heading_degrees = heading_degrees

    def load_scan_file(self, filename):
        path = _get_file_path(filename)
        with open(path, 'rb') as f:
            data = pickle.load(f)
            self.ping_points = data['ping_points']

class Map:
    def __init__(self, image_filename, point_filename):
        self.image = self._load_image(image_filename)
        self.point_data = self._load_point_data(point_filename)

    def _load_image(self, filename):
        path = _get_file_path(filename)
        image = Image.open(path)
        image.load()
        return image

    def _load_point_data(self, filename):
        path = _get_file_path(filename)
        with open(path, 'rb') as f:
            return pickle.load(f)

    @property
    def mm_per_pixel(self):
        return self.point_data['mm_per_pixel']

    @property
    def center_x_pixel(self):
        return self.point_data['center_x_pixel']

    @property
    def center_y_pixel(self):
        return self.point_data['center_y_pixel']

    @property
    def wall_points(self):
        return self.point_data['wall_points']

    @property
    def width_mm(self):
        return self.image.width * self.mm_per_pixel

    @property
    def height_mm(self):
        return self.image.height * self.mm_per_pixel

    @property
    def size_mm(self):
        return (self.width_mm, self.height_mm)

    def coordinates_mm_to_px(self, coords_mm):
        ( x_mm, y_mm ) = coords_mm
        x_px = (x_mm / self.mm_per_pixel) + self.center_x_pixel
        y_px = (y_mm / self.mm_per_pixel) + self.center_y_pixel
        return (x_px, y_px)
    
    def draw_map_with_robot(self, robot):
        base_image = self.image.copy()
        draw = ImageDraw.Draw(base_image)

        ( robot_x_px, robot_y_px ) = self.coordinates_mm_to_px((robot.location_x_mm, robot.location_y_mm))

        radius = ROBOT_MARKER_RADIUS
        line_width = int(ROBOT_MARKER_RADIUS / 2)
        circle = [ robot_x_px - radius, robot_y_px - radius, robot_x_px + radius, robot_y_px + radius ]
        draw.ellipse(circle, fill = ROBOT_COLOR)

        tip_x = robot_x_px + (math.sin(math.radians(180 - robot.heading_degrees)) * radius * 1)
        tip_y = robot_y_px + (math.cos(math.radians(180 - robot.heading_degrees)) * radius * 1)
        draw.line([ robot_x_px, robot_y_px, tip_x, tip_y ], fill=ROBOT_ANGLE_INDICATOR_COLOR, width=line_width)

        if isinstance(robot.ping_points, np.ndarray):
            for index in range(0, len(robot.ping_points[0])):
                x_mm = robot.ping_points[0][index]
                y_mm = robot.ping_points[1][index]
                ( x_px, y_px ) = self.coordinates_mm_to_px((x_mm, y_mm))
                ping_circle = [
                    x_px - ROBOT_PING_POINT_RADIUS,
                    y_px - ROBOT_PING_POINT_RADIUS,
                    x_px + ROBOT_PING_POINT_RADIUS,
                    y_px + ROBOT_PING_POINT_RADIUS,
                ]
                draw.ellipse(ping_circle, fill=ROBOT_PING_COLOR)

        return base_image

    def draw_local_map(self, robot, range_radius_mm):
        location_x_offset = (robot.location_x_mm / self.mm_per_pixel)
        location_y_offset = (robot.location_y_mm / self.mm_per_pixel)
        location_x_px = location_x_offset + self.center_x_pixel
        location_y_px = location_y_offset + self.center_y_pixel
        range_radius_px = range_radius_mm / self.mm_per_pixel

        local_image = self.draw_map_with_robot(robot).rotate(
            robot.heading_degrees,
            center = (location_x_px, location_y_px),
            translate = (-location_x_offset, -location_y_offset),
            fillcolor = '#ddd',
        )

        left = self.center_x_pixel - range_radius_px
        right = self.center_x_pixel + range_radius_px
        top = self.center_y_pixel - range_radius_px
        bottom = self.center_y_pixel + range_radius_px
        local_image = local_image.crop((left, top, right, bottom))

        return local_image

