from PIL import Image, ImageDraw
import math
import os

ROBOT_MARKER_RADIUS = 50
ROBOT_COLOR = "#FFAE02"
ROBOT_ANGLE_INDICATOR_COLOR = "#B97800"

def load_frc2019_field():
    center_point_pixels = (1571, 3001)
    mm_per_pixel = 2.77839
    return Map('FRC2019 LiDAR Lines.png', center_point_pixels, mm_per_pixel)

class Robot:
    def __init__(self):
        self.location_x_mm = 0.0
        self.location_y_mm = 0.0
        self.heading_degrees = 0.0

    def set_position(self, location_x_mm, location_y_mm, heading_degrees):
        self.location_x_mm = location_x_mm
        self.location_y_mm = location_y_mm
        self.heading_degrees = heading_degrees

class Map:
    def __init__(self, filename, center_point_pixels, mm_per_pixel):
        self.filename = filename
        self.center_point_pixels = center_point_pixels
        self.mm_per_pixel = mm_per_pixel
        self.image = self._load_image(filename)

    def _load_image(self, filename):
        this_module_dir = os.path.dirname(os.path.abspath(__file__))
        path = os.path.join(this_module_dir, './maps/', filename)

        if not os.path.isfile(path):
            raise Exception(f"File does not exist: {path}")

        image = Image.open(path)

        if image is None:
            raise Exception(f"Failed to load image {filename}")

        return image

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
        ( center_x_px, center_y_px ) = self.center_point_pixels
        x_px = (x_mm / self.mm_per_pixel) + center_x_px
        y_px = (y_mm / self.mm_per_pixel) + center_y_px
        return (x_px, y_px)
    
    def draw_map_with_robot(self, robot):
        fill = "#FFAE02"
        base_image = self.image.copy()
        draw = ImageDraw.Draw(base_image)

        ( robot_x_px, robot_y_px ) = self.coordinates_mm_to_px((robot.location_x_mm, robot.location_y_mm))

        radius = ROBOT_MARKER_RADIUS
        line_width = int(ROBOT_MARKER_RADIUS / 2)
        circle = [ robot_x_px - radius, robot_y_px - radius, robot_x_px + radius, robot_y_px + radius ]
        draw.ellipse(circle, fill = ROBOT_COLOR)

        tip_x = robot_x_px + (math.sin(math.radians(180 - robot.heading_degrees)) * radius * 1)
        tip_y = robot_y_px + (math.cos(math.radians(180 - robot.heading_degrees)) * radius * 1)
        draw.line([ robot_x_px, robot_y_px, tip_x, tip_y ], fill = ROBOT_ANGLE_INDICATOR_COLOR, width = line_width)

        return base_image

    def draw_local_map(self, robot, range_radius_mm):
        ( center_x_px, center_y_px ) = self.center_point_pixels
        location_x_offset = (robot.location_x_mm / self.mm_per_pixel)
        location_y_offset = (robot.location_y_mm / self.mm_per_pixel)
        location_x_px = location_x_offset + center_x_px
        location_y_px = location_y_offset + center_y_px
        range_radius_px = range_radius_mm / self.mm_per_pixel

        local_image = self.draw_map_with_robot(robot).rotate(
            robot.heading_degrees,
            center = (location_x_px, location_y_px),
            translate = (-location_x_offset, -location_y_offset),
            fillcolor = '#ddd',
        )

        left = center_x_px - range_radius_px
        right = center_x_px + range_radius_px
        top = center_y_px - range_radius_px
        bottom = center_y_px + range_radius_px
        local_image = local_image.crop((left, top, right, bottom))

        return local_image

