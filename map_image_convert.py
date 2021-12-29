import sys
import os
import argparse
import pickle
import numpy as np
from PIL import Image

MM_PER_FOOT = 304.8

COLOR_WALL = (0x00, 0x00, 0x00)
COLOR_PING = (0x00, 0xFF, 0x00)

def convert_coord_tuples_to_nparray(point_tuples):
    nparray = np.zeros((2, len(point_tuples)))
    for index in range(0, len(point_tuples)):
        (x, y) = point_tuples[index]
        nparray[0, index] = x
        nparray[1, index] = y
    return nparray

def convert_image_file(image_filename, center_x_pixel, center_y_pixel, mm_per_pixel):
    point_filename = image_filename + '.point'
    print(f'Converting "{image_filename}" to point file "{point_filename}"')

    with Image.open(image_filename) as image:
        image.load()

        width_mm = image.width * mm_per_pixel
        height_mm = image.height * mm_per_pixel
        center_x_mm = center_x_pixel * mm_per_pixel
        center_y_mm = center_y_pixel * mm_per_pixel
        print(f'mm_per_pixel: {mm_per_pixel}')
        print(f'image: {image.width}px x {image.height}px')
        print(f'center: {center_x_pixel},{center_y_pixel}, mm_per_pixel: {mm_per_pixel}')
        print(f'map: {width_mm}mm x {height_mm}mm ({width_mm / MM_PER_FOOT}ft x {height_mm / MM_PER_FOOT}ft)')
        print(f'map_center: {center_x_mm}mm x {center_y_mm}mm ({center_x_mm / MM_PER_FOOT}ft x {center_y_mm / MM_PER_FOOT}ft)')

        with open(point_filename, 'wb') as point_file:
            wall_points = []
            ping_points = []

            for y_pixel in range(0, image.height):
                for x_pixel in range(0, image.width):
                    pixel_color = image.getpixel((x_pixel, y_pixel))

                    x_mm = (x_pixel - center_x_pixel) * mm_per_pixel
                    y_mm = (center_y_pixel - y_pixel) * mm_per_pixel

                    if (pixel_color == COLOR_WALL):
                        wall_points.append((x_mm, y_mm))
                    elif (pixel_color == COLOR_PING):
                        ping_points.append((x_mm, y_mm))

            point_data = {
                'center_x_pixel': center_x_pixel,
                'center_y_pixel': center_y_pixel,
                'mm_per_pixel': mm_per_pixel,
            }

            if len(wall_points) > 0:
                point_data['wall_points'] = convert_coord_tuples_to_nparray(wall_points)
                print(f'{len(wall_points)} wall points')

            if len(ping_points) > 0:
                point_data['ping_points'] = convert_coord_tuples_to_nparray(ping_points)
                print(f'{len(ping_points)} ping points')

            pickle.dump(point_data, point_file)

def main():
    desc = "Converts image map data to lidar point data.\n"

    parser = argparse.ArgumentParser(description = desc)
    parser.add_argument('image_file', help='The lidar image file')
    parser.add_argument(
        '-cx',
        '--center-x',
        dest='center_x',
        required=True,
        type=int,
        help='x-coordinate for center of map image'
    )
    parser.add_argument(
        '-cy',
        '--center-y',
        dest='center_y',
        required=True,
        type=int,
        help='y-coordinate for center of map image'
    )
    parser.add_argument(
        '-mm',
        '--mm-per-pixel',
        dest='mm_per_pixel',
        required=True,
        type=float,
        help='number of millimeters represented by length of a pixel'
    )

    args = parser.parse_args()

    convert_image_file(args.image_file, args.center_x, args.center_y, args.mm_per_pixel)

if __name__ == '__main__':
    main()
