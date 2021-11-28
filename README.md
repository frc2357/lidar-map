# lidar-map

LIDAR Mapping for FRC

This project's goal is to map a field position using one or more LIDAR units
to project the static field elements and then match the point plane with
the known static field layout, thus deriving the robot's position and heading.

## Environment

This project was designed to be run on a (WPILibPi)[https://github.com/wpilibsuite/WPILibPi] image on a Raspberry Pi 4.

## Setup

1. Set up WPILibPi as described in the docs: https://docs.wpilib.org/en/stable/docs/software/vision-processing/wpilibpi/what-you-need-to-get-the-pi-image-running.html

2. Browse to http://wpilibpi.local and click the "Writable" button at the top.

3. SSH into pi `ssh pi@wpilibpi.local` (default rpi password)

4. Run `sudo pip3 install rplidar-roboticia`

5. Upload each of the `*.py` files in this repo via the web interface individually under the "File Upload" section.

6. Select "Uploaded Python file" under the "Vision Application Configuration" section.

7. Upload `lidar-map.py` under the "Vision Application Configuration" section.
