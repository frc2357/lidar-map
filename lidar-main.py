#!/usr/bin/env python3

import cv2
import json
import os
import sys
from time import sleep, time_ns

# Include the root path of this project for imports below
sys.path.append(__file__[0:__file__.rfind('/')] + '/.')

from lidarunit import LidarUnitProcess
from lidarimage import LidarImage
from cscore import CameraServer, MjpegServer
from networktables import NetworkTablesInstance

LIDAR_UNIT_1_PORT = '/dev/ttyUSB0'
LIDAR_UNIT_2_PORT = '/dev/ttyUSB1'

IMAGE_OUTPUT_WIDTH = 640
IMAGE_OUTPUT_HEIGHT = 480

FRAME_MS = (1000 / 30)
LOOP_ITERATION_WARNING_MS = 15

## Standard Robot (29in x 30in) on field
#ROBOT_WIDTH = 737 # 29 inches
#ROBOT_LENGTH = 762 # 30 inches
#MAX_RANGE = 8230 # 27 feet (1/2 length of FRC field)

## Test Platform (310mm x 225mm) in smaller room
ROBOT_WIDTH = 736.6
ROBOT_LENGTH = 762
MAX_RANGE = 7924 # 15 feet

LIDAR1_COLOR = (50, 255, 50)
LIDAR1_X = ((ROBOT_WIDTH / 2) - 52) # 52 mm inside robot perimiter
LIDAR1_Y = -((ROBOT_LENGTH / 2) - 49) # 49 mm inside robot perimiter

LIDAR2_COLOR = (255, 50, 50)
LIDAR2_X = -((ROBOT_WIDTH / 2) - 40) # 30 mm inside robot perimiter
LIDAR2_Y = ((ROBOT_LENGTH / 2) - 40) # 30 mm inside robot perimiter

lidar_units = [
    LidarUnitProcess(
        '/dev/ttyUSB0',        # lidar usb device
        -90,                   # angle offset
        [  ]         # ignore regions
    ),
    # LidarUnitProcess(
    #     '/dev/ttyUSB1',        # lidar usb device
    #     90,                    # angle offset
    #     [ [180, 300] ]         # ignore regions
    # ),
]

lidar_unit_offsets = [
    (LIDAR1_X, LIDAR1_Y),
    (LIDAR2_X, LIDAR2_Y),
]

lidar_unit_colors = [ LIDAR1_COLOR, LIDAR2_COLOR ]

configFile = "/boot/frc.json"

team = None
server = False

def parseError(str):
    """Report parse error."""
    print("Main: config error in '" + configFile + "': " + str, file=sys.stderr)

def readConfig():
    """Read configuration file."""
    global team
    global server

    # parse file
    try:
        with open(configFile, "rt", encoding="utf-8") as f:
            j = json.load(f)
    except OSError as err:
        print("Main: could not open '{}': {}".format(configFile, err), file=sys.stderr)
        return False

    # top level must be an object
    if not isinstance(j, dict):
        parseError("must be JSON object")
        return False

    # team number
    try:
        team = j["team"]
    except KeyError:
        parseError("could not read team number")
        return False

    # ntmode (optional)
    if "ntmode" in j:
        str = j["ntmode"]
        if str.lower() == "client":
            server = False
        elif str.lower() == "server":
            server = True
        else:
            parseError("could not understand ntmode value '{}'".format(str))

    return True


lidarImage = LidarImage(ROBOT_WIDTH, ROBOT_LENGTH, MAX_RANGE)

if __name__ == "__main__":
    if len(sys.argv) >= 2:
        configFile = sys.argv[1]

    # read configuration
    if not readConfig():
        sys.exit(1)

    # start NetworkTables
    ntinst = NetworkTablesInstance.getDefault()
    if server:
        print("Main: Setting up NetworkTables server")
        ntinst.startServer()
    else:
        print("Main: Setting up NetworkTables client for team {}".format(team))
        ntinst.startClientTeam(team)
        ntinst.startDSClient()

    print("Main: Initializing Lidar Units")
    for unit in lidar_units:
        unit.start()

    print("Main: Starting in 5 seconds")
    sleep(5)

    # start cameras
    cs = CameraServer.getInstance()
    cs.enableLogging()

    outputStream = cs.putVideo("VideoStream", IMAGE_OUTPUT_WIDTH, IMAGE_OUTPUT_HEIGHT)
    last_frame_ms = 0

    # loop forever
    print("Main: Entering main loop")
    while True:
        start_iteration_ms = time_ns() / 1000000

        for unit in lidar_units:
            try:
                unit.update()
            except EOFError as e:
                print(f"Main: EOF on {unit.usb_tty} process. Restarting.")
                unit.start()

        if start_iteration_ms - last_frame_ms > FRAME_MS:
            lidarImage.clear()
            lidarImage.drawRobot()

            for index, unit in enumerate(lidar_units):
               offset_x, offset_y = lidar_unit_offsets[index]
               color = lidar_unit_colors[index]
               lidarImage.drawScan(unit.scan_data, offset_x, offset_y, color)

            outputStream.putFrame(lidarImage.image)
            last_frame_ms = start_iteration_ms

        iteration_time_ms = (time_ns() / 1000000) - start_iteration_ms
        if iteration_time_ms > LOOP_ITERATION_WARNING_MS:
            print(f"Main: WARNING, iteration took {int(iteration_time_ms)}ms")
