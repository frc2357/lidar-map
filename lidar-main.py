#!/usr/bin/env python3

import cv2
import json
import os
import sys
import time

from lidarunit import LidarUnitProcess
from lidarimage import LidarImage
from cscore import CameraServer, MjpegServer
from networktables import NetworkTablesInstance

LIDAR_UNIT_1_PORT = '/dev/ttyUSB0'
LIDAR_UNIT_2_PORT = '/dev/ttyUSB1'

IMAGE_OUTPUT_WIDTH = 640
IMAGE_OUTPUT_HEIGHT = 480

## Standard Robot (29in x 30in) on field
#ROBOT_WIDTH = 737 # 29 inches
#ROBOT_LENGTH = 762 # 30 inches
#MAX_RANGE = 8230 # 27 feet (1/2 length of FRC field)

## Test Platform (310mm x 225mm) in smaller room
ROBOT_WIDTH = 310
ROBOT_LENGTH = 225
MAX_RANGE = 4572 # 15 feet

LIDAR1_COLOR = (100, 255, 100)
LIDAR1_X = ((ROBOT_WIDTH / 2) - 40) # 30 cm inside robot perimiter
LIDAR1_Y = -((ROBOT_LENGTH / 2) - 40) # 30 cm inside robot perimiter

lidar1 = LidarUnitProcess(
    '/dev/ttyUSB0',        # lidar usb device
    -90,                   # angle offset
    [ [180, 270] ]         # ignore regions
)

LIDAR2_COLOR = (255, 100, 100)
LIDAR2_X = -((ROBOT_WIDTH / 2) - 40) # 30 cm inside robot perimiter
LIDAR2_Y = ((ROBOT_LENGTH / 2) - 40) # 30 cm inside robot perimiter

# Include the root path of this project for imports below
sys.path.append(__file__[0:__file__.rfind('/')] + '/.')

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

    # start cameras
    cs = CameraServer.getInstance()
    cs.enableLogging()

    outputStream = cs.putVideo("VideoStream", IMAGE_OUTPUT_WIDTH, IMAGE_OUTPUT_HEIGHT)

    lidar1.start()

    # loop forever
    print("Main: Entering main loop")
    while True:
        if (lidar1.update()):
            lidarImage.clear()
            lidarImage.drawRobot()
            lidarImage.drawScan(lidar1.scan_data, LIDAR1_X, LIDAR1_Y, LIDAR1_COLOR)

        outputStream.putFrame(lidarImage.image)
