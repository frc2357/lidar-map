from math import floor
from multiprocessing import Process, Pipe
from time import sleep
import numpy as np
import signal, os

from rplidar import RPLidar, RPLidarException

CRASH_SLEEP_TIME_SECONDS = 3

class LidarUnitProcess:
    def __init__(self, usb_tty, angle_offset, ignore_regions):
        self.usb_tty = usb_tty
        self.angle_offset = angle_offset
        self.ignore_regions = ignore_regions
        self.conn = None
        self.process = None
        self.scan_data = np.zeros(360)
        self.scan_ping_count = 0
        self.current_index = 0 

    def start(self):
        self.scan_data[:] = 0
        self.last_index = -1 

        parent_conn, child_conn = Pipe()
        self.conn = parent_conn
        self.process = Process(target=_lidar_unit_process, args=(
            child_conn,
            self.usb_tty,
            self.angle_offset,
            self.ignore_regions
        ))
        self.process.start()

    def is_running(self):
        return self.process != None

    def update(self):
        if (not self.conn.poll()):
            return

        is_complete, angle, distance = self.conn.recv()

        if is_complete:
            print(f"LidarUnit [{self.usb_tty}]: scanned {self.scan_ping_count} points")
            self.scan_ping_count = 0
            return

        ping_index = int(min(floor(angle), 359))
        index = self._increment_index(self.current_index)

        # Only move up in degrees, never down.
        # If a ping reads a previous angle, ignore it
        if self._degrees_difference(index, ping_index) < 0:
            return

        # Clear out any skipped angle steps
        while index != ping_index:
            self.scan_data[index] = 0
            index = self._increment_index(index)

        self.current_index = ping_index
        self.scan_data[self.current_index] = distance
        self.scan_ping_count += 1


    def _degrees_difference(self, a, b):
        degrees = b - a if b >= a else (b + 360) - a
        if degrees >= 180:
            degrees -= 360

        return degrees

    def _increment_index(self, index):
        return index + 1 if index < 359 else 0


def _lidar_unit_process(conn, usb_tty, angle_offset, ignore_regions):
    lidar = LidarUnit(usb_tty, angle_offset, ignore_regions)

    def termination_handler(signum, frame):
        print(f"LidarUnit [{usb_tty}]: Terminated with signal {str(signum)}")
        lidar.stop()

    signal.signal(signal.SIGHUP, termination_handler)
    signal.signal(signal.SIGINT, termination_handler)
    signal.signal(signal.SIGTERM, termination_handler)

    while True:
        print(f"LidarUnit [{usb_tty}]: Initializing")
        try:
            lidar.initialize()
            lidar.stop()
        except RPLidarException as e:
            print(f"LidarUnit [{usb_tty}]: Exception on init: {str(e)}")
            sleep(CRASH_SLEEP_TIME_SECONDS)
            continue

        sleep(5)

        print(f"LidarUnit [{usb_tty}]: entering scan loop")
        try:
            for _ in lidar.scan():
                for (angle, distance) in lidar.measurements():
                    conn.send((False, angle, distance))
                conn.send((True, 0, 0))
        except RPLidarException as e:
            print(f"LidarUnit [{usb_tty}]: Exception during scan: {str(e)}")
            sleep(CRASH_SLEEP_TIME_SECONDS)
            continue

    conn.close()


class LidarUnit:
    def __init__(self, usb_tty, angle_offset, ignore_regions):
        self.usb_tty = usb_tty
        self.lidar = RPLidar(self.usb_tty)
        self.angle_offset = angle_offset
        self.ignore_regions = ignore_regions

    def initialize(self):
        self.lidar.clean_input()
        info = self.lidar.get_info()
        health = self.lidar.get_health()
        print(f"LidarUnit [{self.usb_tty}]: {info} ({health})")
    
    def stop(self):
        self.lidar.stop_motor()

    def _is_ignored(self, angle):
        for (_, region) in enumerate(self.ignore_regions):
            if (angle > region[0] and angle < region[1]):
                return True
        return False

    def scan(self):
        self.lidar.start_motor()
        for i, scan in enumerate(self.lidar.iter_scans(min_len=1)):
            self.scan = scan
            yield self

    def measurements(self):
        for (_, angle, distance) in self.scan:
            if (not self._is_ignored(angle)):
                angle += self.angle_offset
                if (angle < 0):
                    angle += 360

                yield (angle, distance)
