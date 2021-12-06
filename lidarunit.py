from math import floor
from multiprocessing import Process, Pipe
import numpy as np
from rplidar import RPLidar

def _lidar_unit_process(conn, usb_tty, angle_offset, ignore_regions):
    lidar = LidarUnit(usb_tty, angle_offset, ignore_regions)

    print(f"LidarUnit [{usb_tty}]: entering scan loop")
    for _ in lidar.scan():
        for (angle, distance) in lidar.measurements():
            conn.send((False, angle, distance))
        conn.send((True, 0, 0))

    conn.close()

class LidarUnitProcess:
    def __init__(self, usb_tty, angle_offset, ignore_regions):
        self.usb_tty = usb_tty
        self.angle_offset = angle_offset
        self.ignore_regions = ignore_regions
        self.conn = None
        self.process = None
        self.scan_data = np.zeros(360)
        self.scan_ping_count = 0
        self.is_complete = False

    def start(self):
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
        if (self.is_complete):
            # The last update completed the last scan. Reset to start next scan.
            self.scan_data[:] = 0
            self.scan_ping_count = 0
            self.is_complete = False

        if (self.conn.poll()):
            is_complete, angle, distance = self.conn.recv()

            if is_complete:
                print(f"lidar ping count: {self.scan_ping_count}")
                self.is_complete = True
                return True
            else:
                self.scan_data[int(floor(angle))] = distance
                self.scan_ping_count += 1

class LidarUnit:
    def __init__(self, usb_tty, angle_offset, ignore_regions):
        print(f"LidarUnit [{usb_tty}]: Initializing")
        self.usb_tty = usb_tty
        self.lidar = RPLidar(self.usb_tty)
        self.angle_offset = angle_offset
        self.ignore_regions = ignore_regions

        info = self.lidar.get_info()
        health = self.lidar.get_health()
        print(f"LidarUnit [{self.usb_tty}]: {info} ({health})")

    def _is_ignored(self, angle):
        for (_, region) in enumerate(self.ignore_regions):
            if (angle > region[0] and angle < region[1]):
                return True
        return False

    def scan(self):
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
