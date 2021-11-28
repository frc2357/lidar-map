from rplidar import RPLidar

def lidar_unit_process(conn, usb_tty, angle_offset, ignore_regions):
    lidar = LidarUnit(usb_tty, angle_offset, ignore_regions)

    print(f"LidarUnit [{usb_tty}]: entering scan loop")
    for _ in lidar.scan():
        for (angle, distance) in lidar.measurements():
            conn.send((False, angle, distance))
        conn.send((True, 0, 0))

    conn.close()

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
