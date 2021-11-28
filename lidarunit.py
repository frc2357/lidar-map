from rplidar import RPLidar

class LidarUnit:
    def __init__(self, usb_tty, angle_offset, ignore_regions):
        print(f"Initializing RPLidar at '{usb_tty}'")
        self.usb_tty = usb_tty
        self.lidar = RPLidar(usb_tty)
        self.angle_offset = angle_offset
        self.ignore_regions = ignore_regions

        info = self.lidar.get_info()
        health = self.lidar.get_health()
        print(f"RPLidar {self.usb_tty}: {info} ({health})")

    def _is_ignored(self, angle):
        for (_, region) in enumerate(self.ignore_regions):
            if (angle > region[0] and angle < region[1]):
                return True
        return False

    def scan(self):
        for i, scan in enumerate(self.lidar.iter_scans(min_len=1)):
            print(f"RPLidar {self.usb_tty}: [{i}] {len(scan)} measurements")
            self.scan = scan
            yield self

    def measurements(self):
        for (_, angle, distance) in self.scan:
            if (not self._is_ignored(angle)):
                yield (angle + self.angle_offset, distance)
