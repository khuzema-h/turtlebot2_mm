import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class MultiSectorFilter(Node):
    def __init__(self):
        super().__init__('filter_node')
        self.sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.pub = self.create_publisher(LaserScan, '/filtered_scan', 10)
        # Define your allowed sectors as a list of (min_angle, max_angle)
        # e.g. in radians, relative to the scan frame:
        self.allowed_sectors = [
            (-math.radians(60), math.radians(60)),   # front ±60°
            (math.radians(90-15), math.radians(90+15)),   # right ±15° around 90° (east)
            (math.radians(270-15), math.radians(270+15)), # left ±15° around 270° (west)
        ]

    def is_in_allowed(self, theta):
        # Normalize angle to [0, 2π) or (–π, π), whichever you prefer
        # Here assume theta in [–π, +π]
        for (amin, amax) in self.allowed_sectors:
            if amin <= theta <= amax:
                return True
        return False

    def scan_callback(self, scan: LaserScan):
        # Compute angle for each measurement
        n = len(scan.ranges)
        new_ranges = list(scan.ranges)
        for i in range(n):
            theta = scan.angle_min + i * scan.angle_increment
            if not self.is_in_allowed(theta):
                new_ranges[i] = float('inf')  # or some sentinel
        scan.ranges = new_ranges
        self.pub.publish(scan)

def main(args=None):
    rclpy.init(args=args)
    node = MultiSectorFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()