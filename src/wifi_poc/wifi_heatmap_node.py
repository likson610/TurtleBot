import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import numpy as np
from scipy.interpolate import Rbf

import tf2_ros
from nav_msgs.msg import OccupancyGrid
from wifi_msgs.msg import WiFiScan

class WiFiHeatmapNode(Node):
    def __init__(self):
        super().__init__('wifi_heatmap_node')

        self.declare_parameter('target_ssid', 'test')
        self.declare_parameter('target_bssid', '')
        self.declare_parameter('resolution', 0.2)
        self.declare_parameter('min_rssi', -95.0)
        self.declare_parameter('max_rssi', -30.0)
        self.declare_parameter('map_frame', 'map')

        self.target_ssid = self.get_parameter('target_ssid').value
        self.target_bssid = self.get_parameter('target_bssid').value
        self.resolution = self.get_parameter('resolution').value
        self.min_rssi = self.get_parameter('min_rssi').value
        self.max_rssi = self.get_parameter('max_rssi').value
        self.map_frame = self.get_parameter('map_frame').value

        self.measurements = []

def main(args=None):
    rclpy.init(args=args)
    node = WiFiHeatmapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()