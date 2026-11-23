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

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.sub_scan = self.create_subscription(WiFiScan, '/wifi/scan', self.scan_callback, 10)
        self.pub_grid = self.create_publisher(OccupancyGrid, '/wifi/heatmap', 1)

        self.update_timer = self.create_timer(3.0, self.generate_heatmap)

    def scan_callback(self, msg: WiFiScan):
        chosen_rssi = None
        for net in msg.networks:
            if self.target_bssid and net.bssid.lower() == self.target_bssid.lower():
                chosen_rssi = net.rssi_dbm
                break
            elif self.target_ssid and net.ssid == self.target_ssid:
                chosen_rssi = net.rssi_dbm
                break

        if chosen_rssi is None and len(msg.networks) > 0 and not self.target_ssid and not self.target_bssid:
            chosen_rssi = max(net.rssi_dbm for net in msg.networks)

        if chosen_rssi is None:
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                msg.header.frame_id,
                msg.header.stamp,
                timeout=Duration(seconds=0.3)
            )
            x = transform.transform.translation.x
            y = transform.transform.translation.y

            if len(self.measurements) > 0:
                last_x, last_y, _ = self.measurements[-1]
                if np.hypot(x - last_x, y - last_y) < 0.10:
                    return

            self.measurements.append((x, y, chosen_rssi))

        except tf2_ros.TransformException as ex:
            return


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