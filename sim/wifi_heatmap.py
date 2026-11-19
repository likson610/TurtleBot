"""WiFi heatmap node: RSSI measurement + robot pose from SLAM -> /wifi/heatmap.

source:=sim  - RSSI computed from the model in wifi_model.py (virtual AP in the Gazebo world),
source:=scan - RSSI from /wifi/scan (wifi_msgs/WiFiScan from src/wifi_poc, needs a built wifi_msgs).

On exit (Ctrl+C) the measurements are saved as CSV and PNG to output_dir.
"""
from datetime import datetime
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException

from wifi_model import HeatmapGrid, RssiModel, load_walls

HERE = Path(__file__).resolve().parent
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class WifiHeatmap(Node):
    def __init__(self):
        super().__init__('wifi_heatmap_node')
        p = self.declare_parameter
        self.source = p('source', 'sim').value
        self.base_frame = p('base_frame', 'base_footprint').value
        self.pose_timeout = p('pose_timeout', 1.0).value
        self.rssi_min = p('rssi_min', -90.0).value
        self.rssi_max = p('rssi_max', -40.0).value
        self.output_dir = Path(p('output_dir', str(HERE / 'wifi_results')).value)
        # world (Gazebo) -> map: SLAM starts its map at the spawn pose of the robot
        self.spawn = (p('spawn_x', 0.0).value, p('spawn_y', 0.0).value)
        self.ap_world = (p('ap_x', 2.5).value, p('ap_y', 2.0).value)
        self.grid = HeatmapGrid(p('resolution', 0.2).value)
        self.slam_map = None
        self.last_rssi = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, LATCHED)
        self.heatmap_pub = self.create_publisher(OccupancyGrid, '/wifi/heatmap', LATCHED)
        self.rssi_pub = self.create_publisher(Float32, '/wifi/rssi', 10)
        self.ap_pub = self.create_publisher(Marker, '/wifi/ap', LATCHED)

        if self.source == 'sim':
            world = p('world', str(HERE / 'room.sdf')).value
            walls = load_walls(world)
            seed = p('seed', -1).value
            self.model = RssiModel(
                self.ap_world, walls,
                tx_power=p('tx_power', -40.0).value, path_loss_exp=p('path_loss_exp', 2.0).value,
                wall_loss=p('wall_loss', 5.0).value, noise_std=p('noise_std', 2.0).value,
                seed=None if seed < 0 else seed)
            self.create_timer(p('sample_period', 1.0).value, self.sample_sim)
            self.get_logger().info(
                f'WiFi (symulacja): AP w ({self.ap_world[0]:.2f}, {self.ap_world[1]:.2f}) świata, '
                f'{len(walls)} przeszkód z {Path(world).name}.')
        elif self.source == 'scan':
            from wifi_msgs.msg import WiFiScan  # built from src/wifi_poc/wifi_msgs
            self.bssid = p('bssid', '').value.lower()
            self.create_subscription(WiFiScan, '/wifi/scan', self.scan_callback, 10)
            self.get_logger().info(f'WiFi (skaner): /wifi/scan, BSSID: {self.bssid or "najsilniejszy"}.')
        else:
            raise ValueError(f"source musi być 'sim' albo 'scan', a jest '{self.source}'")

        self.create_timer(2.0, self.publish_heatmap)
        self.publish_ap_marker()

    # --- inputs ---

    def map_callback(self, msg):
        info = msg.info
        data = np.array(msg.data, dtype=np.int16).reshape((info.height, info.width))
        self.slam_map = (data, info.resolution, info.origin.position.x, info.origin.position.y)

    def robot_xy(self, stamp=None):
        '''Robot position in the map frame, or None when TF is missing or stale.'''
        try:
            t = self.tf_buffer.lookup_transform('map', self.base_frame, stamp or Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None
        age = (self.get_clock().now() - Time.from_msg(t.header.stamp)).nanoseconds / 1e9
        if not -0.1 <= age <= self.pose_timeout:
            return None
        return t.transform.translation.x, t.transform.translation.y

    def sample_sim(self):
        pose = self.robot_xy()
        if pose is None:
            return
        x, y = pose
        rssi = self.model.measure(x + self.spawn[0], y + self.spawn[1])
        self.record(x, y, rssi)

    def scan_callback(self, msg):
        networks = [n for n in msg.networks if not self.bssid or n.bssid.lower() == self.bssid]
        if not networks:
            return
        pose = self.robot_xy(Time.from_msg(msg.header.stamp)) or self.robot_xy()
        if pose is None:
            return
        self.record(*pose, max(n.rssi_dbm for n in networks))

    def record(self, x, y, rssi):
        stamp = self.get_clock().now().nanoseconds / 1e9
        self.grid.add(x, y, rssi, stamp)
        self.last_rssi = rssi
        self.rssi_pub.publish(Float32(data=float(rssi)))

    # --- outputs ---

    def publish_heatmap(self):
        bounds = None
        if self.slam_map is not None:
            data, res, ox, oy = self.slam_map
            bounds = (ox, oy, ox + data.shape[1] * res - 1e-6, oy + data.shape[0] * res - 1e-6)
        payload = self.grid.to_occupancy(self.rssi_min, self.rssi_max, bounds)
        if payload is None:
            return
        ox, oy, width, height, cells = payload
        msg = OccupancyGrid()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = self.grid.resolution
        msg.info.width, msg.info.height = width, height
        msg.info.origin.position.x, msg.info.origin.position.y = ox, oy
        msg.info.origin.orientation.w = 1.0
        msg.data = cells
        self.heatmap_pub.publish(msg)

    def publish_ap_marker(self):
        if self.source != 'sim':
            return
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.ns, marker.id = 'wifi_ap', 0
        marker.type, marker.action = Marker.SPHERE, Marker.ADD
        marker.pose.position.x, marker.pose.position.y = self.ap_map()
        marker.pose.position.z = 0.3
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.25
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 1.0, 1.0, 1.0
        self.ap_pub.publish(marker)

    def ap_map(self):
        return self.ap_world[0] - self.spawn[0], self.ap_world[1] - self.spawn[1]

    def save_results(self):
        if not self.grid.samples:
            self.get_logger().info('WiFi: brak pomiarów, nic nie zapisano.')
            return
        self.output_dir.mkdir(parents=True, exist_ok=True)
        base = self.output_dir / f'wifi_{datetime.now():%Y%m%d_%H%M%S}'
        self.grid.save_csv(base.with_suffix('.csv'))
        ap = self.ap_map() if self.source == 'sim' else None
        png = self.grid.save_png(base.with_suffix('.png'), self.slam_map, ap, self.rssi_min, self.rssi_max)
        rssi = [s[3] for s in self.grid.samples]
        self.get_logger().info(
            f'WiFi: {len(rssi)} pomiarów w {len(self.grid.sums)} polach, RSSI {min(rssi):.0f}..{max(rssi):.0f} dBm. '
            f'Zapisano {base}.csv' + (' i .png' if png else ' (PNG wymaga python3-matplotlib)'))


def main(args=None):
    # Own SIGINT handling keeps the context alive, so results can be saved after Ctrl+C.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = WifiHeatmap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.save_results()
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()
