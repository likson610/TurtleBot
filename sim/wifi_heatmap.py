"""WiFi heatmap node: RSSI measurement + robot pose from SLAM -> /wifi/heatmap.

source:=sim  - RSSI computed from the model in wifi_model.py (virtual AP in the Gazebo world),
source:=scan - RSSI from /wifi/scan (wifi_msgs/WiFiScan from src/wifi_poc, needs a built wifi_msgs).

locate_ap:=true - estimates the AP position from the collected RSSI (wifi_model.locate_ap),
publishes it with a 2-sigma ellipse on /wifi/ap_estimate and logs the error against the true AP
(sim only). In sim the fit also models the walls from the .sdf world.

On exit (Ctrl+C) the measurements are saved as CSV and PNG to output_dir.
"""
import csv
import math
import threading
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
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException

from wifi_model import HeatmapGrid, RssiModel, error_ellipse, load_walls, locate_ap

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
        self.walls_map = []  # walls in the map frame, used by the AP localisation

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
            self.walls_map = [(cx - self.spawn[0], cy - self.spawn[1], yaw, hx, hy)
                              for cx, cy, yaw, hx, hy in walls]
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

        # --- AP localisation ---
        self.locate = p('locate_ap', False).value
        self.estimate = None
        self.estimate_history = []  # (stamp, cells, x, y, tx_power, n, rmse, sigma_major, error)
        self.locating = threading.Lock()
        if self.locate:
            if not p('locate_walls', True).value:
                self.walls_map = []
            self.locate_wall_loss = p('locate_wall_loss', 5.0).value
            self.locate_min_cells = p('locate_min_cells', 15).value
            self.estimate_pub = self.create_publisher(MarkerArray, '/wifi/ap_estimate', LATCHED)
            self.create_timer(p('locate_period', 5.0).value, self.start_locate)
            self.get_logger().info(
                f'WiFi: lokalizacja AP włączona (model ścian: {len(self.walls_map)} przeszkód, '
                f'tłumienie {self.locate_wall_loss:.1f} dB).')

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

    # --- AP localisation ---

    def start_locate(self):
        '''Runs the fit in a worker thread, so that sampling is not blocked by it.'''
        if not self.locating.acquire(blocking=False):
            return
        x, y, rssi = self.grid.cell_points()
        if len(rssi) < self.locate_min_cells or max(np.ptp(x), np.ptp(y)) < 1.0:
            self.locating.release()
            return
        threading.Thread(target=self.locate_worker, args=(x, y, rssi), daemon=True).start()

    def locate_worker(self, x, y, rssi):
        try:
            estimate = locate_ap(x, y, rssi, self.walls_map, self.locate_wall_loss)
            if estimate is None:
                return
            self.estimate = estimate
            sigma_major = error_ellipse(estimate['cov'], sigma=1.0)[0] / 2
            error = None
            if self.source == 'sim':
                ap_x, ap_y = self.ap_map()
                error = math.hypot(estimate['x'] - ap_x, estimate['y'] - ap_y)
            self.estimate_history.append((
                self.get_clock().now().nanoseconds / 1e9, estimate['samples'], estimate['x'], estimate['y'],
                estimate['tx_power'], estimate['path_loss_exp'], estimate['rmse'], sigma_major, error))
            self.publish_estimate(estimate, error)
            self.get_logger().info(
                f"WiFi: AP ~ ({estimate['x']:.2f}, {estimate['y']:.2f}) map, ±{sigma_major:.2f} m (1 sigma), "
                f"P0={estimate['tx_power']:.1f} dBm, n={estimate['path_loss_exp']:.2f}, "
                f"RMSE={estimate['rmse']:.1f} dB, {estimate['samples']} pól"
                + (f', błąd {error:.2f} m' if error is not None else ''))
        except Exception as exc:  # a failed fit must not kill the node
            self.get_logger().warn(f'WiFi: lokalizacja AP nie powiodła się: {exc}')
        finally:
            self.locating.release()

    def publish_estimate(self, estimate, error):
        stamp = self.get_clock().now().to_msg()
        width, height, angle = error_ellipse(estimate['cov'])
        markers = []
        for i, kind in enumerate((Marker.SPHERE, Marker.CYLINDER, Marker.TEXT_VIEW_FACING)):
            m = Marker()
            m.header.frame_id, m.header.stamp = 'map', stamp
            m.ns, m.id, m.type, m.action = 'wifi_ap_estimate', i, kind, Marker.ADD
            m.pose.position.x, m.pose.position.y = estimate['x'], estimate['y']
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.0, 1.0, 1.0
            markers.append(m)
        sphere, ellipse, text = markers
        sphere.pose.position.z = 0.3
        sphere.pose.orientation.w = 1.0
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.2
        # 2-sigma ellipse as a flat cylinder rotated about z
        ellipse.pose.position.z = 0.02
        ellipse.pose.orientation.z, ellipse.pose.orientation.w = math.sin(angle / 2), math.cos(angle / 2)
        ellipse.scale.x, ellipse.scale.y, ellipse.scale.z = max(width, 0.05), max(height, 0.05), 0.02
        ellipse.color.a = 0.35
        text.pose.position.z = 0.7
        text.pose.orientation.w = 1.0
        text.scale.z = 0.18
        text.color.r = text.color.g = text.color.b = 1.0
        text.text = 'AP?' + (f' {error:.2f} m' if error is not None else '')
        self.estimate_pub.publish(MarkerArray(markers=markers))

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
        if self.locate:
            with self.locating:  # wait for a running fit, then make a final one on all data
                x, y, rssi = self.grid.cell_points()
                final = locate_ap(x, y, rssi, self.walls_map, self.locate_wall_loss) if len(rssi) >= 5 else None
                self.estimate = final or self.estimate
            self.save_estimate_history(base.parent / f'{base.name}_ap.csv', ap)
        png = self.grid.save_png(base.with_suffix('.png'), self.slam_map, ap, self.rssi_min, self.rssi_max,
                                 self.estimate if self.locate else None)
        rssi = [s[3] for s in self.grid.samples]
        self.get_logger().info(
            f'WiFi: {len(rssi)} pomiarów w {len(self.grid.sums)} polach, RSSI {min(rssi):.0f}..{max(rssi):.0f} dBm. '
            f'Zapisano {base}.csv' + (' i .png' if png else ' (PNG wymaga python3-matplotlib)'))

    def save_estimate_history(self, path, ap):
        '''History of AP estimates (how the error falls as the robot collects data) + final estimate.'''
        e = self.estimate
        if e is None:
            self.get_logger().info('WiFi: za mało danych do lokalizacji AP.')
            return
        error = math.hypot(e['x'] - ap[0], e['y'] - ap[1]) if ap is not None else None
        sigma_major = error_ellipse(e['cov'], sigma=1.0)[0] / 2
        with open(path, 'w', newline='', encoding='utf-8') as file:
            writer = csv.writer(file)
            writer.writerow(['stamp_s', 'cells', 'x_m', 'y_m', 'tx_power_dbm', 'path_loss_exp',
                             'rmse_db', 'sigma_major_m', 'error_m'])
            rows = self.estimate_history + [(self.get_clock().now().nanoseconds / 1e9, e['samples'], e['x'], e['y'],
                                             e['tx_power'], e['path_loss_exp'], e['rmse'], sigma_major, error)]
            for row in rows:
                writer.writerow([f'{v:.3f}' if isinstance(v, float) else ('' if v is None else v) for v in row])
        self.get_logger().info(
            f"WiFi: końcowa estymacja AP ({e['x']:.2f}, {e['y']:.2f}) map, ±{sigma_major:.2f} m (1 sigma)"
            + (f', błąd {error:.2f} m' if error is not None else '') + f'. Historia: {path}')


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
