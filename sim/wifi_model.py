"""WiFi RSSI model and heatmap grid (no ROS dependencies, testable on its own).

- load_walls(): reads box obstacles from the Gazebo world (.sdf),
- RssiModel: log-distance path loss + attenuation of every crossed wall + noise,
- HeatmapGrid: averages measurements per cell, converts them to an OccupancyGrid
  payload and saves CSV / PNG.
"""
import csv
import math
import xml.etree.ElementTree as ET

import numpy as np


# --- Walls from the SDF world ---


def _pose2d(element):
    '''(x, y, yaw) from the <pose> child of an SDF element; zeros if missing.'''
    pose = element.find('pose')
    if pose is None or not (pose.text or '').strip():
        return 0.0, 0.0, 0.0
    values = [float(v) for v in pose.text.split()] + [0.0] * 6
    return values[0], values[1], values[5]


def _compose(parent, child):
    px, py, pyaw = parent
    cx, cy, cyaw = child
    c, s = math.cos(pyaw), math.sin(pyaw)
    return px + c*cx - s*cy, py + s*cx + c*cy, pyaw + cyaw


def load_walls(sdf_path):
    '''Box collisions of top-level models as (center_x, center_y, yaw, half_x, half_y).
    Planes (floor), cylinders and <include> models are ignored.'''
    root = ET.parse(sdf_path).getroot()
    world = root.find('world')
    walls = []
    for model in (world if world is not None else root).findall('model'):
        model_pose = _pose2d(model)
        for link in model.findall('link'):
            link_pose = _compose(model_pose, _pose2d(link))
            for collision in link.findall('collision'):
                size = collision.find('geometry/box/size')
                if size is None:
                    continue
                sx, sy = (float(v) for v in size.text.split()[:2])
                cx, cy, yaw = _compose(link_pose, _pose2d(collision))
                walls.append((cx, cy, yaw, sx / 2, sy / 2))
    return walls


def segment_hits_box(p0, p1, box):
    '''Does segment p0-p1 cross the rotated rectangle? (Liang-Barsky clipping)'''
    cx, cy, yaw, hx, hy = box
    c, s = math.cos(yaw), math.sin(yaw)

    def local(p):
        dx, dy = p[0] - cx, p[1] - cy
        return c*dx + s*dy, -s*dx + c*dy

    (x0, y0), (x1, y1) = local(p0), local(p1)
    dx, dy = x1 - x0, y1 - y0
    t0, t1 = 0.0, 1.0
    for p, q in ((-dx, x0 + hx), (dx, hx - x0), (-dy, y0 + hy), (dy, hy - y0)):
        if p == 0:
            if q < 0:
                return False
            continue
        t = q / p
        if p < 0:
            t0 = max(t0, t)
        else:
            t1 = min(t1, t)
        if t0 > t1:
            return False
    return True


# --- Signal model ---


class RssiModel:
    '''RSSI [dBm] = tx_power - 10*n*log10(d / 1 m) - wall_loss * crossed_walls + noise'''

    def __init__(self, ap, walls=(), tx_power=-40.0, path_loss_exp=2.0, wall_loss=5.0,
                 noise_std=2.0, floor=-95.0, ceiling=-30.0, seed=None):
        self.ap = (float(ap[0]), float(ap[1]))
        self.walls = list(walls)
        self.tx_power, self.path_loss_exp = tx_power, path_loss_exp
        self.wall_loss, self.noise_std = wall_loss, noise_std
        self.floor, self.ceiling = floor, ceiling
        self.rng = np.random.default_rng(seed)

    def crossed_walls(self, x, y):
        return sum(segment_hits_box(self.ap, (x, y), wall) for wall in self.walls)

    def expected(self, x, y):
        distance = max(math.hypot(x - self.ap[0], y - self.ap[1]), 0.1)
        rssi = (self.tx_power - 10.0 * self.path_loss_exp * math.log10(distance)
                - self.wall_loss * self.crossed_walls(x, y))
        return min(max(rssi, self.floor), self.ceiling)

    def measure(self, x, y):
        rssi = self.expected(x, y) + self.rng.normal(0.0, self.noise_std)
        return float(min(max(rssi, self.floor), self.ceiling))


# --- Heatmap ---


class HeatmapGrid:
    '''Mean RSSI per square cell of size `resolution` in the map frame.'''

    def __init__(self, resolution=0.2):
        self.resolution = float(resolution)
        self.sums, self.counts = {}, {}
        self.samples = []  # (stamp, x, y, rssi)

    def cell(self, x, y):
        return math.floor(x / self.resolution), math.floor(y / self.resolution)

    def add(self, x, y, rssi, stamp=0.0):
        key = self.cell(x, y)
        self.sums[key] = self.sums.get(key, 0.0) + rssi
        self.counts[key] = self.counts.get(key, 0) + 1
        self.samples.append((stamp, x, y, rssi))

    def means(self):
        return {key: self.sums[key] / self.counts[key] for key in self.sums}

    def to_occupancy(self, rssi_min=-90.0, rssi_max=-40.0, bounds=None):
        '''OccupancyGrid payload: (origin_x, origin_y, width, height, data).
        data: 0 = no measurement (transparent in RViz "costmap"), 1..98 = weak..strong.
        bounds = (xmin, ymin, xmax, ymax) extends the grid, e.g. to the SLAM map.'''
        means = self.means()
        if not means and bounds is None:
            return None
        cols = [c for c, _ in means] or []
        rows = [r for _, r in means] or []
        if bounds is not None:
            (bx0, by0), (bx1, by1) = self.cell(bounds[0], bounds[1]), self.cell(bounds[2], bounds[3])
            cols += [bx0, bx1]
            rows += [by0, by1]
        c0, r0 = min(cols), min(rows)
        width, height = max(cols) - c0 + 1, max(rows) - r0 + 1
        data = np.zeros((height, width), dtype=np.int8)
        span = max(rssi_max - rssi_min, 1e-6)
        for (c, r), rssi in means.items():
            level = (min(max(rssi, rssi_min), rssi_max) - rssi_min) / span
            data[r - r0, c - c0] = 1 + int(round(97 * level))
        return c0 * self.resolution, r0 * self.resolution, width, height, data.ravel().tolist()

    def save_csv(self, path):
        with open(path, 'w', newline='', encoding='utf-8') as file:
            writer = csv.writer(file)
            writer.writerow(['stamp_s', 'x_m', 'y_m', 'rssi_dbm'])
            for stamp, x, y, rssi in self.samples:
                writer.writerow([f'{stamp:.3f}', f'{x:.3f}', f'{y:.3f}', f'{rssi:.1f}'])

    def save_png(self, path, slam_map=None, ap=None, rssi_min=-90.0, rssi_max=-40.0):
        '''Needs matplotlib (python3-matplotlib). Returns False when it is missing.
        slam_map = (data[h, w] with -1/0..100, resolution, origin_x, origin_y)'''
        try:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt
        except ImportError:
            return False
        means = self.means()
        fig, ax = plt.subplots(figsize=(8, 6))
        if slam_map is not None:
            grid, res, ox, oy = slam_map
            shade = np.where(grid < 0, 0.85, np.where(grid > 50, 0.1, 1.0))
            ax.imshow(shade, cmap='gray', vmin=0, vmax=1, origin='lower', interpolation='nearest',
                      extent=(ox, ox + grid.shape[1]*res, oy, oy + grid.shape[0]*res))
        if means:
            cols = [c for c, _ in means]
            rows = [r for _, r in means]
            c0, r0 = min(cols), min(rows)
            image = np.full((max(rows) - r0 + 1, max(cols) - c0 + 1), np.nan)
            for (c, r), rssi in means.items():
                image[r - r0, c - c0] = rssi
            res = self.resolution
            shown = ax.imshow(image, cmap='turbo', vmin=rssi_min, vmax=rssi_max, origin='lower',
                              interpolation='nearest', alpha=0.85,
                              extent=(c0*res, (max(cols)+1)*res, r0*res, (max(rows)+1)*res))
            fig.colorbar(shown, ax=ax, label='RSSI [dBm]')
        if ap is not None:
            ax.plot(*ap, marker='*', markersize=16, color='white', markeredgecolor='black', label='AP')
            ax.legend(loc='upper right')
        ax.set_xlabel('x [m] (map)')
        ax.set_ylabel('y [m] (map)')
        ax.set_title(f'WiFi RSSI - {len(self.samples)} pomiarów')
        ax.set_aspect('equal')
        fig.tight_layout()
        fig.savefig(path, dpi=120)
        plt.close(fig)
        return True
