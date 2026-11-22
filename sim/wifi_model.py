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


def count_crossings(ax, ay, px, py, walls):
    '''Vectorised segment_hits_box: number of walls crossed by segments (ax, ay) -> (px, py).
    Arguments broadcast with numpy rules, e.g. candidates (C, 1) against points (P,) -> (C, P).'''
    ax, ay, px, py = (np.asarray(v, dtype=float) for v in (ax, ay, px, py))
    count = np.zeros(np.broadcast_shapes(ax.shape, px.shape), dtype=np.int32)
    for cx, cy, yaw, hx, hy in walls:
        c, s = math.cos(yaw), math.sin(yaw)
        x0, y0 = c*(ax - cx) + s*(ay - cy), -s*(ax - cx) + c*(ay - cy)
        x1, y1 = c*(px - cx) + s*(py - cy), -s*(px - cx) + c*(py - cy)
        dx, dy = x1 - x0, y1 - y0
        t0 = np.zeros(count.shape)
        t1 = np.ones(count.shape)
        inside = np.ones(count.shape, dtype=bool)
        for p, q in ((-dx, x0 + hx), (dx, hx - x0), (-dy, y0 + hy), (dy, hy - y0)):
            p, q = np.broadcast_to(p, count.shape), np.broadcast_to(q, count.shape)
            parallel = p == 0
            inside &= ~(parallel & (q < 0))
            t = np.divide(q, p, out=np.zeros(count.shape), where=~parallel)
            t0 = np.where(~parallel & (p < 0), np.maximum(t0, t), t0)
            t1 = np.where(~parallel & (p > 0), np.minimum(t1, t), t1)
        count += inside & (t0 <= t1)
    return count


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


# --- Access point localisation ---


def _grid_fit(cx, cy, x, y, rssi, walls, wall_loss, d_min):
    '''For every candidate AP (cx, cy): closed-form P0, n and the sum of squared residuals.'''
    cx, cy = np.asarray(cx, dtype=float).reshape(-1, 1), np.asarray(cy, dtype=float).reshape(-1, 1)
    feature = -10.0 * np.log10(np.maximum(np.hypot(x - cx, y - cy), d_min))
    target = rssi + wall_loss * count_crossings(cx, cy, x, y, walls) if walls else np.broadcast_to(rssi, feature.shape)
    f_mean, t_mean = feature.mean(axis=1, keepdims=True), target.mean(axis=1, keepdims=True)
    f_var = ((feature - f_mean) ** 2).sum(axis=1, keepdims=True)
    n = np.divide(((feature - f_mean) * (target - t_mean)).sum(axis=1, keepdims=True), f_var,
                  out=np.zeros_like(f_var), where=f_var > 1e-9)
    p0 = t_mean - n * f_mean
    sse = ((target - p0 - n * feature) ** 2).sum(axis=1)
    sse[(n.ravel() < 0.5) | (f_var.ravel() <= 1e-9)] = np.inf  # signal must fall with distance
    return sse, p0.ravel(), n.ravel()


def locate_ap(x, y, rssi, walls=(), wall_loss=5.0, margin=3.0, coarse_step=0.4, fine_step=0.1,
              max_candidates=1500, d_min=0.1):
    '''Estimates the AP position from RSSI samples (inverse of RssiModel, without noise).

    Fits RSSI = P0 - 10*n*log10(d) - wall_loss * crossed_walls with unknown AP (x, y), P0, n:
    1. grid search over candidate AP positions (sample bounding box + margin, coarse grid and
       then a fine grid around the best cell); for a fixed AP the model is linear in P0 and n,
       so they have a closed-form least-squares solution,
    2. refinement of all four parameters with scipy least_squares (wall counts frozen at the
       current estimate and recomputed until they stop changing).

    Returns None when there are too few samples, otherwise a dict with x, y, tx_power,
    path_loss_exp, rmse [dB], cov (2x2 covariance of x, y in m^2) and samples.'''
    from scipy.optimize import least_squares

    x, y, rssi = (np.asarray(v, dtype=float).ravel() for v in (x, y, rssi))
    if len(rssi) < 5:
        return None
    walls = list(walls)

    xmin, xmax = x.min() - margin, x.max() + margin
    ymin, ymax = y.min() - margin, y.max() + margin
    step = max(coarse_step, math.sqrt((xmax - xmin) * (ymax - ymin) / max_candidates))

    # 1. Grid search, coarse then fine around the best coarse candidate.
    gx, gy = np.meshgrid(np.arange(xmin, xmax + 1e-9, step), np.arange(ymin, ymax + 1e-9, step))
    sse, _, _ = _grid_fit(gx, gy, x, y, rssi, walls, wall_loss, d_min)
    if not np.isfinite(sse).any():
        return None
    best = int(np.argmin(sse))
    offsets = np.arange(-step, step + 1e-9, fine_step)
    gx, gy = np.meshgrid(gx.ravel()[best] + offsets, gy.ravel()[best] + offsets)
    sse, p0, n = _grid_fit(gx, gy, x, y, rssi, walls, wall_loss, d_min)
    best = int(np.argmin(sse))
    params = np.array([gx.ravel()[best], gy.ravel()[best], p0[best], min(max(n[best], 0.6), 5.9)])

    # 2. Refinement.
    def residuals(v, crossings):
        d = np.maximum(np.hypot(x - v[0], y - v[1]), d_min)
        return v[2] - 10.0 * v[3] * np.log10(d) - wall_loss * crossings - rssi

    crossings = count_crossings(params[0], params[1], x, y, walls) if walls else np.zeros_like(rssi)
    for _ in range(3):
        fit = least_squares(residuals, params, args=(crossings,),
                            bounds=([-np.inf, -np.inf, -np.inf, 0.5], [np.inf, np.inf, np.inf, 6.0]))
        params = fit.x
        if not walls:
            break
        new_crossings = count_crossings(params[0], params[1], x, y, walls)
        if np.array_equal(new_crossings, crossings):
            break
        crossings = new_crossings

    res = residuals(params, crossings)
    dof = max(len(rssi) - 4, 1)
    cov = np.linalg.pinv(fit.jac.T @ fit.jac) * float(res @ res) / dof
    return {'x': float(params[0]), 'y': float(params[1]),
            'tx_power': float(params[2]), 'path_loss_exp': float(params[3]),
            'rmse': float(math.sqrt(np.mean(res ** 2))), 'cov': cov[:2, :2], 'samples': len(rssi)}


def error_ellipse(cov, sigma=2.0):
    '''(width, height, angle_rad) of the sigma-ellipse of a 2x2 covariance (full axis lengths).'''
    values, vectors = np.linalg.eigh(np.asarray(cov, dtype=float))
    values = np.maximum(values, 0.0)
    angle = math.atan2(vectors[1, 1], vectors[0, 1])  # direction of the larger eigenvalue
    return 2*sigma*math.sqrt(values[1]), 2*sigma*math.sqrt(values[0]), angle


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

    def cell_points(self):
        '''Per-cell mean position and RSSI as arrays (x, y, rssi): less noise and fewer points
        than raw samples, and long stops in one place do not dominate a fit.'''
        acc = {}
        for _, x, y, rssi in self.samples:
            a = acc.setdefault(self.cell(x, y), [0.0, 0.0, 0.0, 0])
            a[0] += x
            a[1] += y
            a[2] += rssi
            a[3] += 1
        if not acc:
            return np.empty(0), np.empty(0), np.empty(0)
        values = np.array([[a[0] / a[3], a[1] / a[3], a[2] / a[3]] for a in acc.values()])
        return values[:, 0], values[:, 1], values[:, 2]

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

    def save_png(self, path, slam_map=None, ap=None, rssi_min=-90.0, rssi_max=-40.0, estimate=None):
        '''Needs matplotlib (python3-matplotlib). Returns False when it is missing.
        slam_map = (data[h, w] with -1/0..100, resolution, origin_x, origin_y)
        estimate = result of locate_ap() (in the same frame), drawn with its 2-sigma ellipse.'''
        try:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt
            from matplotlib.patches import Ellipse
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
        if estimate is not None:
            width, height, angle = error_ellipse(estimate['cov'])
            ax.add_patch(Ellipse((estimate['x'], estimate['y']), width, height, angle=math.degrees(angle),
                                 fill=False, color='magenta', linewidth=2, label='2σ'))
            ax.plot(estimate['x'], estimate['y'], marker='X', markersize=12, color='magenta',
                    markeredgecolor='black', linestyle='none', label='AP (estymacja)')
        if ap is not None or estimate is not None:
            # imshow fixes the limits to the image; make sure both APs (and the ellipse) are visible.
            (x0, x1), (y0, y1) = ax.get_xlim(), ax.get_ylim()
            points = [ap] if ap is not None else []
            if estimate is not None:
                reach = max(width, height) / 2
                points += [(estimate['x'] - reach, estimate['y'] - reach),
                           (estimate['x'] + reach, estimate['y'] + reach)]
            for px, py in points:
                x0, x1, y0, y1 = min(x0, px - 0.3), max(x1, px + 0.3), min(y0, py - 0.3), max(y1, py + 0.3)
            ax.set_xlim(x0, x1)
            ax.set_ylim(y0, y1)
            ax.legend(loc='best', framealpha=0.8)
        ax.set_xlabel('x [m] (map)')
        ax.set_ylabel('y [m] (map)')
        ax.set_title(f'WiFi RSSI - {len(self.samples)} pomiarów')
        ax.set_aspect('equal')
        fig.tight_layout()
        fig.savefig(path, dpi=120)
        plt.close(fig)
        return True
