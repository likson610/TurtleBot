"""Lokalny dobór prędkości DWA, przewidywanie ruchu i hamowanie.

Ustawienia jazdy znajdują się w DWAConfig. Moduł nie wymaga ROS.
Pochodzenie algorytmów: README.md.
"""
import math

import numpy as np

from astar import traversable_trajectory


class DWAConfig:
    # linear velocity limit
    max_linear_vel = 0.15
    min_linear_vel = 0.0

    # angular velocity limit
    max_angular_vel = 1.0
    min_angular_vel = -1.0

    # limits of velocities' acceleration
    max_linear_acc = 0.5
    max_angular_acc = 3.5

    # resolution for predicting velocities (sample of each)
    linear_resolution = 0.01
    angular_resolution = 0.05

    # using for predict velocity: v = a * dt
    dt = 0.1
    # lower -> worse prediction 
    predict_time = 1.2

    # weights to adjust (increase if specific paramater is more important)
    w_heading   = 1.0 # angular direction to target point
    w_clearance = 0.8 # bypassing obstacles
    w_velocity  = 0.6 # speed value

    # radius of wheel platform
    robo_radius = 0.12
    # lower -> less computations
    lidar_max_range = 3.0
    # keep robot from hitting obstacle
    inflation_radius = 0.20
    safety_margin = 0.05
    reaction_time = 0.30


# --- WYBÓR PRĘDKOŚCI ---

def dwa_control(pose, velocity, goal, obstacles, grid, resolution, origin, cfg):
    """Zwróć prędkości (v, w), sprawdzając ruch na mapie i skanach.

    pose = (x, y, yaw), velocity = (v, w), goal = (x, y).
    grid to mapa z marginesem od przeszkód; origin = (x, y, yaw).
    """
    x, y, theta = pose
    v, w = velocity
    goal_x, goal_y = goal
    # velocity limits (rapid velocity changes are impossible)
    v_min = max(cfg.min_linear_vel,  v - cfg.max_linear_acc  * cfg.dt)
    v_max = min(cfg.max_linear_vel,  v + cfg.max_linear_acc  * cfg.dt)
    w_min = max(cfg.min_angular_vel, w - cfg.max_angular_acc * cfg.dt)
    w_max = min(cfg.max_angular_vel, w + cfg.max_angular_acc * cfg.dt)

    best_score = -math.inf
    best_v, best_w = 0.0, 0.0

    # velocity samples (change resolutions to speed up calculations)
    v_samples = bounded_samples(v_min, v_max, cfg.linear_resolution)
    w_samples = bounded_samples(w_min, w_max, cfg.angular_resolution)
    obs_arr = np.array(obstacles) if obstacles else None

    # simulate trajectory for each velocities' sample
    for sv in v_samples:
        for sw in w_samples:
            traj = [(x, y, theta)] + trajectory(x, y, theta, sv, sw, cfg.dt, cfg.predict_time)
            if not traversable_trajectory(traj, grid, resolution, origin):
                continue
            min_dist = (cfg.lidar_max_range if obs_arr is None or len(obs_arr) == 0
                        else path_clearance(traj, obs_arr))

            if min_dist < cfg.robo_radius + cfg.safety_margin: continue

            # scores base on weights 
            heading_score = score_heading(traj, goal_x, goal_y)
            clearance_score = min(min_dist / cfg.lidar_max_range, 1.0)
            score = (cfg.w_heading * heading_score + cfg.w_clearance * clearance_score + cfg.w_velocity * sv)

            if score > best_score:
                best_score = score
                best_v, best_w = sv, sw

    # No unchecked recovery turn: wait for a valid collision-free command.
    if best_score == -math.inf:
        best_v, best_w = 0.0, 0.0

    return best_v, best_w


def score_heading(traj, goal_x, goal_y):
    """Oceń kierunek i odległość końca przewidywanej trajektorii od celu."""
    end_x, end_y, end_theta = traj[-1]
    goal_angle   = math.atan2(goal_y - end_y, goal_x - end_x)
    # atan2(sin/cos) - angle normalization
    angle_diff = abs(math.atan2(math.sin(goal_angle - end_theta), math.cos(goal_angle - end_theta)))

    # provide heading toward target point
    w_head = 0.7
    return w_head * ((math.pi - angle_diff) / math.pi) + 0.3 * (1.0 / (1.0 + math.hypot(goal_x - end_x, goal_y - end_y)))


# --- PRZEWIDYWANIE RUCHU I ODLEGŁOŚCI ---

def bounded_samples(low, high, resolution):
    count = max(2, math.ceil((high - low) / resolution) + 1)
    values = list(np.linspace(low, high, count))
    if low <= 0 <= high:
        values.append(0.0)
    return sorted(set(values))


def trajectory(x, y, yaw, v, w, dt, horizon):
    result = []
    for i in range(1, int(horizon / dt) + 1):
        t = i * dt
        if abs(w) < 1e-9:
            px, py = x + v*t*math.cos(yaw), y + v*t*math.sin(yaw)
        else:
            px = x + v/w * (math.sin(yaw+w*t) - math.sin(yaw))
            py = y - v/w * (math.cos(yaw+w*t) - math.cos(yaw))
        result.append((px, py, yaw + w*t))
    return result


def path_clearance(traj, points):
    """Minimum distance to swept line segments, including the current position."""
    if points is None or len(points) == 0:
        return math.inf
    path = np.asarray(traj, dtype=float)[:, :2]
    obstacles = np.asarray(points, dtype=float)
    if len(path) == 1:
        return float(np.linalg.norm(obstacles-path[0], axis=1).min())
    starts, vectors = path[:-1], np.diff(path, axis=0)
    delta = obstacles[None, :, :] - starts[:, None, :]
    denominator = np.sum(vectors*vectors, axis=1)
    fractions = np.clip(np.sum(delta*vectors[:, None, :], axis=2)
                        / np.maximum(denominator[:, None], 1e-12), 0, 1)
    nearest = starts[:, None, :] + fractions[:, :, None]*vectors[:, None, :]
    return float(np.linalg.norm(obstacles[None, :, :]-nearest, axis=2).min())


# --- SPRAWDZANIE DROGI HAMOWANIA ---

def braking_path(v, w, linear_deceleration, angular_deceleration, reaction_time, dt=0.05):
    """Sweep through reaction delay and gradual braking, in the base frame."""
    horizon = reaction_time + max(abs(v)/linear_deceleration, abs(w)/angular_deceleration)
    path = [(0.0, 0.0, 0.0)]
    elapsed = 0.0
    while elapsed < horizon - 1e-10:
        step = min(dt, horizon-elapsed)
        braking_time = max(0.0, elapsed + step/2 - reaction_time)
        speed = math.copysign(max(0.0, abs(v)-linear_deceleration*braking_time), v)
        turn = math.copysign(max(0.0, abs(w)-angular_deceleration*braking_time), w)
        path.extend(trajectory(*path[-1], speed, turn, step, step))
        elapsed += step
    return path


def braking_clear(v, w, points, cfg, scan_age=0.0):
    sweep = braking_path(v, w, cfg.max_linear_acc, cfg.max_angular_acc,
                         cfg.reaction_time + max(0.0, scan_age))
    return path_clearance(sweep, points) >= cfg.robo_radius + cfg.safety_margin
