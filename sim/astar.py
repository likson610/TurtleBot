"""Planowanie globalnej trasy A* i operacje na siatce mapy.

Nie wymaga ROS. Pochodzenie algorytmów: README.md.
"""
import math
from collections import deque
from heapq import heappop, heappush

import numpy as np


# --- PLANOWANIE TRASY A* ---


def get_neighbors(pos, grid_map, rows, cols):
    '''
    Check if neighbors are obstacles
    '''
    dirs = [(-1, 0), (1, 0), (0, -1), (0, 1), 
            (-1, -1), (-1, 1), (1, -1), (1, 1)]

    neighbors = []
    distances = []

    for d in dirs:
        nr = pos[0] + d[0]
        nc = pos[1] + d[1]

        # if the cell is in range and free
        if 0 <= nr < rows and 0 <= nc < cols:
            if grid_map[nr, nc] == 0:
                # Do not cut diagonally between blocked cells.
                if d[0] and d[1] and (grid_map[pos[0], nc] or grid_map[nr, pos[1]]):
                    continue
                neighbors.append((nr, nc))

                # Diagonal movement costs more (sqrt(1^2 + 1^2) ≈ 1.414)
                if d[0] != 0 and d[1] != 0:
                    distances.append(2 ** 0.5)
                else:
                    distances.append(1.0)

    return neighbors, distances


def heuristic(p_start, p_end):
    '''Octile distance for eight-way movement'''
    dr = abs(p_start[0] - p_end[0])
    dc = abs(p_start[1] - p_end[1])
    return max(dr, dc) + (2 ** 0.5 - 1) * min(dr, dc)


def reconstruct_path(came_from, current, start, rows):
    '''Invert path'''

    path = [[current[0], current[1]]]

    while current != start:
        current = came_from[current]
        path.insert(0, [current[0], current[1]])

    return path


def a_star(grid_map, start, goal):
    '''Heap-based A*: avoid rescanning the entire map for each expanded cell.'''
    rows, cols = grid_map.shape

    start = (int(start[0]), int(start[1]))
    goal = (int(goal[0]), int(goal[1]))

    if not (0 <= start[0] < rows and 0 <= start[1] < cols
            and 0 <= goal[0] < rows and 0 <= goal[1] < cols):
        return []
    if grid_map[start] != 0 or grid_map[goal] != 0:
        return []

    costs = {start: 0.0}
    came_from = {}
    pending = [(heuristic(start, goal), 0.0, start)]
    while pending:
        _, cost, current = heappop(pending)
        if cost > costs.get(current, math.inf):
            continue
        if current == goal:
            return reconstruct_path(came_from, current, start, rows)
        neighbors, distances = get_neighbors(current, grid_map, rows, cols)
        for neighbor, distance in zip(neighbors, distances):
            candidate = cost + distance
            if candidate < costs.get(neighbor, math.inf):
                costs[neighbor] = candidate
                came_from[neighbor] = current
                heappush(pending, (candidate + heuristic(neighbor, goal), candidate, neighbor))
    return []


def reachable_cells(grid_map, start):
    """Zaznacz wolne pola połączone ze startem, bez przecinania narożników.

    Te same zasady przejazdu co w A*. Pozwala odrzucić nieosiągalne cele
    eksploracji przed uruchomieniem planowania trasy.
    """
    rows, cols = grid_map.shape
    reachable = np.zeros((rows, cols), dtype=bool)
    if start is None:
        return reachable
    row, col = int(start[0]), int(start[1])
    if not (0 <= row < rows and 0 <= col < cols) or grid_map[row, col] != 0:
        return reachable

    reachable[row, col] = True
    pending = deque([(row, col)])
    while pending:
        current = pending.popleft()
        neighbors, _ = get_neighbors(current, grid_map, rows, cols)
        for neighbor in neighbors:
            if not reachable[neighbor]:
                reachable[neighbor] = True
                pending.append(neighbor)
    return reachable


# --- WSPÓŁRZĘDNE MAPY I SPRAWDZANIE PRZEJAZDU ---


def world_to_grid(x, y, resolution, origin):
    ox, oy, yaw = origin
    dx, dy = x - ox, y - oy
    return (math.floor((-math.sin(yaw)*dx + math.cos(yaw)*dy) / resolution),
            math.floor((math.cos(yaw)*dx + math.sin(yaw)*dy) / resolution))


def grid_to_world(row, col, resolution, origin):
    ox, oy, yaw = origin
    x, y = (col + 0.5)*resolution, (row + 0.5)*resolution
    return ox + math.cos(yaw)*x - math.sin(yaw)*y, oy + math.sin(yaw)*x + math.cos(yaw)*y


def traversable_trajectory(traj, grid, resolution, origin):
    if not traj:
        return False
    for i, pose in enumerate(traj):
        previous = traj[max(0, i-1)]
        length = math.hypot(pose[0]-previous[0], pose[1]-previous[1])
        samples = max(1, math.ceil(length / (0.5*resolution)))
        for fraction in np.linspace(0, 1, samples+1):
            x = previous[0] + fraction*(pose[0]-previous[0])
            y = previous[1] + fraction*(pose[1]-previous[1])
            row, col = world_to_grid(x, y, resolution, origin)
            if not (0 <= row < grid.shape[0] and 0 <= col < grid.shape[1]):
                return False
            if grid[row, col]:
                return False
    return True
