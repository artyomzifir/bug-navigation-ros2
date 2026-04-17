"""Pure A* on a 2D occupancy grid.

No ROS dependencies — keeps the algorithm trivially unit-testable. The ROS
node imports `astar_grid` from here.

Conventions
-----------
grid : np.ndarray, shape (H, W), int8.
    Cell value >= occupied_threshold OR == -1 (unknown) is treated as
    blocked. All other cells are free.
start, goal : (row, col) tuples in grid index space.
Returns : list of (row, col) from start to goal inclusive, or None.

Movement is 8-connected; the heuristic is the octile distance, which is both
admissible and consistent for that move set so A* returns an optimal path.
"""
from __future__ import annotations

import heapq
import math
from typing import Iterable

import numpy as np


_SQRT2 = math.sqrt(2.0)
_NEIGHBOURS = (
    (-1,  0, 1.0),  (1,  0, 1.0),  (0, -1, 1.0),  (0,  1, 1.0),
    (-1, -1, _SQRT2), (-1, 1, _SQRT2), (1, -1, _SQRT2), (1, 1, _SQRT2),
)


def octile(a, b):
    dr = abs(a[0] - b[0])
    dc = abs(a[1] - b[1])
    return (dr + dc) + (_SQRT2 - 2.0) * min(dr, dc)


def is_free(grid, r, c, occupied_threshold=50, treat_unknown_as_obstacle=True):
    H, W = grid.shape
    if not (0 <= r < H and 0 <= c < W):
        return False
    v = int(grid[r, c])
    if v == -1:
        return not treat_unknown_as_obstacle
    return v < occupied_threshold


def inflate_obstacles(grid, radius_cells, occupied_threshold=50):
    """Dilate obstacles by `radius_cells` using a Euclidean disk kernel.

    Avoids pulling in scipy. Output cells inside the inflation radius are
    forced to value 100 so downstream code treats them as occupied.
    """
    if radius_cells <= 0:
        return grid.copy()

    H, W = grid.shape
    obstacle = (grid >= occupied_threshold) | (grid == -1)
    out = obstacle.copy()
    rs, cs = np.where(obstacle)
    r = radius_cells
    for dr in range(-r, r + 1):
        for dc in range(-r, r + 1):
            if dr * dr + dc * dc > r * r:
                continue
            nrs = rs + dr
            ncs = cs + dc
            mask = (nrs >= 0) & (nrs < H) & (ncs >= 0) & (ncs < W)
            out[nrs[mask], ncs[mask]] = True

    inflated = grid.copy()
    inflated[out] = 100
    return inflated


def astar_grid(grid, start, goal,
               occupied_threshold=50,
               treat_unknown_as_obstacle=True):
    """Run A* and return the shortest path or None."""
    if not is_free(grid, *start, occupied_threshold, treat_unknown_as_obstacle):
        return None
    if not is_free(grid, *goal, occupied_threshold, treat_unknown_as_obstacle):
        return None
    if start == goal:
        return [start]

    open_heap = []
    counter = 0  # tie-breaker so heapq never compares tuples
    g_score = {start: 0.0}
    parent = {}
    closed = set()

    heapq.heappush(open_heap, (octile(start, goal), counter, start))

    while open_heap:
        _, _, current = heapq.heappop(open_heap)
        if current in closed:
            continue
        if current == goal:
            return _reconstruct(parent, current)
        closed.add(current)

        cr, cc = current
        for dr, dc, step in _NEIGHBOURS:
            nr, nc = cr + dr, cc + dc
            neighbour = (nr, nc)
            if neighbour in closed:
                continue
            if not is_free(grid, nr, nc, occupied_threshold, treat_unknown_as_obstacle):
                continue
            # Disallow cutting diagonally through corner gaps.
            if dr != 0 and dc != 0:
                if not is_free(grid, cr + dr, cc, occupied_threshold,
                               treat_unknown_as_obstacle):
                    continue
                if not is_free(grid, cr, cc + dc, occupied_threshold,
                               treat_unknown_as_obstacle):
                    continue

            tentative_g = g_score[current] + step
            if tentative_g < g_score.get(neighbour, float('inf')):
                g_score[neighbour] = tentative_g
                parent[neighbour] = current
                f = tentative_g + octile(neighbour, goal)
                counter += 1
                heapq.heappush(open_heap, (f, counter, neighbour))

    return None


def _reconstruct(parent, end):
    path = [end]
    while end in parent:
        end = parent[end]
        path.append(end)
    path.reverse()
    return path


# ---------- coordinate conversion ----------
def world_to_grid(x, y, origin_x, origin_y, resolution):
    """OccupancyGrid is row-major from origin. Returns (row, col)."""
    col = int(math.floor((x - origin_x) / resolution))
    row = int(math.floor((y - origin_y) / resolution))
    return row, col


def grid_to_world(row, col, origin_x, origin_y, resolution):
    x = origin_x + (col + 0.5) * resolution
    y = origin_y + (row + 0.5) * resolution
    return x, y


def smooth_path(path):
    """Drop colinear waypoints — purely cosmetic, makes RViz path nicer."""
    pts = list(path)
    if len(pts) < 3:
        return pts
    out = [pts[0]]
    for i in range(1, len(pts) - 1):
        a, b, c = out[-1], pts[i], pts[i + 1]
        # cross product zero -> b lies on segment a->c
        if (b[0] - a[0]) * (c[1] - a[1]) != (b[1] - a[1]) * (c[0] - a[0]):
            out.append(b)
    out.append(pts[-1])
    return out
