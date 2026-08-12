#!/usr/bin/env python3
"""Reusable map, frontier, and path-planning primitives for Stage-3.

The functions in this file intentionally use only NumPy/OpenCV plus the Python
standard library so that the same frontier and A* logic can be used by both the
visualisation node and the central exploration manager.
"""
from __future__ import annotations

import heapq
import math
from dataclasses import dataclass
from typing import Iterable, List, Optional, Sequence, Tuple

import cv2
import numpy as np

Cell = Tuple[int, int]  # (x, y)


@dataclass
class GridMap:
    data: np.ndarray  # shape [height, width], occupancy values -1 / 0 / 100
    resolution: float
    origin_x: float
    origin_y: float
    frame_id: str = "map"

    @property
    def height(self) -> int:
        return int(self.data.shape[0])

    @property
    def width(self) -> int:
        return int(self.data.shape[1])

    def inside(self, cell: Cell) -> bool:
        x, y = cell
        return 0 <= x < self.width and 0 <= y < self.height

    def world_to_cell(self, x: float, y: float) -> Optional[Cell]:
        ix = int(math.floor((x - self.origin_x) / self.resolution))
        iy = int(math.floor((y - self.origin_y) / self.resolution))
        cell = (ix, iy)
        return cell if self.inside(cell) else None

    def cell_to_world(self, cell: Cell) -> Tuple[float, float]:
        x, y = cell
        return (
            self.origin_x + (x + 0.5) * self.resolution,
            self.origin_y + (y + 0.5) * self.resolution,
        )


@dataclass
class FrontierCluster:
    cluster_id: int
    cells: List[Cell]
    centroid_cell: Cell
    viewpoint_cell: Cell
    frontier_length_m: float
    information_gain: float
    risk: float
    zone: Tuple[int, int]
    # Filled by frontier_topology.associate_frontier_clusters().  They remain
    # optional so Stage-3 users that only need geometric frontiers keep working.
    topology_region_id: Optional[str] = None
    topology_branch_id: Optional[int] = None
    topology_layer: Optional[str] = None
    topology_association_distance_m: Optional[float] = None
    topology_confidence: str = "LOW"


def occupancy_grid_from_flat(data: Sequence[int], width: int, height: int,
                             resolution: float, origin_x: float, origin_y: float,
                             frame_id: str = "map") -> GridMap:
    array = np.asarray(data, dtype=np.int16)
    if array.size != width * height:
        raise ValueError("OccupancyGrid data size does not match width*height.")
    return GridMap(array.reshape((height, width)), float(resolution), float(origin_x), float(origin_y), frame_id)


def neighbours8(cell: Cell) -> Iterable[Cell]:
    x, y = cell
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            if dx == 0 and dy == 0:
                continue
            yield x + dx, y + dy


def neighbours4(cell: Cell) -> Iterable[Cell]:
    x, y = cell
    yield x + 1, y
    yield x - 1, y
    yield x, y + 1
    yield x, y - 1


def bresenham_cells(a: Cell, b: Cell) -> List[Cell]:
    x0, y0 = a
    x1, y1 = b
    cells: List[Cell] = []
    dx = abs(x1 - x0)
    sx = 1 if x0 < x1 else -1
    dy = -abs(y1 - y0)
    sy = 1 if y0 < y1 else -1
    err = dx + dy
    while True:
        cells.append((x0, y0))
        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x0 += sx
        if e2 <= dx:
            err += dx
            y0 += sy
    return cells


def build_passable_mask(grid: GridMap, occupied_threshold: int,
                        inflation_radius_m: float) -> Tuple[np.ndarray, np.ndarray]:
    """Return passable known-free cells and clearance in metres.

    Unknown cells are deliberately not traversable. The explorer reaches a safe
    known-free viewpoint next to unknown cells, then lets LiDAR expand the map.
    """
    occupied = (grid.data >= occupied_threshold).astype(np.uint8)
    radius_cells = max(0, int(math.ceil(inflation_radius_m / grid.resolution)))
    if radius_cells > 0:
        kernel_size = 2 * radius_cells + 1
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
        inflated = cv2.dilate(occupied, kernel)
    else:
        inflated = occupied
    free = grid.data == 0
    passable = free & (inflated == 0)
    # cv2.distanceTransform returns a pixel distance to the nearest zero pixel.
    # Invert occupancy so obstacles are zeros and free cells receive positive distance.
    clearance_px = cv2.distanceTransform((occupied == 0).astype(np.uint8), cv2.DIST_L2, 3)
    clearance_m = clearance_px.astype(np.float32) * float(grid.resolution)
    return passable, clearance_m


def _unknown_integral(grid: GridMap) -> np.ndarray:
    unknown = (grid.data < 0).astype(np.uint8)
    return cv2.integral(unknown, sdepth=cv2.CV_32S)


def _rect_sum(integral: np.ndarray, x0: int, y0: int, x1: int, y1: int) -> int:
    # x1/y1 are exclusive and may be clamped by caller.
    return int(integral[y1, x1] - integral[y0, x1] - integral[y1, x0] + integral[y0, x0])


def unknown_gain(integral: np.ndarray, grid: GridMap, cell: Cell, radius_m: float) -> float:
    radius = max(1, int(math.ceil(radius_m / grid.resolution)))
    x, y = cell
    x0 = max(0, x - radius)
    y0 = max(0, y - radius)
    x1 = min(grid.width, x + radius + 1)
    y1 = min(grid.height, y + radius + 1)
    return float(_rect_sum(integral, x0, y0, x1, y1))


def _frontier_mask(grid: GridMap) -> np.ndarray:
    free = grid.data == 0
    unknown = grid.data < 0
    unknown_u8 = unknown.astype(np.uint8)
    neighbour_unknown = cv2.dilate(unknown_u8, np.ones((3, 3), np.uint8)) > 0
    return free & neighbour_unknown


def _connected_components(mask: np.ndarray) -> List[List[Cell]]:
    height, width = mask.shape
    visited = np.zeros_like(mask, dtype=np.uint8)
    components: List[List[Cell]] = []
    for y in range(height):
        for x in range(width):
            if not mask[y, x] or visited[y, x]:
                continue
            stack = [(x, y)]
            visited[y, x] = 1
            cells: List[Cell] = []
            while stack:
                cx, cy = stack.pop()
                cells.append((cx, cy))
                for nx, ny in neighbours8((cx, cy)):
                    if 0 <= nx < width and 0 <= ny < height and mask[ny, nx] and not visited[ny, nx]:
                        visited[ny, nx] = 1
                        stack.append((nx, ny))
            components.append(cells)
    return components


def _nearest_cell(cells: Sequence[Cell], target: Tuple[float, float]) -> Cell:
    tx, ty = target
    return min(cells, key=lambda c: (c[0] - tx) ** 2 + (c[1] - ty) ** 2)


def extract_frontier_clusters(grid: GridMap,
                              occupied_threshold: int = 65,
                              inflation_radius_m: float = 0.45,
                              min_frontier_length_m: float = 0.8,
                              gain_radius_m: float = 3.5,
                              min_clearance_m: float = 0.45,
                              hgrid_size_m: float = 4.0,
                              sample_stride: int = 2) -> Tuple[List[FrontierCluster], np.ndarray]:
    """Extract clustered frontiers and one safe information-rich viewpoint per cluster."""
    passable, clearance_m = build_passable_mask(grid, occupied_threshold, inflation_radius_m)
    raw_frontier = _frontier_mask(grid)
    integral = _unknown_integral(grid)
    clusters: List[FrontierCluster] = []
    next_id = 1

    for cells in _connected_components(raw_frontier):
        length_m = len(cells) * grid.resolution
        if length_m < min_frontier_length_m:
            continue
        centroid = (float(np.mean([c[0] for c in cells])), float(np.mean([c[1] for c in cells])))
        centroid_cell = _nearest_cell(cells, centroid)

        candidates = cells[::max(1, int(sample_stride))]
        best_cell: Optional[Cell] = None
        best_score = -float("inf")
        best_gain = 0.0
        for candidate in candidates:
            x, y = candidate
            if not passable[y, x]:
                continue
            clearance = float(clearance_m[y, x])
            if clearance < min_clearance_m:
                continue
            gain = unknown_gain(integral, grid, candidate, gain_radius_m)
            distance_to_centroid = math.hypot(candidate[0] - centroid[0], candidate[1] - centroid[1]) * grid.resolution
            # Prefer high unknown gain, but retain a representative point near the frontier centroid.
            score = gain - 1.5 * distance_to_centroid + 0.5 * clearance
            if score > best_score:
                best_score = score
                best_cell = candidate
                best_gain = gain

        if best_cell is None:
            continue
        bx, by = best_cell
        risk = max(0.0, min(1.0, 1.0 - float(clearance_m[by, bx]) / max(min_clearance_m * 2.0, 1e-3)))
        wx, wy = grid.cell_to_world(best_cell)
        zone = (int(math.floor(wx / hgrid_size_m)), int(math.floor(wy / hgrid_size_m)))
        clusters.append(FrontierCluster(
            cluster_id=next_id,
            cells=cells,
            centroid_cell=centroid_cell,
            viewpoint_cell=best_cell,
            frontier_length_m=length_m,
            information_gain=best_gain,
            risk=risk,
            zone=zone,
        ))
        next_id += 1
    return clusters, passable


def nearest_passable(passable: np.ndarray, cell: Cell, max_radius_cells: int = 25) -> Optional[Cell]:
    height, width = passable.shape
    x, y = cell
    if 0 <= x < width and 0 <= y < height and passable[y, x]:
        return cell
    for radius in range(1, max_radius_cells + 1):
        for yy in range(y - radius, y + radius + 1):
            for xx in (x - radius, x + radius):
                if 0 <= xx < width and 0 <= yy < height and passable[yy, xx]:
                    return xx, yy
        for xx in range(x - radius + 1, x + radius):
            for yy in (y - radius, y + radius):
                if 0 <= xx < width and 0 <= yy < height and passable[yy, xx]:
                    return xx, yy
    return None


def astar(passable: np.ndarray, start: Cell, goal: Cell,
          max_expansions: int = 30000) -> Optional[List[Cell]]:
    """8-connected A* over known free cells."""
    height, width = passable.shape
    if not (0 <= start[0] < width and 0 <= start[1] < height and passable[start[1], start[0]]):
        return None
    if not (0 <= goal[0] < width and 0 <= goal[1] < height and passable[goal[1], goal[0]]):
        return None

    def heuristic(a: Cell, b: Cell) -> float:
        return math.hypot(a[0] - b[0], a[1] - b[1])

    open_heap: List[Tuple[float, float, Cell]] = [(heuristic(start, goal), 0.0, start)]
    parent = {start: None}
    g_cost = {start: 0.0}
    expansions = 0
    directions = [
        (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
        (1, 1, math.sqrt(2.0)), (1, -1, math.sqrt(2.0)),
        (-1, 1, math.sqrt(2.0)), (-1, -1, math.sqrt(2.0)),
    ]

    while open_heap and expansions < max_expansions:
        _, current_g, current = heapq.heappop(open_heap)
        if current_g > g_cost.get(current, float("inf")) + 1e-9:
            continue
        if current == goal:
            path: List[Cell] = []
            node: Optional[Cell] = current
            while node is not None:
                path.append(node)
                node = parent[node]
            path.reverse()
            return path
        expansions += 1
        cx, cy = current
        for dx, dy, step_cost in directions:
            nx, ny = cx + dx, cy + dy
            if not (0 <= nx < width and 0 <= ny < height and passable[ny, nx]):
                continue
            # Do not cut diagonally through obstacle corners.
            if dx != 0 and dy != 0 and (not passable[cy, nx] or not passable[ny, cx]):
                continue
            neighbour = (nx, ny)
            tentative = current_g + step_cost
            if tentative + 1e-9 < g_cost.get(neighbour, float("inf")):
                g_cost[neighbour] = tentative
                parent[neighbour] = current
                heapq.heappush(open_heap, (tentative + heuristic(neighbour, goal), tentative, neighbour))
    return None


def line_is_passable(passable: np.ndarray, a: Cell, b: Cell) -> bool:
    height, width = passable.shape
    for x, y in bresenham_cells(a, b):
        if not (0 <= x < width and 0 <= y < height and passable[y, x]):
            return False
    return True


def simplify_path(path: Sequence[Cell], passable: np.ndarray) -> List[Cell]:
    if len(path) <= 2:
        return list(path)
    result = [path[0]]
    anchor = 0
    while anchor < len(path) - 1:
        target = len(path) - 1
        while target > anchor + 1 and not line_is_passable(passable, path[anchor], path[target]):
            target -= 1
        result.append(path[target])
        anchor = target
    return result


def path_length_m(path: Sequence[Cell], resolution: float) -> float:
    if len(path) < 2:
        return 0.0
    return float(sum(math.hypot(path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1])
                     for i in range(1, len(path))) * resolution)
