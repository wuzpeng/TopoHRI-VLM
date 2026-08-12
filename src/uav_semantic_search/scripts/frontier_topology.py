#!/usr/bin/env python3
"""Free-space skeleton topology and frontier-region association.

The module is intentionally ROS-independent.  It consumes the already inflated
``passable`` mask produced by :mod:`frontier_core`, extracts a one-pixel free-
space skeleton, splits the skeleton at junctions, and assigns each traversable
cell to its closest skeleton branch through a multi-source geodesic search.

Frontier candidates attached to the same branch therefore receive the same
``topology_region_id`` even when they are separate frontier connected
components.  Only NumPy/OpenCV and the Python standard library are required.
"""
from __future__ import annotations

from dataclasses import dataclass
import heapq
import math
from typing import Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np

Cell = Tuple[int, int]


@dataclass
class SkeletonBranch:
    branch_id: int
    cells: List[Cell]
    length_m: float
    centroid_cell: Cell
    endpoint_cells: List[Cell]
    adjacent_junction_ids: List[int]


@dataclass
class FreeSpaceTopology:
    layer_id: str
    skeleton_mask: np.ndarray
    endpoint_mask: np.ndarray
    junction_mask: np.ndarray
    junction_labels: np.ndarray
    branch_labels: np.ndarray
    nearest_branch_labels: np.ndarray
    branch_distance_m: np.ndarray
    branches: List[SkeletonBranch]


def _neighbour_count(mask: np.ndarray) -> np.ndarray:
    kernel = np.ones((3, 3), dtype=np.uint8)
    kernel[1, 1] = 0
    return cv2.filter2D(mask.astype(np.uint8), cv2.CV_16U, kernel,
                        borderType=cv2.BORDER_CONSTANT)


def _remove_small_components(mask: np.ndarray, min_area_cells: int) -> np.ndarray:
    binary = mask.astype(np.uint8)
    if min_area_cells <= 1 or not np.any(binary):
        return binary.astype(bool)
    count, labels, stats, _ = cv2.connectedComponentsWithStats(binary, connectivity=8)
    keep = np.zeros_like(binary)
    for label in range(1, count):
        if int(stats[label, cv2.CC_STAT_AREA]) >= min_area_cells:
            keep[labels == label] = 1
    return keep.astype(bool)


def zhang_suen_thinning(mask: np.ndarray, max_iterations: int = 500) -> np.ndarray:
    """Return a one-pixel skeleton using vectorised Zhang-Suen thinning."""
    image = mask.astype(np.uint8).copy()
    if image.ndim != 2:
        raise ValueError("Skeleton input must be a 2-D mask.")
    image[[0, -1], :] = 0
    image[:, [0, -1]] = 0

    for _ in range(max(1, int(max_iterations))):
        changed = False
        for first_subiteration in (True, False):
            p = np.pad(image, 1, mode="constant")
            p2 = p[:-2, 1:-1]
            p3 = p[:-2, 2:]
            p4 = p[1:-1, 2:]
            p5 = p[2:, 2:]
            p6 = p[2:, 1:-1]
            p7 = p[2:, :-2]
            p8 = p[1:-1, :-2]
            p9 = p[:-2, :-2]
            neighbours = p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9
            transitions = (
                ((p2 == 0) & (p3 == 1)).astype(np.uint8)
                + ((p3 == 0) & (p4 == 1)).astype(np.uint8)
                + ((p4 == 0) & (p5 == 1)).astype(np.uint8)
                + ((p5 == 0) & (p6 == 1)).astype(np.uint8)
                + ((p6 == 0) & (p7 == 1)).astype(np.uint8)
                + ((p7 == 0) & (p8 == 1)).astype(np.uint8)
                + ((p8 == 0) & (p9 == 1)).astype(np.uint8)
                + ((p9 == 0) & (p2 == 1)).astype(np.uint8)
            )
            remove = ((image == 1) & (neighbours >= 2) & (neighbours <= 6)
                      & (transitions == 1))
            if first_subiteration:
                remove &= ((p2 * p4 * p6) == 0) & ((p4 * p6 * p8) == 0)
            else:
                remove &= ((p2 * p4 * p8) == 0) & ((p2 * p6 * p8) == 0)
            if np.any(remove):
                image[remove] = 0
                changed = True
        if not changed:
            break
    return image.astype(bool)


def _trace_endpoint_spur(skeleton: np.ndarray, endpoint: Cell,
                         max_steps: int) -> Tuple[List[Cell], bool]:
    """Trace from an endpoint; return path and whether a junction was reached."""
    height, width = skeleton.shape
    path = [endpoint]
    previous: Optional[Cell] = None
    current = endpoint
    for _ in range(max_steps + 1):
        cx, cy = current
        neighbours: List[Cell] = []
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                nx, ny = cx + dx, cy + dy
                if (0 <= nx < width and 0 <= ny < height and skeleton[ny, nx]
                        and (nx, ny) != previous):
                    neighbours.append((nx, ny))
        if len(neighbours) == 0:
            return path, False
        if len(neighbours) > 1:
            return path, True
        previous, current = current, neighbours[0]
        path.append(current)
        degree = int(_local_degree(skeleton, current))
        if degree >= 3:
            return path, True
        if degree <= 1 and current != endpoint:
            return path, False
        if len(path) > max_steps:
            return path, False
    return path, False


def _local_degree(mask: np.ndarray, cell: Cell) -> int:
    x, y = cell
    y0, y1 = max(0, y - 1), min(mask.shape[0], y + 2)
    x0, x1 = max(0, x - 1), min(mask.shape[1], x + 2)
    return int(np.count_nonzero(mask[y0:y1, x0:x1])) - int(bool(mask[y, x]))


def prune_short_spurs(skeleton: np.ndarray, max_spur_cells: int,
                      max_rounds: int = 8) -> np.ndarray:
    """Remove terminal branches shorter than ``max_spur_cells``."""
    pruned = skeleton.astype(bool).copy()
    if max_spur_cells <= 0:
        return pruned
    for _ in range(max(1, int(max_rounds))):
        degree = _neighbour_count(pruned)
        ys, xs = np.where(pruned & (degree == 1))
        to_remove: List[Cell] = []
        for x, y in zip(xs.tolist(), ys.tolist()):
            path, reached_junction = _trace_endpoint_spur(
                pruned, (x, y), max_spur_cells)
            if reached_junction and len(path) - 1 <= max_spur_cells:
                # Preserve the junction pixel itself.
                to_remove.extend(path[:-1])
        if not to_remove:
            break
        for x, y in to_remove:
            pruned[y, x] = False
    return pruned


def _split_branches(skeleton: np.ndarray, resolution: float,
                    min_branch_cells: int) -> Tuple[np.ndarray, np.ndarray,
                                                    np.ndarray, np.ndarray,
                                                    List[SkeletonBranch]]:
    degree = _neighbour_count(skeleton)
    endpoints = skeleton & (degree == 1)
    raw_junctions = skeleton & (degree >= 3)

    # A real junction is often a 2x2/3x3 cluster.  Treat it as one node and
    # remove the complete cluster before extracting branch components.
    count_j, junction_labels, _, _ = cv2.connectedComponentsWithStats(
        raw_junctions.astype(np.uint8), connectivity=8)
    branch_pixels = skeleton & ~raw_junctions
    count_b, raw_branch_labels, stats_b, _ = cv2.connectedComponentsWithStats(
        branch_pixels.astype(np.uint8), connectivity=8)

    components = []
    for old_label in range(1, count_b):
        ys, xs = np.where(raw_branch_labels == old_label)
        if len(xs) < max(1, min_branch_cells):
            continue
        cells = list(zip(xs.tolist(), ys.tolist()))
        components.append((min((y, x) for x, y in cells), old_label, cells))
    components.sort(key=lambda item: item[0])

    labels = np.zeros_like(raw_branch_labels, dtype=np.int32)
    branches: List[SkeletonBranch] = []
    for branch_id, (_, old_label, cells) in enumerate(components, start=1):
        for x, y in cells:
            labels[y, x] = branch_id
        cx = int(round(sum(x for x, _ in cells) / float(len(cells))))
        cy = int(round(sum(y for _, y in cells) / float(len(cells))))
        endpoint_cells = [(x, y) for x, y in cells if endpoints[y, x]]
        adjacent = set()
        for x, y in cells:
            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    nx, ny = x + dx, y + dy
                    if (0 <= nx < labels.shape[1] and 0 <= ny < labels.shape[0]
                            and junction_labels[ny, nx] > 0):
                        adjacent.add(int(junction_labels[ny, nx]))
        branches.append(SkeletonBranch(
            branch_id=branch_id,
            cells=cells,
            length_m=max(0.0, (len(cells) - 1) * float(resolution)),
            centroid_cell=(cx, cy),
            endpoint_cells=endpoint_cells,
            adjacent_junction_ids=sorted(adjacent),
        ))
    return endpoints, raw_junctions, junction_labels.astype(np.int32), labels, branches


def _geodesic_branch_voronoi(passable: np.ndarray, branch_labels: np.ndarray,
                             resolution: float) -> Tuple[np.ndarray, np.ndarray]:
    """Label passable cells by nearest branch using multi-source Dijkstra."""
    height, width = passable.shape
    nearest = np.zeros((height, width), dtype=np.int32)
    distance = np.full((height, width), np.inf, dtype=np.float32)
    queue = []
    ys, xs = np.where(branch_labels > 0)
    seeds = sorted(zip(xs.tolist(), ys.tolist()),
                   key=lambda cell: (int(branch_labels[cell[1], cell[0]]), cell[1], cell[0]))
    for x, y in seeds:
        nearest[y, x] = int(branch_labels[y, x])
        distance[y, x] = 0.0
        heapq.heappush(queue, (0.0, int(branch_labels[y, x]), x, y))
    neighbours = ((1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
                  (1, 1, math.sqrt(2.0)), (1, -1, math.sqrt(2.0)),
                  (-1, 1, math.sqrt(2.0)), (-1, -1, math.sqrt(2.0)))
    while queue:
        current_distance, source, x, y = heapq.heappop(queue)
        if current_distance > float(distance[y, x]) + 1e-6:
            continue
        if source != int(nearest[y, x]):
            continue
        for dx, dy, step_cost in neighbours:
            nx, ny = x + dx, y + dy
            if not (0 <= nx < width and 0 <= ny < height and passable[ny, nx]):
                continue
            # Match frontier_core.astar(): never cut through obstacle corners.
            if dx != 0 and dy != 0 and (not passable[y, nx] or not passable[ny, x]):
                continue
            candidate_distance = current_distance + step_cost * float(resolution)
            if candidate_distance + 1e-6 < float(distance[ny, nx]):
                distance[ny, nx] = candidate_distance
                nearest[ny, nx] = source
                heapq.heappush(queue, (candidate_distance, source, nx, ny))
            elif (abs(candidate_distance - float(distance[ny, nx])) <= 1e-6
                  and source < int(nearest[ny, nx])):
                nearest[ny, nx] = source
                heapq.heappush(queue, (candidate_distance, source, nx, ny))
    return nearest, distance


def build_free_space_topology(passable: np.ndarray, resolution: float,
                              layer_id: str,
                              min_free_component_area_m2: float = 0.8,
                              spur_prune_length_m: float = 0.6,
                              min_branch_length_m: float = 0.4,
                              thinning_max_iterations: int = 500) -> FreeSpaceTopology:
    """Extract a deterministic branch topology from an inflated free-space mask."""
    resolution = max(1e-6, float(resolution))
    min_area_cells = max(1, int(math.ceil(
        float(min_free_component_area_m2) / (resolution * resolution))))
    cleaned = _remove_small_components(passable, min_area_cells)
    skeleton = zhang_suen_thinning(cleaned, thinning_max_iterations)
    skeleton = prune_short_spurs(
        skeleton, max(0, int(round(float(spur_prune_length_m) / resolution))))
    min_branch_cells = max(1, int(round(float(min_branch_length_m) / resolution)))
    endpoints, junctions, junction_labels, branch_labels, branches = _split_branches(
        skeleton, resolution, min_branch_cells)
    nearest, distance = _geodesic_branch_voronoi(cleaned, branch_labels, resolution)
    return FreeSpaceTopology(
        layer_id=str(layer_id),
        skeleton_mask=skeleton,
        endpoint_mask=endpoints,
        junction_mask=junctions,
        junction_labels=junction_labels,
        branch_labels=branch_labels,
        nearest_branch_labels=nearest,
        branch_distance_m=distance,
        branches=branches,
    )


def associate_frontier_clusters(clusters: Sequence[object], topology: FreeSpaceTopology,
                                max_distance_m: float = 4.0,
                                high_confidence_distance_m: float = 1.5) -> Dict[int, Dict[str, object]]:
    """Return topology metadata indexed by a frontier cluster's ``cluster_id``."""
    associations: Dict[int, Dict[str, object]] = {}
    height, width = topology.nearest_branch_labels.shape
    for cluster in clusters:
        cluster_id = int(getattr(cluster, "cluster_id"))
        x, y = getattr(cluster, "viewpoint_cell")
        branch_id = 0
        distance = float("inf")
        if 0 <= x < width and 0 <= y < height:
            branch_id = int(topology.nearest_branch_labels[y, x])
            distance = float(topology.branch_distance_m[y, x])
        valid = branch_id > 0 and math.isfinite(distance) and distance <= float(max_distance_m)
        if valid:
            region_id = "%s:R%03d" % (topology.layer_id, branch_id)
            confidence = "HIGH" if distance <= float(high_confidence_distance_m) else "MEDIUM"
            resolved_branch: Optional[int] = branch_id
        else:
            # Keep unassigned frontiers distinct; a shared UNASSIGNED label would
            # incorrectly make unrelated candidates mutually exclusive.
            region_id = "%s:UNASSIGNED:F%03d" % (topology.layer_id, cluster_id)
            confidence = "LOW"
            resolved_branch = None
        associations[cluster_id] = {
            "topology_region_id": region_id,
            "topology_branch_id": resolved_branch,
            "topology_layer": topology.layer_id,
            "topology_association_distance_m": (
                round(distance, 3) if math.isfinite(distance) else None),
            "topology_confidence": confidence,
        }
    return associations


def topology_summary(topology: FreeSpaceTopology,
                     associations: Dict[int, Dict[str, object]]) -> Dict[str, int]:
    regions = {str(item["topology_region_id"]) for item in associations.values()
               if item.get("topology_confidence") != "LOW"}
    return {
        "skeleton_cells": int(np.count_nonzero(topology.skeleton_mask)),
        "branches": len(topology.branches),
        "frontiers": len(associations),
        "regions": len(regions),
        "unassigned": sum(1 for item in associations.values()
                          if item.get("topology_confidence") == "LOW"),
    }
