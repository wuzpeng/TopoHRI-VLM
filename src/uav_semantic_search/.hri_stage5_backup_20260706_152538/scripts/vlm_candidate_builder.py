#!/usr/bin/env python3
"""Geometry-safe candidate generation for Stage-5 central VLM planning.

This module retains FUEL-style frontier extraction only as a *feasibility and
candidate-generation primitive*. It never selects the final assignment utility;
the centralized VLM chooses among the resulting feasible candidates.
"""
from __future__ import annotations

import math
import os
import sys
from typing import Any, Dict, List, Optional, Tuple

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)

from vlm_common import GridView, grid_from_msg, pose_to_dict
from frontier_core import (astar, build_passable_mask, extract_frontier_clusters,
                           nearest_passable, path_length_m)


def _profile(robot: Dict[str, Any], racer: Dict[str, Any], heterogeneous: Dict[str, Any]) -> Dict[str, Any]:
    if robot.get("type") == "ugv":
        return {
            "occupied_threshold": int(racer.get("occupied_threshold", 65)),
            "inflation": float(heterogeneous.get("ugv_obstacle_inflation_m", 0.54)),
            "min_clearance": float(heterogeneous.get("ugv_min_clearance_m", 0.52)),
            "min_frontier": float(heterogeneous.get("ugv_min_frontier_length_m", 0.60)),
            "gain_radius": float(heterogeneous.get("ugv_gain_radius_m", 2.2)),
            "stride": int(heterogeneous.get("ugv_frontier_sample_stride", 1)),
            "hgrid": float(heterogeneous.get("ugv_hgrid_size_m", 3.0)),
            "nearest": int(heterogeneous.get("ugv_nearest_free_search_cells", 20)),
            "max_exp": int(heterogeneous.get("ugv_astar_max_expansions", 30000)),
            "max_path": float(heterogeneous.get("ugv_max_assignment_path_m", 24.0)),
            "gain_weight": float(heterogeneous.get("ugv_gain_weight", 0.055)),
            "frontier_weight": float(heterogeneous.get("ugv_frontier_weight", 1.00)),
            "path_cost_weight": float(heterogeneous.get("ugv_path_cost_weight", 1.00)),
            "risk_weight": float(heterogeneous.get("ugv_risk_weight", 7.00)),
        }
    return {
        "occupied_threshold": int(racer.get("occupied_threshold", 65)),
        "inflation": float(racer.get("obstacle_inflation_m", 0.55)),
        "min_clearance": float(racer.get("min_clearance_m", 0.60)),
        "min_frontier": float(racer.get("min_frontier_length_m", 0.60)),
        "gain_radius": float(racer.get("gain_radius_m", 3.20)),
        "stride": int(racer.get("frontier_sample_stride", 2)),
        "hgrid": float(racer.get("hgrid_size_m", 4.0)),
        "nearest": int(racer.get("nearest_free_search_cells", 25)),
        "max_exp": int(racer.get("astar_max_expansions", 30000)),
        "max_path": float(racer.get("max_assignment_path_m", 35.0)),
        "gain_weight": float(racer.get("gain_weight", 0.040)),
        "frontier_weight": float(racer.get("frontier_weight", 1.20)),
        "path_cost_weight": float(racer.get("path_cost_weight", 1.00)),
        "risk_weight": float(racer.get("risk_weight", 6.00)),
    }


def _robot_goal_z(robot: Dict[str, Any], current_pose: Any) -> float:
    if robot.get("type") == "ugv":
        return float(robot.get("base_height_m", 0.34))
    if current_pose is not None:
        return float(current_pose.pose.position.z)
    return float(robot.get("takeoff_height", 1.8))


def _yaw_to(src: Tuple[float, float], dst: Tuple[float, float]) -> float:
    return float(math.atan2(dst[1] - src[1], dst[0] - src[0]))


def _grid_from_any(msg: Any) -> Optional[GridView]:
    return grid_from_msg(msg)

def _target_confidence(obj: Dict[str, Any]) -> float:
    return float(
        obj.get(
            "target_confidence",
            obj.get("confidence", 0.0),
        ) or 0.0
    )

def _target_state(obj: Dict[str, Any]) -> str:
    return str(
        obj.get(
            "target_state",
            obj.get("state", ""),
        )
    ).upper()

def _is_suspected_target(
        obj: Dict[str, Any],
        min_confidence: float
) -> bool:
    label = str(obj.get("label", "")).lower()
    category = str(obj.get("category", "")).lower()

    is_query_target = (
        label == "target_candidate"
        or category == "query_target"
    )

    state = _target_state(obj)
    confidence = _target_confidence(obj)

    # state 缺失时兼容旧语义地图；
    # 明确 NONE / UNKNOWN 的对象不应作为目标候选。
    valid_state = state not in ("NONE", "UNKNOWN")

    return (
        is_query_target
        and valid_state
        and confidence >= min_confidence
    )

def _target_state_rank(state: str) -> int:
    return {
        "NONE": 0,
        "POSSIBLE": 1,
        "LIKELY": 2,
        "CONFIRMED": 3,
    }.get(str(state).upper(), 1)

def _catalog_sort_key(candidate: Dict[str, Any]):
    """Hard candidate-tier ordering.

    Tier 0: target candidate
    Tier 1: FUEL-style exploration frontier
    Tier 2: ordinary semantic inspection
    Tier 3: scan/hold
    """

    tier = int(candidate.get("priority_tier", 99))

    target_conf = float(
        candidate.get("target_confidence", 0.0) or 0.0
    )

    target_state = _target_state_rank(
        candidate.get("target_state", "")
    )

    utility = float(
        candidate.get("frontier_utility", -1e9) or -1e9
    )

    confidence = float(
        candidate.get("confidence", 0.0) or 0.0
    )

    risk = float(
        candidate.get("risk", 0.0) or 0.0
    )

    path_length = float(
        candidate.get("path_length_m", 1e9) or 1e9
    )

    if tier == 0:
        return (
            0,
            -target_state,
            -target_conf,
            risk,
            path_length,
        )

    if tier == 1:
        return (
            1,
            -utility,
            risk,
            path_length,
        )

    if tier == 2:
        return (
            2,
            -confidence,
            risk,
            path_length,
        )

    return (
        3,
        path_length,
    )

def _object_candidates(
        robot: Dict[str, Any],
        grid: GridView,
        passable,
        start_cell,
        current_pose: Any,
        overlay: Dict[str, Any],
        profile: Dict[str, Any],
        max_objects: int,
        target_min_confidence: float,
        ground_verify_min_confidence: float
) -> List[Dict[str, Any]]:
    """Generate Tier-0 target candidates and Tier-2 generic semantic candidates."""

    candidates: List[Dict[str, Any]] = []

    if current_pose is None:
        return candidates

    objects = [
        obj for obj in overlay.get("objects", [])
        if isinstance(obj, dict)
    ]

    # 先检查真正的 target_candidate，避免被普通 obstacle 的插入顺序淹没。
    objects.sort(
        key=lambda obj: (
            0 if _is_suspected_target(
                obj,
                target_min_confidence,
            ) else 1,
            -_target_confidence(obj),
            -float(obj.get("confidence", 0.0) or 0.0),
        )
    )

    for obj in objects[:max(0, int(max_objects))]:
        pos = obj.get("position_map") or {}

        if (
            not isinstance(pos, dict)
            or "x" not in pos
            or "y" not in pos
        ):
            continue

        cell = grid.world_to_cell(
            float(pos["x"]),
            float(pos["y"]),
        )

        if cell is None:
            continue

        goal_cell = nearest_passable(
            passable,
            cell,
            profile["nearest"],
        )

        if goal_cell is None:
            continue

        path = astar(
            passable,
            start_cell,
            goal_cell,
            profile["max_exp"],
        )

        if path is None:
            continue

        plen = path_length_m(
            path,
            grid.resolution,
        )

        if plen > profile["max_path"]:
            continue

        gx, gy = grid.cell_to_world(goal_cell)

        label = str(obj.get("label", "object"))
        semantic_confidence = float(
            obj.get("confidence", 0.0) or 0.0
        )

        target_confidence = _target_confidence(obj)
        target_state = _target_state(obj)

        is_target = _is_suspected_target(
            obj,
            target_min_confidence,
        )

        if is_target:
            task_type = (
                "GROUND_VERIFY"
                if (
                    robot.get("type") == "ugv"
                    and target_confidence
                    >= ground_verify_min_confidence
                )
                else "INSPECT"
            )

            priority_tier = 0
            candidate_class = "TARGET"

        else:
            task_type = "INSPECT"
            priority_tier = 2
            candidate_class = "SEMANTIC_OBJECT"

        candidates.append({
            "id": "%s_OBJ_%s_%s" % (
                robot["name"],
                str(obj.get("object_id", "x")),
                task_type,
            ),
            "robot_id": robot["name"],
            "robot_type": robot.get("type"),

            "task_type": task_type,
            "priority_tier": priority_tier,
            "candidate_class": candidate_class,

            "semantic_anchor": {
                "object_id": obj.get("object_id"),
                "label": label,
                "appearance_color": obj.get(
                    "appearance_color",
                    "unknown",
                ),
                "navigation_effect": obj.get(
                    "navigation_effect",
                    "none",
                ),
            },

            "target_confidence": round(
                target_confidence,
                3,
            ),
            "target_state": (
                target_state
                if is_target
                else "NONE"
            ),

            "confidence": round(
                semantic_confidence,
                3,
            ),

            "goal": {
                "x": round(gx, 3),
                "y": round(gy, 3),
                "z": round(
                    _robot_goal_z(
                        robot,
                        current_pose,
                    ),
                    3,
                ),
                "yaw_rad": 0.0,
            },

            "path_length_m": round(
                plen,
                3,
            ),

            "information_gain": 0.0,
            "frontier_utility": 0.0,
            "risk": 0.0,

            "reason": (
                "localized suspected target verification point"
                if is_target
                else "reachable inspection point for ordinary semantic object %s"
                % label
            ),
        })

    return candidates

def build_candidates(robots: List[Dict[str, Any]], maps: Dict[str, Any], poses: Dict[str, Any],
                     overlay: Dict[str, Any], racer: Dict[str, Any], heterogeneous: Dict[str, Any],
                     cfg: Dict[str, Any]) -> List[Dict[str, Any]]:
    """Return safe candidate catalog for VLM selection.

    `maps` uses keys `uav` and `ugv`, matching robot-specific geometry layers.
    """
    catalog: List[Dict[str, Any]] = []
    max_per_robot = int(cfg.get("max_candidates_per_robot", 6))
    for robot in robots:
        rid = robot.get("name")
        map_key = "ugv" if robot.get("type") == "ugv" else "uav"
        grid = _grid_from_any(maps.get(map_key))
        pose = poses.get(rid)
        if grid is None or pose is None:
            continue
        profile = _profile(robot, racer, heterogeneous)
        passable, _ = build_passable_mask(grid, profile["occupied_threshold"], profile["inflation"])
        start = grid.world_to_cell(pose.pose.position.x, pose.pose.position.y)
        if start is None:
            continue
        start = nearest_passable(passable, start, profile["nearest"])
        if start is None:
            continue

        frontier_candidates: List[Dict[str, Any]] = []
        max_frontiers = int(
            cfg.get(
                "max_frontier_candidates_per_robot",
                max_per_robot,
            )
        )
        max_targets = int(
            cfg.get(
                "max_target_candidates_per_robot",
                2,
            )
        )
        max_generic_inspections = int(
            cfg.get(
                "max_generic_inspection_candidates_per_robot",
                2,
            )
        )
        target_min_confidence = float(
            cfg.get(
                "target_candidate_min_confidence",
                0.45,
            )
        )
        ground_verify_min_confidence = float(
            cfg.get(
                "ground_verify_min_confidence",
                0.75,
            )
        )

        clusters, _ = extract_frontier_clusters(
            grid,
            occupied_threshold=profile["occupied_threshold"],
            inflation_radius_m=profile["inflation"],
            min_frontier_length_m=profile["min_frontier"],
            gain_radius_m=profile["gain_radius"],
            min_clearance_m=profile["min_clearance"],
            hgrid_size_m=profile["hgrid"],
            sample_stride=profile["stride"],
        )
        sx, sy = pose.pose.position.x, pose.pose.position.y
        # Candidate-order is only a bounded pre-filter; final VLM selection is
        # deliberately not this score.
        clusters.sort(key=lambda c: (-float(c.information_gain), float(c.risk)))
        for cluster in clusters:
            path = astar(passable, start, cluster.viewpoint_cell, profile["max_exp"])
            if path is None:
                continue
            plen = path_length_m(path, grid.resolution)
            if plen > profile["max_path"]:
                continue
            gx, gy = grid.cell_to_world(cluster.viewpoint_cell)
            frontier_length = float(
                cluster.frontier_length_m
            )

            frontier_utility = (
                profile["gain_weight"]
                * float(cluster.information_gain)
                + profile["frontier_weight"]
                * frontier_length
                - profile["path_cost_weight"]
                * plen
                - profile["risk_weight"]
                * float(cluster.risk)
            )

            frontier_candidates.append({
                "id": "%s_F_%d" % (rid, int(cluster.cluster_id)),
                "robot_id": rid,
                "robot_type": robot.get("type"),
                "task_type": "EXPLORE",
                "semantic_anchor": None,
                "goal": {"x": round(gx, 3), "y": round(gy, 3), "z": round(_robot_goal_z(robot, pose), 3),
                         "yaw_rad": 0.0},
                "path_length_m": round(plen, 3),
                "information_gain": round(float(cluster.information_gain), 2),
                "risk": round(float(cluster.risk), 3),
                "reason": "safe frontier viewpoint generated from robot-specific occupancy map",
                "priority_tier": 1,
                "candidate_class": "FRONTIER",
                "target_confidence": 0.0,
                "target_state": "NONE",
                "confidence": 0.0,

                "frontier_length_m": round(frontier_length, 3,),
                "frontier_utility": round(frontier_utility, 3,),
            })
            # if len(local) >= max_per_robot:
            #     break
        
        # 所有安全可达 frontier 都完成 A* 可达性检查后，
        # 再按照 utility 进行排序和截断。
        frontier_candidates.sort(
            key=lambda candidate: (
                -float(
                    candidate.get(
                        "frontier_utility",
                        -1e9,
                    )
                ),
                float(candidate.get("risk", 0.0)),
                float(
                    candidate.get(
                        "path_length_m",
                        1e9,
                    )
                ),
            )
        )

        frontier_candidates = frontier_candidates[
            :max_frontiers
        ]

        object_candidates: List[Dict[str, Any]] = []

        if bool(
            cfg.get(
                "include_semantic_inspection",
                True,
            )
        ):
            object_candidates = _object_candidates(
                robot,
                grid,
                passable,
                start,
                pose,
                overlay,
                profile,
                int(
                    cfg.get(
                        "max_inspection_objects",
                        16,
                    )
                ),
                target_min_confidence,
                ground_verify_min_confidence,
            )

        target_candidates = [
            candidate
            for candidate in object_candidates
            if int(candidate.get("priority_tier", 99)) == 0
        ]

        generic_candidates = [
            candidate
            for candidate in object_candidates
            if int(candidate.get("priority_tier", 99)) == 2
        ]

        target_candidates.sort(
            key=_catalog_sort_key
        )

        generic_candidates.sort(
            key=_catalog_sort_key
        )

        # 关键顺序：
        # Tier 0 target
        # → Tier 1 frontier
        # → Tier 2 ordinary semantic objects
        local = (
            target_candidates[:max_targets]
            + frontier_candidates
        )

        if bool(
            cfg.get(
                "include_generic_semantic_inspection",
                True,
            )
        ):
            local.extend(
                generic_candidates[
                    :max_generic_inspections
                ]
            )

        if bool(
            cfg.get(
                "include_scan_in_place",
                True,
            )
        ):
            p = pose.pose.position

            local.append({
                "id": "%s_SCAN_0" % rid,
                "robot_id": rid,
                "robot_type": robot.get("type"),

                "task_type": (
                    "HOVER_AND_SCAN"
                    if robot.get("type") == "uav"
                    else "SCAN_IN_PLACE"
                ),

                "priority_tier": 3,
                "candidate_class": "SCAN",

                "semantic_anchor": None,
                "target_confidence": 0.0,
                "target_state": "NONE",
                "confidence": 0.0,

                "goal": {
                    "x": round(float(p.x), 3),
                    "y": round(float(p.y), 3),
                    "z": round(
                        _robot_goal_z(robot, pose),
                        3,
                    ),
                    "yaw_rad": 0.0,
                },

                "path_length_m": 0.0,
                "information_gain": 0.0,
                "frontier_utility": 0.0,
                "risk": 0.0,

                "reason": (
                    "safe in-place active observation candidate"
                ),
            })

        # 最终硬优先级排序。
        # 普通 obstacle INSPECT 永远不会排到 EXPLORE 前面。
        local.sort(
            key=_catalog_sort_key
        )

        catalog.extend(
            local[:max_per_robot]
        )

    return catalog[:int(cfg.get("max_total_candidates", 18))]
