#!/usr/bin/env python3
"""Shared utilities for the Stage-5 synchronous VLM semantic-search stack.

The module deliberately uses only Python stdlib, NumPy, OpenCV, rospy and the
ROS dependencies already present in the Stage-4 baseline.  Vision-language
network access is optional and uses a generic OpenAI-compatible chat-completions
HTTP interface, so the system still runs in deterministic mock mode without any
external model server.
"""
from __future__ import annotations

import base64
import json
import math
import os
import re
import time
import urllib.error
import urllib.request
from dataclasses import dataclass
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

import cv2
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from tf.transformations import euler_matrix, quaternion_matrix, euler_from_quaternion


JSONDict = Dict[str, Any]


def now_wall() -> float:
    """Wall-clock time for logic that must continue while Gazebo is paused."""
    return time.monotonic()


def safe_json_loads(text: str, default: Optional[Any] = None) -> Any:
    if text is None:
        return default
    try:
        return json.loads(text)
    except Exception:
        return default


def compact_json(value: Any) -> str:
    return json.dumps(value, ensure_ascii=False, separators=(",", ":"), sort_keys=False)


def extract_json_object(text: str) -> Optional[JSONDict]:
    """Extract the first valid JSON object from a model response."""
    if not text:
        return None
    direct = safe_json_loads(text)
    if isinstance(direct, dict):
        return direct
    cleaned = re.sub(r"```(?:json)?", "", text, flags=re.IGNORECASE).replace("```", "")
    decoder = json.JSONDecoder()
    for i, char in enumerate(cleaned):
        if char != "{":
            continue
        try:
            obj, _ = decoder.raw_decode(cleaned[i:])
            if isinstance(obj, dict):
                return obj
        except Exception:
            continue
    return None


def image_to_data_url(bgr: np.ndarray, quality: int = 85) -> str:
    quality = max(20, min(100, int(quality)))
    ok, encoded = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
    if not ok:
        raise RuntimeError("Cannot JPEG-encode image for VLM request.")
    payload = base64.b64encode(encoded.tobytes()).decode("ascii")
    return "data:image/jpeg;base64," + payload


def yaw_from_pose(msg: PoseStamped) -> float:
    q = msg.pose.orientation
    return float(euler_from_quaternion([q.x, q.y, q.z, q.w])[2])


def pose_to_dict(msg: Optional[PoseStamped]) -> Optional[JSONDict]:
    if msg is None:
        return None
    p = msg.pose.position
    return {
        "x": round(float(p.x), 3),
        "y": round(float(p.y), 3),
        "z": round(float(p.z), 3),
        "yaw_rad": round(yaw_from_pose(msg), 4),
    }


def camera_to_map_matrix(robot_cfg: JSONDict, pose: PoseStamped) -> np.ndarray:
    camera_to_body = euler_matrix(
        *[float(v) for v in robot_cfg.get("camera_optical_to_body_rpy", [0.0, 0.0, 0.0])]
    )
    camera_to_body[:3, 3] = [float(v) for v in robot_cfg.get("camera_xyz", [0.0, 0.0, 0.0])]
    q = pose.pose.orientation
    body_to_map = quaternion_matrix([q.x, q.y, q.z, q.w])
    body_to_map[:3, 3] = [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]
    return body_to_map.dot(camera_to_body)


def valid_depth_m(depth: np.ndarray, u: int, v: int, radius: int,
                  min_depth: float, max_depth: float) -> Optional[float]:
    if depth is None or depth.ndim < 2:
        return None
    h, w = depth.shape[:2]
    if not (0 <= u < w and 0 <= v < h):
        return None
    r = max(0, int(radius))
    patch = depth[max(0, v-r):min(h, v+r+1), max(0, u-r):min(w, u+r+1)].astype(np.float32)
    if depth.dtype == np.uint16:
        patch *= 0.001
    values = patch[np.isfinite(patch)]
    values = values[(values >= float(min_depth)) & (values <= float(max_depth))]
    return None if values.size == 0 else float(np.median(values))


def pixel_depth_to_map(robot_cfg: JSONDict, pose: Optional[PoseStamped],
                       camera_info: Any, u: int, v: int, depth_m: float) -> Optional[Tuple[float, float, float]]:
    if pose is None or camera_info is None or depth_m is None:
        return None
    if len(camera_info.K) < 9 or abs(float(camera_info.K[0])) < 1e-6 or abs(float(camera_info.K[4])) < 1e-6:
        return None
    fx, fy = float(camera_info.K[0]), float(camera_info.K[4])
    cx, cy = float(camera_info.K[2]), float(camera_info.K[5])
    camera_point = np.array([
        (float(u) - cx) * float(depth_m) / fx,
        (float(v) - cy) * float(depth_m) / fy,
        float(depth_m),
        1.0,
    ], dtype=np.float64)
    point = camera_to_map_matrix(robot_cfg, pose).dot(camera_point)[:3]
    return float(point[0]), float(point[1]), float(point[2])


def image_quality_score(bgr: Optional[np.ndarray]) -> float:
    if bgr is None or bgr.size == 0:
        return 0.0
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    return float(cv2.Laplacian(gray, cv2.CV_64F).var())


def normalized_hsv_histogram(bgr: Optional[np.ndarray]) -> Optional[np.ndarray]:
    if bgr is None or bgr.size == 0:
        return None
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    hist = cv2.calcHist([hsv], [0, 1], None, [16, 8], [0, 180, 0, 256])
    hist = cv2.normalize(hist, None).flatten().astype(np.float32)
    norm = float(np.linalg.norm(hist))
    return hist / norm if norm > 1e-9 else hist


def histogram_novelty(hist_a: Optional[np.ndarray], hist_b: Optional[np.ndarray]) -> float:
    if hist_a is None or hist_b is None:
        return 1.0
    return float(max(0.0, 1.0 - np.clip(float(np.dot(hist_a, hist_b)), -1.0, 1.0)))


@dataclass
class GridView:
    data: np.ndarray
    resolution: float
    origin_x: float
    origin_y: float
    frame_id: str

    @property
    def width(self) -> int:
        return int(self.data.shape[1])

    @property
    def height(self) -> int:
        return int(self.data.shape[0])

    def inside(self, cell: Tuple[int, int]) -> bool:
        x, y = cell
        return 0 <= x < self.width and 0 <= y < self.height

    def world_to_cell(self, x: float, y: float) -> Optional[Tuple[int, int]]:
        ix = int(math.floor((float(x) - self.origin_x) / self.resolution))
        iy = int(math.floor((float(y) - self.origin_y) / self.resolution))
        return (ix, iy) if self.inside((ix, iy)) else None

    def cell_to_world(self, cell: Tuple[int, int]) -> Tuple[float, float]:
        x, y = cell
        return (self.origin_x + (x + 0.5) * self.resolution,
                self.origin_y + (y + 0.5) * self.resolution)


def grid_from_msg(msg: Optional[OccupancyGrid]) -> Optional[GridView]:
    if msg is None or not msg.data or msg.info.width <= 0 or msg.info.height <= 0:
        return None
    expected = int(msg.info.width) * int(msg.info.height)
    array = np.asarray(msg.data, dtype=np.int16)
    if array.size != expected:
        return None
    return GridView(array.reshape((int(msg.info.height), int(msg.info.width))),
                    float(msg.info.resolution), float(msg.info.origin.position.x),
                    float(msg.info.origin.position.y), msg.header.frame_id or "map")


def map_summary(grid: Optional[GridView]) -> JSONDict:
    if grid is None:
        return {"available": False}
    data = grid.data
    free = int((data == 0).sum())
    occupied = int((data >= 65).sum())
    unknown = int((data < 0).sum())
    return {
        "available": True,
        "resolution_m": round(float(grid.resolution), 3),
        "width": int(grid.width),
        "height": int(grid.height),
        "free_cells": free,
        "occupied_cells": occupied,
        "unknown_cells": unknown,
    }


def sampled_depth_coverage_cells(robot_cfg: JSONDict, pose: Optional[PoseStamped], camera_info: Any,
                                 depth: Optional[np.ndarray], grid: Optional[GridView], stride: int,
                                 min_depth: float, max_depth: float, max_cells: int) -> List[List[int]]:
    """Project sparse valid depth samples to map cells for semantic-coverage bookkeeping."""
    if pose is None or camera_info is None or depth is None or grid is None:
        return []
    if len(camera_info.K) < 9 or abs(float(camera_info.K[0])) < 1e-6 or abs(float(camera_info.K[4])) < 1e-6:
        return []
    fx, fy, cx, cy = float(camera_info.K[0]), float(camera_info.K[4]), float(camera_info.K[2]), float(camera_info.K[5])
    transform = camera_to_map_matrix(robot_cfg, pose)
    h, w = depth.shape[:2]
    if depth.dtype == np.uint16:
        dimg = depth.astype(np.float32) * 0.001
    else:
        dimg = depth.astype(np.float32)
    found = set()
    step = max(4, int(stride))
    for v in range(step // 2, h, step):
        for u in range(step // 2, w, step):
            d = float(dimg[v, u])
            if not math.isfinite(d) or d < min_depth or d > max_depth:
                continue
            point = transform.dot(np.array([(u - cx) * d / fx, (v - cy) * d / fy, d, 1.0]))
            cell = grid.world_to_cell(float(point[0]), float(point[1]))
            if cell is not None:
                found.add(cell)
            if len(found) >= int(max_cells):
                return [[int(a), int(b)] for a, b in sorted(found)]
    return [[int(a), int(b)] for a, b in sorted(found)]


def response_content(payload: JSONDict) -> str:
    try:
        content = payload["choices"][0]["message"]["content"]
    except Exception:
        return ""
    if isinstance(content, str):
        return content
    if isinstance(content, list):
        pieces: List[str] = []
        for item in content:
            if isinstance(item, dict) and "text" in item:
                pieces.append(str(item["text"]))
        return "\n".join(pieces)
    return str(content)


class OpenAICompatibleVisionClient:
    """Small dependency-free client for an OpenAI-compatible multimodal endpoint."""
    def __init__(self, cfg: JSONDict):
        self.cfg = cfg

    def _endpoint(self) -> str:
        base = str(self.cfg.get("base_url", "")).rstrip("/")
        return base if base.endswith("/chat/completions") else base + "/chat/completions"

    def complete_json(self, system_prompt: str, user_prompt: str,
                      image_bgr: Optional[np.ndarray] = None,
                      jpeg_quality: int = 85,
                      timeout_sec: Optional[float] = None) -> JSONDict:
        # Direct api_key in vlm_semantic_search.yaml takes precedence.  The
        # api_key_env path is retained as an optional fallback for shared or
        # production deployments that should avoid storing secrets in config.
        direct_key = str(self.cfg.get("api_key", "") or "").strip()
        api_key = direct_key or os.environ.get(
            str(self.cfg.get("api_key_env", "VLM_API_KEY")), ""
        ).strip()
        content: List[JSONDict] = [{"type": "text", "text": user_prompt}]
        if image_bgr is not None:
            content.append({"type": "image_url", "image_url": {"url": image_to_data_url(image_bgr, jpeg_quality)}})
        payload: JSONDict = {
            "model": str(self.cfg.get("model", "")),
            "temperature": float(self.cfg.get("temperature", 0.0)),
            "max_tokens": int(self.cfg.get("max_tokens", 1200)),
            "messages": [
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": content},
            ],
        }
        request = urllib.request.Request(
            self._endpoint(),
            data=json.dumps(payload).encode("utf-8"),
            headers={
                "Content-Type": "application/json",
                **({"Authorization": "Bearer " + api_key} if api_key else {}),
            },
            method="POST",
        )
        request_timeout = float(self.cfg.get("timeout_sec", 20.0)) if timeout_sec is None else float(timeout_sec)
        request_timeout = max(0.1, request_timeout)
        try:
            with urllib.request.urlopen(request, timeout=request_timeout) as resp:
                raw = json.loads(resp.read().decode("utf-8"))
        except urllib.error.HTTPError as exc:
            raise RuntimeError("VLM HTTP %s: %s" % (exc.code, exc.read().decode("utf-8", errors="ignore")[:500]))
        except Exception as exc:
            raise RuntimeError("VLM request failed: %r" % (exc,))
        result = extract_json_object(response_content(raw))
        if result is None:
            raise RuntimeError("VLM response does not contain a valid JSON object.")
        return result
        
        # raw_content = response_content(raw)

        # # 临时调试：打印第三方 VLM 接口返回的原始文本。
        # # repr() 会显式保留空格、换行和转义字符，便于准确判断。
        # if str(os.environ.get("VLM_DEBUG_RAW_RESPONSE", "0")).lower() in (
        #         "1", "true", "yes"):
        #     print(
        #         "\n========== RAW VLM RESPONSE ==========\n%s\n"
        #         "======================================\n"
        #         % repr(raw_content),
        #         flush=True,
        #     )

        # result = extract_json_object(raw_content)

        # if str(os.environ.get("VLM_DEBUG_RAW_RESPONSE", "0")).lower() in (
        #         "1", "true", "yes"):
        #     print(
        #         "\n======= PARSED VLM SCENE SUMMARY =======\n%s\n"
        #         "========================================\n"
        #         % repr(
        #             result.get("scene_summary", "")
        #             if isinstance(result, dict) else None
        #         ),
        #         flush=True,
        #     )

        # if result is None:
        #     raise RuntimeError("VLM response does not contain a valid JSON object.")
        # return result


def normalize_bbox(value: Any, width: int, height: int) -> Optional[List[int]]:
    if not isinstance(value, (list, tuple)) or len(value) != 4:
        return None
    try:
        x0, y0, x1, y1 = [int(round(float(x))) for x in value]
    except Exception:
        return None
    x0, x1 = sorted((max(0, min(width - 1, x0)), max(0, min(width - 1, x1))))
    y0, y1 = sorted((max(0, min(height - 1, y0)), max(0, min(height - 1, y1))))
    if x1 <= x0 or y1 <= y0:
        return None
    return [x0, y0, x1, y1]


def mock_local_report(query: JSONDict, image_shape: Tuple[int, int], quality: float) -> JSONDict:
    h, w = image_shape
    return {
        "scene_summary": "Mock local VLM report: visual-semantic endpoint is not configured; geometry and coverage metadata remain active.",
        "entities": [],
        "target_evidence": {
            "state": "UNKNOWN",
            "confidence": 0.0,
            "bbox": None,
            "reason": "mock backend",
        },
        "observation_quality": min(1.0, float(quality) / 100.0),
        "requested_follow_up": "none",
        "image_size": {"width": int(w), "height": int(h)},
        "query_echo": query.get("query_text", ""),
    }


def mock_central_plan(candidates: List[JSONDict], robots: List[JSONDict], query: JSONDict,
                      overlay: JSONDict) -> JSONDict:
    assignments = []
    used = set()
    used_topology_regions = set()
    for robot in robots:
        rid = robot.get("id")
        options = [c for c in candidates if c.get("robot_id") == rid and c.get("id") not in used]
        # Mock backend follows the same coarse priority as the real planner:
        # target verification > human priority region/frontier > ordinary inspect > scan.
        def _rank(candidate):
            task = str(candidate.get("task_type", "")).upper()
            candidate_class = str(candidate.get("candidate_class", "")).upper()
            is_hri_region = int(
                task == "HRI_REGION_SEARCH"
                or candidate_class in ("HRI_REGION_SEARCH", "HRI_REGION_PERIMETER_SCAN")
            )
            topology_region = candidate.get("topology_region_id")
            topology_confidence = str(candidate.get("topology_confidence", "LOW")).upper()
            duplicate_topology_region = int(
                task == "EXPLORE"
                and topology_region
                and (topology_confidence in ("HIGH", "MEDIUM")
                     or ":UNASSIGNED:F" in str(topology_region))
                and str(topology_region) in used_topology_regions
            )
            return (
                int(candidate.get("priority_tier", 99)),
                -is_hri_region,
                -float(candidate.get("human_priority_score", 0.0) or 0.0),
                duplicate_topology_region,
                -float(candidate.get("frontier_utility", candidate.get("information_gain", 0.0)) or 0.0),
                float(candidate.get("risk", 0.0) or 0.0),
                float(candidate.get("path_length_m", 9999.0) or 9999.0),
            )

        options.sort(key=_rank)
        if not options:
            continue
        c = options[0]
        used.add(c["id"])
        if (str(c.get("task_type", "")).upper() == "EXPLORE"
                and c.get("topology_region_id")
                and (str(c.get("topology_confidence", "LOW")).upper() in ("HIGH", "MEDIUM")
                     or ":UNASSIGNED:F" in str(c.get("topology_region_id")))):
            used_topology_regions.add(str(c["topology_region_id"]))
        assignments.append({
            "robot_id": rid,
            "role": "GROUND_VERIFY" if c.get("task_type") == "GROUND_VERIFY" else
                    ("AERIAL_SCOUT" if robot.get("type") == "uav" else "GROUND_SCOUT"),
            "candidate_id": c["id"],
            "task_type": c.get("task_type", "EXPLORE"),
            "reason": "mock policy follows target, human-region, frontier, inspect priority",
        })
    return {
        "mission_mode": "EXPLORE",
        "assignments": assignments,
        "plan_valid_for_sec": 14.0,
        "reason": "mock central VLM fallback",
        "target_query": query.get("query_text", ""),
        "overlay_version": overlay.get("version", 0),
    }
