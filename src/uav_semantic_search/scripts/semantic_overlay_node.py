#!/usr/bin/env python3
"""Sparse semantic overlay in the shared map frame.

The overlay stores semantic metadata on top of existing UAV/UGV geometry maps;
it never changes OccupancyGrid values. It fuses local VLM reports by spatial
association and publishes a compact JSON summary for the central planner.
"""
from __future__ import annotations

import os
import sys
import threading
from typing import Any, Dict, List

import rospy
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)

from vlm_common import compact_json, safe_json_loads


class SemanticOverlay:
    def __init__(self):
        self.root = rospy.get_param('/vlm_semantic_search')
        self.cfg = self.root['semantic_overlay']
        self.lock = threading.RLock()
        self.version = 0
        self.cells: Dict[tuple, Dict[str, Any]] = {}
        self.objects: List[Dict[str, Any]] = []
        self.latest_reports: Dict[str, Dict[str, Any]] = {}

        # 新增：保存历史语义观测点。
        # 这些点用于目标描述切换后的 QUERY_RESCAN 候选生成。
        self.observation_history: List[Dict[str, Any]] = []
        self.observation_index = 0

        self.query: Dict[str, Any] = dict(self.root.get('target_query', {}))
        self.summary_pub = rospy.Publisher('/semantic_overlay/summary', String, queue_size=5, latch=True)
        self.marker_pub = rospy.Publisher('/semantic_overlay/markers', MarkerArray, queue_size=5, latch=True)
        rospy.Subscriber('/vlm/local_semantic_observation', String, self._observation_cb, queue_size=30)
        rospy.Subscriber('/vlm/target_query', String, self._query_cb, queue_size=3)
        rate = max(0.2, float(self.cfg.get('publish_rate_hz', 2.0)))
        rospy.Timer(rospy.Duration(1.0 / rate), self._publish)

    def _query_cb(self, msg):
        parsed = safe_json_loads(msg.data, None)
        if isinstance(parsed, dict):
            with self.lock:
                old_version = int(self.query.get('query_version', -1))
                new_version = int(parsed.get('query_version', old_version))
                self.query = parsed
                if new_version != old_version:
                    # Query-specific target evidence is invalid after the operator
                    # changes the target description.  Keep generic scene entities.
                    self.objects = [obj for obj in self.objects
                                    if str(obj.get('category', '')) != 'query_target'
                                    and str(obj.get('label', '')) != 'target_candidate']
                self.version += 1
                self._publish_locked()

    def _associate(self, obj: Dict[str, Any]) -> Dict[str, Any]:
        pos = obj.get('position_map') or {}
        if not isinstance(pos, dict) or 'x' not in pos or 'y' not in pos:
            return obj
        radius = float(self.cfg.get('association_radius_m', 1.25))
        for existing in self.objects:
            epos = existing.get('position_map') or {}
            if (
                existing.get('label') != obj.get('label')
                or existing.get('appearance_color', 'unknown')
                != obj.get('appearance_color', 'unknown')
                or (str(obj.get('label')) == 'target_candidate'
                    and int(existing.get('query_version', -1)) != int(obj.get('query_version', -2)))
                or 'x' not in epos
                or 'y' not in epos
            ):
                continue
            dx = float(epos['x']) - float(pos['x'])
            dy = float(epos['y']) - float(pos['y'])
            if dx*dx + dy*dy <= radius*radius:
                old_confidence = float(existing.get('confidence', 0.0))
                new_confidence = float(obj.get('confidence', 0.0))
                existing['confidence'] = max(old_confidence, new_confidence,)
                existing['target_confidence'] = max(
                    float(existing.get('target_confidence', 0.0)),
                    float(obj.get('target_confidence', 0.0)),
                )
                existing['observed_by'] = sorted(
                    set(existing.get('observed_by', []))
                    | set(obj.get('observed_by', []))
                )
                existing['last_epoch_id'] = obj.get('last_epoch_id')

                # target_candidate 融合时，保留更高等级的目标状态。
                if obj.get('label') == 'target_candidate':
                    state_rank = {
                        'NONE': 0,
                        'POSSIBLE': 1,
                        'LIKELY': 2,
                        'CONFIRMED': 3,
                    }

                    old_state = str(existing.get('target_state', 'POSSIBLE')).upper()
                    new_state = str(obj.get('target_state', 'POSSIBLE')).upper()
                    if state_rank.get(
                        new_state,
                        0,
                    ) >= state_rank.get(
                        old_state,
                        0,
                    ):
                        existing['target_state'] = new_state

                return existing
        self.objects.append(obj)
        max_objects = int(self.cfg.get('max_objects', 300))
        if len(self.objects) > max_objects:
            self.objects = self.objects[-max_objects:]
        return obj

    def _observation_cb(self, msg):
        report = safe_json_loads(msg.data, None)
        if not isinstance(report, dict) or report.get('status') != 'OK':
            return
        with self.lock:
            current_query_version = int(self.query.get('query_version', 0))
            report_query_version = int(report.get('query_version', -1))
            if report_query_version != current_query_version:
                rospy.logwarn('Ignored stale local VLM report from query v%d; active query is v%d.',
                              report_query_version, current_query_version)
                return
            self.latest_reports[report.get('robot_id', 'unknown')] = report
            for cell in report.get('coverage_cells', []):
                if isinstance(cell, list) and len(cell) == 2:
                    key = (int(cell[0]), int(cell[1]))
                    self.cells[key] = {
                        'semantic_covered': True,
                        'last_robot': report.get('robot_id'),
                        'query_version': report.get('query_version', 0),
                        'last_epoch_id': report.get('epoch_id'),
                    }
            target = report.get('target_evidence') or {}
            target_conf = float(target.get('confidence', 0.0)) if isinstance(target, dict) else 0.0
            # ------------------------------------------------------------
            # 新增：记录历史观测点。
            # 作用：目标描述切换后，在已探索区域中生成 QUERY_RESCAN 候选。
            # ------------------------------------------------------------
            pose_map = report.get('pose_map') or {}
            if (
                isinstance(pose_map, dict)
                and 'x' in pose_map
                and 'y' in pose_map
            ):
                self.observation_index += 1

                target_state = (
                    str(target.get('state', 'UNKNOWN')).upper()
                    if isinstance(target, dict)
                    else 'UNKNOWN'
                )

                history_item = {
                    'history_id': 'obs_%06d' % self.observation_index,
                    'robot_id': str(report.get('robot_id', 'unknown')),
                    'robot_type': str(report.get('robot_type', 'unknown')),
                    'pose_map': {
                        'x': float(pose_map.get('x', 0.0)),
                        'y': float(pose_map.get('y', 0.0)),
                        'z': float(pose_map.get('z', 0.0)),
                        'yaw_rad': float(pose_map.get('yaw_rad', 0.0)),
                    },
                    'query_version': int(report.get('query_version', 0)),
                    'epoch_id': report.get('epoch_id'),
                    'scene_summary': report.get('scene_summary', ''),
                    'coverage_cell_count': len(
                        report.get('coverage_cells', [])
                        if isinstance(report.get('coverage_cells', []), list)
                        else []
                    ),
                    'target_state': target_state,
                    'target_confidence': round(float(target_conf), 3),
                }

                self.observation_history.append(history_item)

                max_history = int(
                    self.cfg.get('max_observation_history', 300)
                )
                if len(self.observation_history) > max_history:
                    self.observation_history = self.observation_history[
                        -max_history:
                    ]
            for entity in report.get('entities', []):
                if not isinstance(entity, dict):
                    continue
                obj = dict(entity)
                obj['observed_by'] = [report.get('robot_id')]
                obj['last_epoch_id'] = report.get('epoch_id')
                obj['target_confidence'] = target_conf if entity.get('label') in ('person', 'other') else 0.0
                self._associate(obj)
            # Target evidence is added as a query-specific object if localized.
            if (
                isinstance(target, dict)
                and target.get('position_map')
                and target_conf > 0.0
            ):
                target_state = str(
                    target.get('state', 'POSSIBLE')
                ).upper()

                # 防止后端异常返回 UNKNOWN / NONE 但 confidence 非零时，
                # 仍然被显示为高可信目标。
                if target_state not in ('POSSIBLE', 'LIKELY'):
                    target_state = 'POSSIBLE'

                obj = {
                    'object_id': 'query_%s_%s' % (
                        report.get('query_version', 0),
                        report.get('robot_id'),
                    ),
                    'label': 'target_candidate',
                    'category': 'query_target',

                    # VLM 对目标匹配程度的置信度。
                    'confidence': target_conf,
                    'target_confidence': target_conf,

                    # 新增：保存 Local VLM 返回的 POSSIBLE / LIKELY。
                    'target_state': target_state,

                    'position_map': target.get('position_map'),
                    'observed_by': [report.get('robot_id')],
                    'last_epoch_id': report.get('epoch_id'),
                    'query_version': report_query_version,
                }

                self._associate(obj)
            self.version += 1
            self._publish_locked()

    def _summary_locked(self) -> Dict[str, Any]:
        reports = []
        for rid, rep in self.latest_reports.items():
            reports.append({
                'robot_id': rid,
                'scene_summary': rep.get('scene_summary', ''),
                'target_evidence': rep.get('target_evidence', {}),
                'requested_follow_up': rep.get('requested_follow_up', 'none'),
                'query_version': rep.get('query_version', 0),
            })

        max_history_in_summary = int(self.cfg.get('max_observation_history_in_summary', 120))
        max_cells_in_summary = int(self.cfg.get('max_semantic_coverage_cells_in_summary', 800))
        semantic_coverage_cells = []
        for (cx, cy), meta in list(self.cells.items())[-max_cells_in_summary:]:
            item = {
                'x': int(cx),
                'y': int(cy),
                'query_version': int(meta.get('query_version', 0)),
                'last_robot': meta.get('last_robot'),
                'last_epoch_id': meta.get('last_epoch_id'),
            }
            semantic_coverage_cells.append(item)

        return {
            'version': self.version,
            'query': self.query,
            'semantic_covered_cell_count': len(self.cells),
            'semantic_coverage_cells': semantic_coverage_cells,
            'objects': self.objects[-40:],
            'latest_reports': reports,

            # 新增：供 Central VLM 候选生成器使用。
            # 注意：这里不是给 VLM 直接推理用，而是给 vlm_candidate_builder
            # 在目标切换后生成 QUERY_RESCAN 安全候选点。
            'observation_history': self.observation_history[-max_history_in_summary:],
        }

    def _markers_locked(self) -> MarkerArray:
        arr = MarkerArray()
        now = rospy.Time.now()

        for i, obj in enumerate(self.objects[-40:]):
            pos = obj.get('position_map') or {}

            if 'x' not in pos or 'y' not in pos:
                continue

            is_target = (
                obj.get('label') == 'target_candidate'
            )

            # ------------------------------------------------------------
            # 1. 原有位置球形 Marker
            # ------------------------------------------------------------
            marker = Marker()
            marker.header.frame_id = self.root.get(
                'frame_id',
                'map',
            )
            marker.header.stamp = now

            marker.ns = 'semantic_overlay_points'
            marker.id = i

            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.pose.position.x = float(pos['x'])
            marker.pose.position.y = float(pos['y'])
            marker.pose.position.z = float(
                pos.get('z', 0.4)
            )

            marker.pose.orientation.w = 1.0

            # 目标红点比普通语义对象稍大。
            point_size = 0.34 if is_target else 0.28

            marker.scale.x = point_size
            marker.scale.y = point_size
            marker.scale.z = point_size

            marker.color.r = 1.0 if is_target else 0.15
            marker.color.g = 0.15 if is_target else 0.75
            marker.color.b = 0.15
            marker.color.a = 0.9

            marker.lifetime = rospy.Duration(
                float(
                    self.cfg.get(
                        'marker_lifetime_sec',
                        0.0,
                    )
                )
            )

            arr.markers.append(marker)

            # ------------------------------------------------------------
            # 2. 仅为 target_candidate 额外创建文字 Marker
            # ------------------------------------------------------------
            if not is_target:
                continue

            target_state = str(
                obj.get('target_state', 'POSSIBLE')
            ).upper()

            confidence = float(
                obj.get(
                    'target_confidence',
                    obj.get('confidence', 0.0),
                )
            )

            observed_by = obj.get(
                'observed_by',
                [],
            )

            observers_text = ','.join(
                str(item)
                for item in observed_by
            )

            # 文本显示内容。
            # 例如：
            # LIKELY | conf=0.84
            # seen by: uav1,ugv0
            label_text = (
                '%s | conf=%.2f\nseen by: %s'
                % (
                    target_state,
                    confidence,
                    observers_text or 'unknown',
                )
            )

            text_marker = Marker()
            text_marker.header.frame_id = self.root.get(
                'frame_id',
                'map',
            )
            text_marker.header.stamp = now

            # 使用独立 namespace 和较大 ID，避免与红点 Marker 冲突。
            text_marker.ns = 'semantic_overlay_target_text'
            text_marker.id = 10000 + i

            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD

            text_marker.pose.position.x = float(pos['x'])
            text_marker.pose.position.y = float(pos['y'])

            # 文字放到红点上方，避免和球体重叠。
            text_marker.pose.position.z = float(
                pos.get('z', 0.4)
            ) + 0.45

            text_marker.pose.orientation.w = 1.0

            # TEXT_VIEW_FACING 只使用 scale.z 作为字体大小。
            text_marker.scale.z = 0.22

            text_marker.text = label_text

            # POSSIBLE：黄色文字。
            # LIKELY：白色文字。
            if target_state == 'LIKELY':
                text_marker.color.r = 1.0
                text_marker.color.g = 1.0
                text_marker.color.b = 1.0
            else:
                text_marker.color.r = 1.0
                text_marker.color.g = 0.85
                text_marker.color.b = 0.10

            text_marker.color.a = 1.0

            text_marker.lifetime = rospy.Duration(
                float(
                    self.cfg.get(
                        'marker_lifetime_sec',
                        0.0,
                    )
                )
            )

            arr.markers.append(text_marker)

        return arr

    def _publish_locked(self):
        # Publish directly from callbacks as well as the periodic timer. Gazebo
        # simulated time is frozen during a VLM epoch, so rospy.Timer alone would
        # postpone fresh semantic evidence until after planning has already run.
        self.summary_pub.publish(compact_json(self._summary_locked()))
        self.marker_pub.publish(self._markers_locked())

    def _publish(self, _event):
        with self.lock:
            self._publish_locked()


def main():
    rospy.init_node('semantic_overlay_node')
    SemanticOverlay()
    rospy.spin()


if __name__ == '__main__':
    main()
