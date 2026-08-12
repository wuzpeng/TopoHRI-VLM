#!/usr/bin/env python3
"""Human–AI interaction state manager for Stage-5 VLM semantic search.

This node is the only bridge between the GUI and the autonomy stack.  Human
inputs are represented as structured, versioned *intent*; they never bypass
candidate validation, A* route planning, or low-level safety control.

Wire format
-----------
All HRI topics use ``std_msgs/String`` carrying compact JSON, so the extension
can be installed in the current ROS Noetic package without adding custom ROS
messages.

Inputs
------
/hri/set_priority_region       add/update a polygon priority region
/hri/remove_priority_region    remove one region by id
/hri/clear_priority_regions    remove all regions
/hri/set_target_instruction    replace target query or append a human clue
/hri/force_replan              request a fresh central allocation

Outputs
-------
/hri/shared_context            latched active human intent
/hri/replan_request            consumed by vlm_trigger_scheduler.py
/hri/decision_feedback         latched GUI-ready AI feedback
/hri/operator_action_log       experiment/audit log
"""
from __future__ import annotations

import copy
import math
import os
import sys
import threading
import time
import uuid
from typing import Any, Dict, List, Optional

import rospy
from std_msgs.msg import String

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)

from vlm_common import compact_json, safe_json_loads


class HumanInteractionManager:
    """Maintain human intent, trigger safe replanning, and aggregate feedback."""

    def __init__(self) -> None:
        self.lock = threading.RLock()
        self.root = rospy.get_param('/vlm_semantic_search', {})
        self.cfg = rospy.get_param('/human_ai_interaction', {})
        self.max_regions = max(1, int(self.cfg.get('max_regions', 12)))
        self.max_action_history = max(10, int(self.cfg.get('max_action_history', 100)))
        self.max_feedback_history = max(10, int(self.cfg.get('max_feedback_history', 80)))
        self.default_priority = float(self.cfg.get('default_region_priority', 0.80))
        self.default_mode = str(self.cfg.get('default_region_mode', 'soft')).lower()
        self.default_max_robots = max(1, int(self.cfg.get('default_max_robots', 1)))
        self.default_ttl_sec = max(0.0, float(self.cfg.get('default_region_ttl_sec', 0.0)))

        self.context_version = 0
        self.priority_regions: Dict[str, Dict[str, Any]] = {}
        self.current_query: Dict[str, Any] = dict(self.root.get('target_query', {}))
        self.action_history: List[Dict[str, Any]] = []
        self.feedback_history: List[Dict[str, Any]] = []
        self.latest_local_reports: Dict[str, Dict[str, Any]] = {}
        self.semantic_overlay: Dict[str, Any] = {'version': 0, 'objects': [], 'latest_reports': []}
        self.candidate_by_id: Dict[str, Dict[str, Any]] = {}
        self.current_assignments: Dict[str, Dict[str, Any]] = {}
        self.latest_central_plan: Dict[str, Any] = {}
        self.latest_validated_plan: Dict[str, Any] = {}
        self.route_status: Dict[str, Dict[str, Any]] = {}
        self.sync_status = 'INITIALIZING'
        self.trigger_status = 'INITIALIZING'

        self.context_pub = rospy.Publisher('/hri/shared_context', String, queue_size=5, latch=True)
        self.replan_pub = rospy.Publisher('/hri/replan_request', String, queue_size=10)
        self.feedback_pub = rospy.Publisher('/hri/decision_feedback', String, queue_size=10, latch=True)
        self.action_pub = rospy.Publisher('/hri/operator_action_log', String, queue_size=30)
        self.status_pub = rospy.Publisher('/hri/status', String, queue_size=5, latch=True)
        self.set_query_pub = rospy.Publisher('/vlm/set_target_query', String, queue_size=5)

        rospy.Subscriber('/hri/set_priority_region', String, self._set_priority_region_cb, queue_size=20)
        rospy.Subscriber('/hri/remove_priority_region', String, self._remove_priority_region_cb, queue_size=20)
        rospy.Subscriber('/hri/clear_priority_regions', String, self._clear_priority_regions_cb, queue_size=10)
        rospy.Subscriber('/hri/set_target_instruction', String, self._target_instruction_cb, queue_size=20)
        rospy.Subscriber('/hri/force_replan', String, self._force_replan_cb, queue_size=10)

        rospy.Subscriber('/vlm/target_query', String, self._query_cb, queue_size=10)
        rospy.Subscriber('/vlm/central_plan', String, self._central_plan_cb, queue_size=10)
        rospy.Subscriber('/vlm/validated_plan', String, self._validated_plan_cb, queue_size=10)
        rospy.Subscriber('/vlm/goal_dispatch', String, self._goal_dispatch_cb, queue_size=30)
        rospy.Subscriber('/vlm/route_result', String, self._route_result_cb, queue_size=30)
        rospy.Subscriber('/vlm/sync_status', String, self._sync_status_cb, queue_size=10)
        rospy.Subscriber('/vlm/trigger_status', String, self._trigger_status_cb, queue_size=10)
        rospy.Subscriber('/vlm/local_semantic_observation', String, self._local_report_cb, queue_size=30)
        rospy.Subscriber('/semantic_overlay/summary', String, self._semantic_overlay_cb, queue_size=10)

        self.expiry_timer = rospy.Timer(rospy.Duration(1.0), self._expiry_timer_cb)
        with self.lock:
            self._publish_context_locked()
            self._publish_feedback_locked()
        self.status_pub.publish('READY')
        rospy.loginfo('Human interaction manager ready. JSON HRI topics are active.')

    @staticmethod
    def _now() -> float:
        return time.time()

    @staticmethod
    def _as_float(value: Any, name: str) -> float:
        try:
            result = float(value)
        except (TypeError, ValueError):
            raise ValueError('%s must be numeric' % name)
        if not math.isfinite(result):
            raise ValueError('%s must be finite' % name)
        return result

    @classmethod
    def _normalize_polygon(cls, payload: Any) -> List[Dict[str, float]]:
        if not isinstance(payload, list) or len(payload) < 3:
            raise ValueError('polygon must contain at least three vertices')
        points: List[Dict[str, float]] = []
        for item in payload:
            if isinstance(item, dict):
                x = cls._as_float(item.get('x'), 'polygon.x')
                y = cls._as_float(item.get('y'), 'polygon.y')
            elif isinstance(item, (list, tuple)) and len(item) >= 2:
                x = cls._as_float(item[0], 'polygon[0]')
                y = cls._as_float(item[1], 'polygon[1]')
            else:
                raise ValueError('each polygon vertex must be {x,y} or [x,y]')
            if not points or abs(points[-1]['x'] - x) > 1e-6 or abs(points[-1]['y'] - y) > 1e-6:
                points.append({'x': round(x, 4), 'y': round(y, 4)})
        if len(points) >= 2 and abs(points[0]['x'] - points[-1]['x']) < 1e-6 and abs(points[0]['y'] - points[-1]['y']) < 1e-6:
            points.pop()
        if len(points) < 3:
            raise ValueError('polygon must contain at least three distinct vertices')
        area2 = 0.0
        for index, point in enumerate(points):
            nxt = points[(index + 1) % len(points)]
            area2 += point['x'] * nxt['y'] - nxt['x'] * point['y']
        if abs(area2) < 1e-4:
            raise ValueError('polygon area is too small')
        return points

    def _active_regions_locked(self) -> List[Dict[str, Any]]:
        now = self._now()
        active: List[Dict[str, Any]] = []
        for region in self.priority_regions.values():
            expires = float(region.get('expires_wall_time', 0.0) or 0.0)
            if expires > 0.0 and expires <= now:
                continue
            active.append(copy.deepcopy(region))
        active.sort(key=lambda item: (-float(item.get('priority', 0.0)), str(item.get('region_id', ''))))
        return active

    def _context_locked(self) -> Dict[str, Any]:
        return {
            'schema_version': 1,
            'context_version': int(self.context_version),
            'frame_id': str(self.cfg.get('frame_id', self.root.get('frame_id', 'map'))),
            'updated_wall_time': round(self._now(), 3),
            'priority_regions': self._active_regions_locked(),
            'target_query': copy.deepcopy(self.current_query),
            'operator_action_count': len(self.action_history),
        }

    def _feedback_locked(self) -> Dict[str, Any]:
        assignments = [copy.deepcopy(value) for _, value in sorted(self.current_assignments.items())]
        reports = [copy.deepcopy(value) for _, value in sorted(self.latest_local_reports.items())]
        return {
            'schema_version': 1,
            'context_version': int(self.context_version),
            'target_query': copy.deepcopy(self.current_query),
            'priority_regions': self._active_regions_locked(),
            'sync_status': self.sync_status,
            'trigger_status': self.trigger_status,
            'current_assignments': assignments,
            'route_status': copy.deepcopy(self.route_status),
            'latest_validated_plan': copy.deepcopy(self.latest_validated_plan),
            'latest_central_plan': self._compact_plan_locked(),
            'latest_local_reports': reports,
            'semantic_overlay': copy.deepcopy(self.semantic_overlay),
            'recent_events': copy.deepcopy(self.feedback_history[-self.max_feedback_history:]),
        }

    def _compact_plan_locked(self) -> Dict[str, Any]:
        if not isinstance(self.latest_central_plan, dict):
            return {}
        raw_plan = self.latest_central_plan.get('plan', {})
        if not isinstance(raw_plan, dict):
            raw_plan = {}
        assignments: List[Dict[str, Any]] = []
        for item in raw_plan.get('assignments', []) if isinstance(raw_plan.get('assignments', []), list) else []:
            if not isinstance(item, dict):
                continue
            candidate = self.candidate_by_id.get(str(item.get('candidate_id', '')), {})
            assignments.append({
                'robot_id': str(item.get('robot_id', '')),
                'candidate_id': str(item.get('candidate_id', '')),
                'task_type': item.get('task_type', candidate.get('task_type', '')),
                'role': item.get('role', ''),
                'reason': item.get('reason', ''),
                'goal': candidate.get('goal', {}),
                'human_priority_score': candidate.get('human_priority_score', 0.0),
                'human_priority_regions': candidate.get('human_priority_regions', []),
                'explanation': self._assignment_explanation(candidate, item),
            })
        return {
            'epoch_id': self.latest_central_plan.get('epoch_id'),
            'status': self.latest_central_plan.get('status'),
            'event_reason': self.latest_central_plan.get('event_reason'),
            'mission_mode': raw_plan.get('mission_mode'),
            'reason': raw_plan.get('reason', ''),
            'assignments': assignments,
        }

    @staticmethod
    def _assignment_explanation(candidate: Dict[str, Any], assignment: Dict[str, Any]) -> str:
        if not isinstance(candidate, dict):
            return str(assignment.get('reason', ''))
        regions = candidate.get('human_priority_regions', [])
        if isinstance(regions, list) and regions:
            labels = ', '.join(str(region.get('region_id', '')) for region in regions if isinstance(region, dict))
            return 'Selected candidate lies in operator priority region(s): %s. %s' % (
                labels or 'unknown', str(assignment.get('reason', '')))
        return str(assignment.get('reason', '')) or 'Selected from safe candidate catalog.'

    def _publish_context_locked(self) -> None:
        self.context_pub.publish(compact_json(self._context_locked()))

    def _publish_feedback_locked(self) -> None:
        self.feedback_pub.publish(compact_json(self._feedback_locked()))

    def _record_action_locked(self, action_type: str, payload: Dict[str, Any], outcome: str = 'accepted') -> None:
        event = {
            'event_id': 'hri_%s' % uuid.uuid4().hex[:10],
            'wall_time': round(self._now(), 3),
            'action_type': action_type,
            'outcome': outcome,
            'payload': copy.deepcopy(payload),
            'context_version': int(self.context_version),
        }
        self.action_history.append(event)
        self.action_history = self.action_history[-self.max_action_history:]
        self.feedback_history.append({'type': 'operator_action', **event})
        self.feedback_history = self.feedback_history[-self.max_feedback_history:]
        self.action_pub.publish(compact_json(event))

    def _record_feedback_locked(self, event_type: str, payload: Dict[str, Any]) -> None:
        self.feedback_history.append({
            'type': event_type,
            'wall_time': round(self._now(), 3),
            'payload': copy.deepcopy(payload),
        })
        self.feedback_history = self.feedback_history[-self.max_feedback_history:]

    def _request_replan_locked(self, reason: str, details: Optional[Dict[str, Any]] = None,
                               refresh_local_perception: bool = False) -> None:
        details = dict(details or {})
        details['human_context'] = self._context_locked()
        details['refresh_local_perception'] = bool(refresh_local_perception)
        details['context_version'] = int(self.context_version)
        self.replan_pub.publish(compact_json({
            'reason': reason,
            'details': details,
            'requested_wall_time': round(self._now(), 3),
        }))

    def _set_priority_region_cb(self, msg: String) -> None:
        payload = safe_json_loads(msg.data, None)
        if not isinstance(payload, dict):
            rospy.logwarn('Ignored invalid /hri/set_priority_region payload.')
            return
        try:
            polygon = self._normalize_polygon(payload.get('polygon', payload.get('points')))
            priority = min(1.0, max(0.0, self._as_float(payload.get('priority', self.default_priority), 'priority')))
            mode = str(payload.get('mode', self.default_mode)).lower()
            if mode not in ('soft', 'hard'):
                raise ValueError('mode must be soft or hard')
            max_robots = max(1, int(payload.get('max_robots', self.default_max_robots)))
            ttl_sec = max(0.0, self._as_float(payload.get('ttl_sec', self.default_ttl_sec), 'ttl_sec'))
        except (ValueError, TypeError) as exc:
            rospy.logwarn('Rejected priority region: %s', exc)
            return

        with self.lock:
            region_id = str(payload.get('region_id', '')).strip() or ('H%d' % (self.context_version + 1))
            if region_id not in self.priority_regions and len(self.priority_regions) >= self.max_regions:
                rospy.logwarn('Rejected priority region %s: max_regions=%d reached.', region_id, self.max_regions)
                return
            now = self._now()
            previous = self.priority_regions.get(region_id, {})
            region = {
                'region_id': region_id,
                'polygon': polygon,
                'priority': round(priority, 3),
                'mode': mode,
                'max_robots': max_robots,
                'ttl_sec': round(ttl_sec, 3),
                'created_wall_time': float(previous.get('created_wall_time', now)),
                'updated_wall_time': now,
                'expires_wall_time': round(now + ttl_sec, 3) if ttl_sec > 0.0 else 0.0,
                'operator_note': str(payload.get('operator_note', payload.get('note', ''))).strip(),
            }
            self.priority_regions[region_id] = region
            self.context_version += 1
            self._record_action_locked('SET_PRIORITY_REGION', region)
            self._publish_context_locked()
            self._request_replan_locked('HRI_PRIORITY_REGION_UPDATE', {
                'operator_action': 'SET_PRIORITY_REGION',
                'region_id': region_id,
            }, refresh_local_perception=False)
            self._publish_feedback_locked()
        rospy.loginfo('HRI priority region %s accepted (priority=%.2f, mode=%s).', region_id, priority, mode)

    def _remove_priority_region_cb(self, msg: String) -> None:
        payload = safe_json_loads(msg.data, None)
        region_id = str(payload.get('region_id', '')).strip() if isinstance(payload, dict) else str(msg.data).strip()
        if not region_id:
            rospy.logwarn('Ignored remove region request without region_id.')
            return
        with self.lock:
            region = self.priority_regions.pop(region_id, None)
            if region is None:
                rospy.logwarn('Priority region %s does not exist.', region_id)
                return
            self.context_version += 1
            self._record_action_locked('REMOVE_PRIORITY_REGION', {'region_id': region_id})
            self._publish_context_locked()
            self._request_replan_locked('HRI_PRIORITY_REGION_UPDATE', {
                'operator_action': 'REMOVE_PRIORITY_REGION',
                'region_id': region_id,
            }, refresh_local_perception=False)
            self._publish_feedback_locked()

    def _clear_priority_regions_cb(self, _msg: String) -> None:
        with self.lock:
            if not self.priority_regions:
                return
            removed = sorted(self.priority_regions.keys())
            self.priority_regions.clear()
            self.context_version += 1
            self._record_action_locked('CLEAR_PRIORITY_REGIONS', {'removed_region_ids': removed})
            self._publish_context_locked()
            self._request_replan_locked('HRI_PRIORITY_REGION_UPDATE', {
                'operator_action': 'CLEAR_PRIORITY_REGIONS',
                'removed_region_ids': removed,
            }, refresh_local_perception=False)
            self._publish_feedback_locked()

    def _target_instruction_cb(self, msg: String) -> None:
        payload = safe_json_loads(msg.data, None)
        if not isinstance(payload, dict):
            rospy.logwarn('Ignored invalid /hri/set_target_instruction payload.')
            return
        mode = str(payload.get('mode', 'replace')).lower()
        if mode not in ('replace', 'append_hint'):
            rospy.logwarn('Rejected target instruction: mode must be replace or append_hint.')
            return
        with self.lock:
            previous = copy.deepcopy(self.current_query)
            if mode == 'replace':
                query_text = str(payload.get('query_text', '')).strip()
                if not query_text:
                    rospy.logwarn('Rejected target replacement without query_text.')
                    return
                update = {
                    'query_text': query_text,
                    'hri_base_query_text': query_text,
                    'human_hints': [str(value).strip() for value in payload.get('human_hints', [])
                                    if str(value).strip()] if isinstance(payload.get('human_hints', []), list) else [],
                    'source': 'human_operator',
                    'last_human_instruction_mode': 'replace',
                    'last_human_instruction_wall_time': round(self._now(), 3),
                }
            else:
                hint = str(payload.get('hint_text', payload.get('query_text', ''))).strip()
                if not hint:
                    rospy.logwarn('Rejected append_hint request without hint_text.')
                    return
                base = str(previous.get('hri_base_query_text', previous.get('query_text', ''))).strip()
                hints = previous.get('human_hints', [])
                if not isinstance(hints, list):
                    hints = []
                normalized_hints = [str(value).strip() for value in hints if str(value).strip()]
                if hint not in normalized_hints:
                    normalized_hints.append(hint)
                guidance = '\n'.join('- %s' % value for value in normalized_hints)
                update = {
                    'query_text': '%s\n\nHuman operator guidance:\n%s' % (base, guidance),
                    'hri_base_query_text': base,
                    'human_hints': normalized_hints,
                    'source': 'human_operator',
                    'last_human_instruction_mode': 'append_hint',
                    'last_human_instruction_wall_time': round(self._now(), 3),
                }
            attributes = payload.get('target_attributes')
            # A replacement may intentionally clear the attribute dictionary;
            # an append-only clue must not erase the active color/shape filter
            # merely because the operator left optional attribute boxes blank.
            if isinstance(attributes, dict) and (mode == 'replace' or attributes):
                update['target_attributes'] = attributes
            query_type = str(payload.get('query_type', '')).strip()
            if query_type:
                update['query_type'] = query_type
            self._record_action_locked('SET_TARGET_INSTRUCTION', {
                'mode': mode,
                'query_update': update,
            })
            # target_query_manager increments query_version and activates the normal
            # TARGET_QUERY_CHANGE trigger, which refreshes Local VLM observations.
            self.set_query_pub.publish(compact_json(update))
            self._publish_feedback_locked()
        rospy.loginfo('HRI submitted target instruction mode=%s.', mode)

    def _force_replan_cb(self, msg: String) -> None:
        payload = safe_json_loads(msg.data, {})
        if not isinstance(payload, dict):
            payload = {}
        with self.lock:
            refresh = bool(payload.get('refresh_local_perception', True))
            self._record_action_locked('FORCE_REPLAN', payload)
            self._request_replan_locked('HRI_FORCE_REPLAN', {
                'operator_note': str(payload.get('operator_note', '')),
                'operator_action': 'FORCE_REPLAN',
            }, refresh_local_perception=refresh)
            self._publish_feedback_locked()

    def _query_cb(self, msg: String) -> None:
        query = safe_json_loads(msg.data, None)
        if not isinstance(query, dict):
            return
        with self.lock:
            self.current_query = query
            self.context_version += 1
            self._record_feedback_locked('target_query', {
                'query_version': query.get('query_version'),
                'query_id': query.get('query_id'),
                'query_text': query.get('query_text', ''),
            })
            self._publish_context_locked()
            self._publish_feedback_locked()

    def _central_plan_cb(self, msg: String) -> None:
        envelope = safe_json_loads(msg.data, None)
        if not isinstance(envelope, dict):
            return
        with self.lock:
            self.latest_central_plan = envelope
            catalog = envelope.get('candidate_catalog', [])
            if isinstance(catalog, list):
                self.candidate_by_id = {
                    str(item.get('id')): copy.deepcopy(item)
                    for item in catalog if isinstance(item, dict) and item.get('id')
                }
            self._record_feedback_locked('central_plan', {
                'epoch_id': envelope.get('epoch_id'),
                'status': envelope.get('status'),
                'event_reason': envelope.get('event_reason'),
            })
            self._publish_feedback_locked()

    def _validated_plan_cb(self, msg: String) -> None:
        plan = safe_json_loads(msg.data, None)
        if not isinstance(plan, dict):
            return
        with self.lock:
            self.latest_validated_plan = plan
            self._record_feedback_locked('validated_plan', {
                'epoch_id': plan.get('epoch_id'),
                'status': plan.get('status'),
                'accepted_count': len(plan.get('accepted', [])) if isinstance(plan.get('accepted'), list) else 0,
                'rejected_count': len(plan.get('rejected', [])) if isinstance(plan.get('rejected'), list) else 0,
            })
            self._publish_feedback_locked()

    def _goal_dispatch_cb(self, msg: String) -> None:
        dispatch = safe_json_loads(msg.data, None)
        if not isinstance(dispatch, dict):
            return
        robot_id = str(dispatch.get('robot_id', ''))
        if not robot_id:
            return
        with self.lock:
            candidate = self.candidate_by_id.get(str(dispatch.get('candidate_id', '')), {})
            item = copy.deepcopy(dispatch)
            item['candidate_metadata'] = {
                'human_priority_score': candidate.get('human_priority_score', 0.0),
                'human_priority_regions': candidate.get('human_priority_regions', []),
                'information_gain': candidate.get('information_gain', 0.0),
                'risk': candidate.get('risk', 0.0),
                'path_length_m': candidate.get('path_length_m'),
            }
            item['explanation'] = self._assignment_explanation(candidate, dispatch)
            self.current_assignments[robot_id] = item
            self._record_feedback_locked('goal_dispatch', {
                'robot_id': robot_id,
                'candidate_id': dispatch.get('candidate_id'),
                'task_type': dispatch.get('task_type'),
            })
            self._publish_feedback_locked()

    def _route_result_cb(self, msg: String) -> None:
        result = safe_json_loads(msg.data, None)
        if not isinstance(result, dict):
            return
        robot_id = str(result.get('robot_id', ''))
        if not robot_id:
            return
        with self.lock:
            self.route_status[robot_id] = result
            self._record_feedback_locked('route_result', result)
            self._publish_feedback_locked()

    def _sync_status_cb(self, msg: String) -> None:
        with self.lock:
            self.sync_status = str(msg.data)
            self._publish_feedback_locked()

    def _trigger_status_cb(self, msg: String) -> None:
        with self.lock:
            self.trigger_status = str(msg.data)
            self._publish_feedback_locked()

    def _semantic_overlay_cb(self, msg: String) -> None:
        overlay = safe_json_loads(msg.data, None)
        if not isinstance(overlay, dict):
            return
        with self.lock:
            # The overlay is already bounded to recent objects by its publisher.
            self.semantic_overlay = {
                'version': overlay.get('version', 0),
                'semantic_covered_cell_count': overlay.get('semantic_covered_cell_count', 0),
                'objects': overlay.get('objects', []) if isinstance(overlay.get('objects', []), list) else [],
                'latest_reports': overlay.get('latest_reports', []) if isinstance(overlay.get('latest_reports', []), list) else [],
            }
            self._publish_feedback_locked()

    def _local_report_cb(self, msg: String) -> None:
        report = safe_json_loads(msg.data, None)
        if not isinstance(report, dict):
            return
        robot_id = str(report.get('robot_id', ''))
        if not robot_id:
            return
        with self.lock:
            self.latest_local_reports[robot_id] = {
                'robot_id': robot_id,
                'epoch_id': report.get('epoch_id'),
                'status': report.get('status'),
                'query_version': report.get('query_version'),
                'scene_summary': report.get('scene_summary', ''),
                'target_evidence': report.get('target_evidence', {}),
                'requested_follow_up': report.get('requested_follow_up', 'none'),
                'trigger_reason': report.get('trigger_reason', ''),
            }
            self._publish_feedback_locked()

    def _expiry_timer_cb(self, _event: rospy.timer.TimerEvent) -> None:
        with self.lock:
            now = self._now()
            expired = [rid for rid, region in self.priority_regions.items()
                       if float(region.get('expires_wall_time', 0.0) or 0.0) > 0.0
                       and float(region.get('expires_wall_time')) <= now]
            if not expired:
                return
            for region_id in expired:
                self.priority_regions.pop(region_id, None)
            self.context_version += 1
            self._record_action_locked('EXPIRE_PRIORITY_REGION', {'region_ids': expired})
            self._publish_context_locked()
            self._request_replan_locked('HRI_PRIORITY_REGION_UPDATE', {
                'operator_action': 'EXPIRE_PRIORITY_REGION',
                'region_ids': expired,
            }, refresh_local_perception=False)
            self._publish_feedback_locked()
            rospy.loginfo('Expired HRI priority regions: %s.', ','.join(expired))


def main() -> None:
    rospy.init_node('human_interaction_manager')
    HumanInteractionManager()
    rospy.spin()


if __name__ == '__main__':
    main()
