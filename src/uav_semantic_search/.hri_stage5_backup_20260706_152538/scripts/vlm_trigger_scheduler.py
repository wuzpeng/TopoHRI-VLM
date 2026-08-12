#!/usr/bin/env python3
"""Lightweight trigger scheduler for local VLM perception.

No large VLM is invoked here.  The node continuously receives maps, poses and
images, and emits a single structured trigger when geometry, visual novelty,
semantic-coverage gain, query changes, mission completion or safety events
make a synchronous semantic decision epoch worthwhile.
"""
from __future__ import annotations

import copy
import math
import os
import sys
import threading
import uuid
from typing import Any, Dict, Optional

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, String

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)

from vlm_common import (compact_json, grid_from_msg, histogram_novelty,
                        normalized_hsv_histogram, now_wall, pose_to_dict,
                        sampled_depth_coverage_cells, safe_json_loads, yaw_from_pose)


class TriggerScheduler:
    def __init__(self):
        # 必须在创建任何 Publisher、Subscriber、Timer 之前初始化。
        # ROS 的 latched topic 可能在 Subscriber 创建后立即触发回调。
        self.lock = threading.RLock()

        root = rospy.get_param('/vlm_semantic_search')
        self.root = root
        self.cfg = root['scheduler']
        self.local_cfg = root['local_perception']
        # HSV 颜色候选检测及冻结快照配置。
        self.color_cfg = dict(
            self.cfg.get('color_candidate_trigger', {})
        )
        self.bridge = CvBridge()
        self.robots = list(rospy.get_param('/vehicles', [])) + list(rospy.get_param('/ground_robots', []))
        self.robot_by_name = {item['name']: item for item in self.robots if item.get('name')}
        self.states: Dict[str, Dict[str, Any]] = {}
        self.maps: Dict[str, Optional[OccupancyGrid]] = {'uav': None, 'ugv': None}
        self.epoch_active = False
        self.query = dict(root.get('target_query', {}))
        self.query_version = 0
        # /vlm/target_query is latched.  The first callback only establishes
        # the initial query and must never create a pre-readiness epoch.
        self.initial_query_received = False
        self.pending_query_change = False
        self.initial_fired = False
        self.last_trigger_wall = 0.0

        self.trigger_pub = rospy.Publisher(
            '/vlm/trigger_event',
            String,
            queue_size=5,
        )

        self.status_pub = rospy.Publisher(
            '/vlm/trigger_status',
            String,
            queue_size=5,
            latch=True,
        )

        # 新增：发布稳定颜色候选信息，便于调试。
        self.color_candidate_pub = rospy.Publisher(
            '/vlm/color_candidate',
            String,
            queue_size=10,
        )

        # 每台机器人各自发布 HSV 检测调试图像。
        self.color_debug_pubs = {}

        # 颜色候选产生瞬间的冻结 RGB-D 快照。
        self.color_snapshot_rgb_pubs = {}
        self.color_snapshot_depth_pubs = {}
        self.color_snapshot_info_pubs = {}
        self.color_snapshot_pose_pubs = {}
        self.color_snapshot_meta_pubs = {}

        # 颜色候选触发的停止/悬停控制命令。
        self.color_hold_pubs = {}

        # 每台机器人各自发布一个带颜色候选框的调试图像。
        self.color_debug_pubs = {}

        for robot in self.robots:
            name = robot['name']
            snapshot_ns = '/%s/vlm/color_snapshot' % name

            self.color_snapshot_rgb_pubs[name] = rospy.Publisher(
                snapshot_ns + '/rgb',
                Image,
                queue_size=2,
            )

            self.color_snapshot_depth_pubs[name] = rospy.Publisher(
                snapshot_ns + '/depth',
                Image,
                queue_size=2,
            )

            self.color_snapshot_info_pubs[name] = rospy.Publisher(
                snapshot_ns + '/camera_info',
                CameraInfo,
                queue_size=2,
            )

            self.color_snapshot_pose_pubs[name] = rospy.Publisher(
                snapshot_ns + '/pose_map',
                PoseStamped,
                queue_size=2,
            )

            self.color_snapshot_meta_pubs[name] = rospy.Publisher(
                snapshot_ns + '/meta',
                String,
                queue_size=2,
            )

            self.color_hold_pubs[name] = rospy.Publisher(
                '/%s/vlm/observation_hold_command' % name,
                String,
                queue_size=5,
            )

            # Scheduler 同时订阅这个 topic。
            # 它发布 active=True；Coordinator 在颜色事件完成后发布 active=False。
            rospy.Subscriber(
                '/%s/vlm/observation_hold_command' % name,
                String,
                lambda msg, n=name: self._color_hold_cmd_cb(n, msg),
                queue_size=10,
            )

            if bool(self.color_cfg.get('publish_debug_image', True)):
                self.color_debug_pubs[name] = rospy.Publisher(
                    '/%s/vlm/color_candidate_debug_image' % name,
                    Image,
                    queue_size=1,
                )

            self.states[name] = {
                'pose': None, 'image': None, 'depth': None, 'camera_info': None,
                'last_sem_pose': None, 'last_sem_yaw': None, 'last_sem_hist': None,
                'last_sem_coverage': set(), 'last_free_count': 0,
                # `goal_reached` is a one-shot pending event. `reached_level` stores
                # the raw Bool level from the executor so repeated latched True
                # messages cannot retrigger a new VLM epoch.
                'goal_reached': False, 'reached_level': False,
                # Completion is armed exclusively by the validated goal-dispatch
                # event.  Raw /mission/reached False messages from the executor do
                # NOT rearm it; otherwise a repeated same-goal dispatch creates an
                # infinite reached -> plan -> reached loop.
                'completion_armed': False, 'completion_pending_dispatch': None,
                'last_dispatch_signature': None, 'last_completed_dispatch': None,
                'last_dispatch_task_type': '', 'local_vlm_ready': False,
                'blocked': False,
                'topology_signature': None,
                'cooldown_until': 0.0,
                # 当前 RGB 图中实时检测到的颜色候选。
                'color_candidate': None,
                # 已冻结、等待或正在等待 Local VLM 复核的颜色事件。
                'pending_color_event': None,
                'color_hits': 0,
                'color_clears': 0,
                # 颜色事件已生成快照，但尚未发到 Coordinator。
                'color_trigger_pending': False,
                # 颜色事件已发到 Coordinator；在当前颜色候选消失前不重复触发。
                'color_trigger_consumed': False,
                # 防止同一稳定颜色区域重复生成 RGB-D 快照。
                'color_event_frozen': False,
                # 当前机器人是否因颜色候选而进入观测保持状态。
                'color_hold_active': False,
                'color_hold_id': None,
                'last_color_detection_wall': 0.0,

                'blocked_event_pending': False,
                'blocked_event_consumed': False,
            }
            rospy.Subscriber(robot['global_pose_topic'], PoseStamped,
                             lambda msg, n=name: self._pose_cb(n, msg), queue_size=20)
            rospy.Subscriber(robot['rgb_topic'], Image,
                             lambda msg, n=name: self._rgb_cb(n, msg), queue_size=1, buff_size=2**24)
            rospy.Subscriber(robot['depth_topic'], Image,
                             lambda msg, n=name: self._depth_cb(n, msg), queue_size=1, buff_size=2**24)
            rospy.Subscriber(robot['camera_info_topic'], CameraInfo,
                             lambda msg, n=name: self._info_cb(n, msg), queue_size=2, buff_size=2**20)
            # Stage-4 UAV configs historically did not declare this key explicitly.
            # Keep the Stage-5 scheduler backward-compatible while using the standard
            # per-robot mission completion topic.
            reached_topic = str(robot.get('mission_reached_topic', '/%s/mission/reached' % name))
            rospy.Subscriber(reached_topic, Bool,
                             lambda msg, n=name: self._reached_cb(n, msg), queue_size=5)
            if robot.get('blocked_topic'):
                rospy.Subscriber(robot['blocked_topic'], Bool,
                                 lambda msg, n=name: self._blocked_cb(n, msg), queue_size=5)
            # Startup barrier: do not emit INITIAL_CONTEXT_READY until every
            # local VLM node has created its request subscriber.
            rospy.Subscriber('/%s/vlm/ready' % name, Bool,
                             lambda msg, n=name: self._local_ready_cb(n, msg), queue_size=2)

        rospy.Subscriber('/global_map_2d', OccupancyGrid, lambda msg: self._map_cb('uav', msg), queue_size=2)
        rospy.Subscriber('/ugv0/ground_map_2d', OccupancyGrid, lambda msg: self._map_cb('ugv', msg), queue_size=2)
        rospy.Subscriber('/vlm/epoch_active', Bool, self._epoch_cb, queue_size=5)
        rospy.Subscriber('/vlm/target_query', String, self._query_cb, queue_size=5)
        rospy.Subscriber('/vlm/local_semantic_observation', String, self._semantic_cb, queue_size=20)
        # Goal dispatch metadata is the authoritative re-arm signal for
        # GOAL_REACHED. It carries candidate/task semantics that raw executor
        # Bool topics do not provide.
        rospy.Subscriber('/vlm/goal_dispatch', String, self._goal_dispatch_cb, queue_size=20)
        for robot in self.robots:
            if robot.get('type') == 'uav':
                rospy.Subscriber('/%s/mission/takeoff_ready' % robot['name'], Bool,
                                 lambda msg, n=robot['name']: self._takeoff_cb(n, msg), queue_size=2)
                self.states[robot['name']]['takeoff_ready'] = False

        rate = max(0.2, float(self.cfg.get('tick_hz', 2.0)))
        rospy.Timer(rospy.Duration(1.0 / rate), self._tick)
        self.status_pub.publish('WAITING_FOR_SENSOR_CONTEXT')

    def _pose_cb(self, name, msg):
        with self.lock:
            self.states[name]['pose'] = msg

    def _rgb_cb(self, name, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )
        except Exception:
            return

        image = np.array(image, copy=True)

        with self.lock:
            self.states[name]['image'] = image

        # 无论当前是否已有 VLM epoch，都持续检测颜色候选。
        # 这里不直接调用 VLM，只进行廉价 HSV 检测。
        if bool(self.color_cfg.get('enabled', False)):
            self._update_color_candidate(name, image)

    def _detect_color_candidate(self, bgr):
        """Detect the largest configured color region.

        This is only a trigger proposal. It does not determine whether the object
        is the requested target. Local VLM performs the semantic confirmation.
        """
        profiles = self.color_cfg.get('profiles', {})

        if not isinstance(profiles, dict) or not profiles:
            return None

        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        image_area = float(max(1, bgr.shape[0] * bgr.shape[1]))

        best = None

        for color_name, profile in profiles.items():
            if not isinstance(profile, dict):
                continue

            if not bool(profile.get('enabled', True)):
                continue

            ranges = profile.get('hsv_ranges', [])
            if not isinstance(ranges, list) or not ranges:
                continue

            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)

            for hsv_range in ranges:
                if not isinstance(hsv_range, dict):
                    continue

                lower = np.asarray(
                    hsv_range.get('lower', [0, 0, 0]),
                    dtype=np.uint8,
                )

                upper = np.asarray(
                    hsv_range.get('upper', [179, 255, 255]),
                    dtype=np.uint8,
                )

                mask = cv2.bitwise_or(
                    mask,
                    cv2.inRange(hsv, lower, upper),
                )

            kernel_size = max(
                1,
                int(profile.get('morphology_kernel_px', 5)),
            )

            if kernel_size % 2 == 0:
                kernel_size += 1

            kernel = np.ones(
                (kernel_size, kernel_size),
                dtype=np.uint8,
            )

            mask = cv2.morphologyEx(
                mask,
                cv2.MORPH_OPEN,
                kernel,
            )

            mask = cv2.morphologyEx(
                mask,
                cv2.MORPH_CLOSE,
                kernel,
            )

            contour_result = cv2.findContours(
                mask,
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE,
            )

            if len(contour_result) == 2:
                contours, _ = contour_result
            else:
                _, contours, _ = contour_result

            if not contours:
                continue

            contour = max(contours, key=cv2.contourArea)
            area = float(cv2.contourArea(contour))

            if area < float(profile.get('min_area_px', 180)):
                continue

            x, y, w, h = cv2.boundingRect(contour)

            moments = cv2.moments(contour)

            if abs(moments['m00']) < 1e-6:
                u = x + w // 2
                v = y + h // 2
            else:
                u = int(moments['m10'] / moments['m00'])
                v = int(moments['m01'] / moments['m00'])

            candidate = {
                'color': str(color_name),
                'bbox': [int(x), int(y), int(x + w), int(y + h)],
                'center_px': [int(u), int(v)],
                'area_px': round(area, 1),
                'confidence': round(
                    min(0.99, area / max(1.0, 0.02 * image_area)),
                    3,
                ),
            }

            if best is None or candidate['area_px'] > best['area_px']:
                best = candidate

        return best


    def _publish_color_debug(self, name, bgr, candidate):
        """Publish a debug image with the HSV candidate box."""

        pub = self.color_debug_pubs.get(name)

        if pub is None:
            return

        debug = bgr.copy()

        if candidate is not None:
            x0, y0, x1, y1 = candidate['bbox']

            cv2.rectangle(
                debug,
                (x0, y0),
                (x1, y1),
                (255, 0, 255),
                2,
            )

            text = '%s HSV candidate c=%.2f' % (
                candidate['color'],
                candidate['confidence'],
            )

            cv2.putText(
                debug,
                text,
                (x0, max(18, y0 - 6)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.50,
                (255, 0, 255),
                2,
            )

        try:
            pub.publish(
                self.bridge.cv2_to_imgmsg(
                    debug,
                    encoding='bgr8',
                )
            )
        except Exception:
            pass

    def _update_color_candidate(self, name, bgr):
        """Detect, freeze and preserve a stable color candidate.

        Key differences from the previous version:
        1. A stable color candidate generates one frozen RGB-D snapshot.
        2. The corresponding robot is immediately held.
        3. Candidate disappearance does not erase a queued/active color event.
        """

        if not bool(self.color_cfg.get('enabled', False)):
            return

        detect_rate_hz = max(
            0.1,
            float(self.color_cfg.get('detect_rate_hz', 5.0)),
        )

        wall = now_wall()

        with self.lock:
            state = self.states[name]

            if (
                wall - float(
                    state.get('last_color_detection_wall', 0.0)
                )
                < 1.0 / detect_rate_hz
            ):
                return

            state['last_color_detection_wall'] = wall

        candidate = self._detect_color_candidate(bgr)

        self._publish_color_debug(
            name,
            bgr,
            candidate,
        )

        confirm_frames = max(
            1,
            int(self.color_cfg.get('confirm_frames', 2)),
        )

        clear_frames = max(
            1,
            int(self.color_cfg.get('clear_frames', 3)),
        )

        candidate_to_freeze = None

        with self.lock:
            state = self.states[name]

            if candidate is not None:
                previous = state.get('color_candidate') or {}

                previous_color = str(
                    previous.get('color', '')
                )

                current_color = str(
                    candidate.get('color', '')
                )

                # Color category changes create a new candidate sequence.
                if previous_color != current_color:
                    state['color_hits'] = 1
                else:
                    state['color_hits'] = int(
                        state.get('color_hits', 0)
                    ) + 1

                state['color_clears'] = 0
                state['color_candidate'] = dict(candidate)

                # Create exactly one snapshot for one stable candidate event.
                if (
                    state['color_hits'] >= confirm_frames
                    and not bool(
                        state.get('color_event_frozen', False)
                    )
                ):
                    state['color_event_frozen'] = True
                    candidate_to_freeze = dict(candidate)

            else:
                state['color_clears'] = int(
                    state.get('color_clears', 0)
                ) + 1

                if state['color_clears'] >= clear_frames:
                    state['color_candidate'] = None
                    state['color_hits'] = 0

                    # Crucial: do not erase a frozen event while it is queued,
                    # executing, or holding the robot for VLM confirmation.
                    if not bool(
                        state.get('color_hold_active', False)
                    ):
                        self._reset_color_event_locked(state)

        if candidate_to_freeze is None:
            return

        frozen_candidate = self._freeze_and_publish_color_snapshot(
            name,
            candidate_to_freeze,
        )

        if frozen_candidate is None:
            with self.lock:
                self.states[name]['color_event_frozen'] = False
            return

        snapshot_id = str(
            frozen_candidate['snapshot_id']
        )

        with self.lock:
            state = self.states[name]

            state['pending_color_event'] = dict(
                frozen_candidate
            )

            state['color_trigger_pending'] = True
            state['color_trigger_consumed'] = False
            state['color_hold_active'] = bool(
                self.color_cfg.get(
                    'hold_on_stable_candidate',
                    True,
                )
            )
            state['color_hold_id'] = snapshot_id

        if bool(
            self.color_cfg.get(
                'hold_on_stable_candidate',
                True,
            )
        ):
            self._publish_color_hold(
                name,
                True,
                snapshot_id,
                'STABLE_COLOR_CANDIDATE',
            )

        self.color_candidate_pub.publish(
            compact_json({
                'robot_id': name,
                'status': 'FROZEN_COLOR_CANDIDATE',
                'candidate': frozen_candidate,
            })
        )

        rospy.loginfo(
            '%s froze stable %s color candidate: snapshot=%s bbox=%s area=%.1f.',
            name,
            frozen_candidate.get('color'),
            snapshot_id,
            frozen_candidate.get('bbox'),
            float(frozen_candidate.get('area_px', 0.0)),
        )

    def _reset_color_event_locked(self, state: Dict[str, Any]) -> None:
        """Re-arm color detection after the old candidate has disappeared."""

        state['pending_color_event'] = None
        state['color_trigger_pending'] = False
        state['color_trigger_consumed'] = False
        state['color_event_frozen'] = False
        state['color_hold_id'] = None


    def _color_hold_cmd_cb(self, name: str, msg: String) -> None:
        """Track active/released observation holds.

        Scheduler publishes active=True immediately after detecting a stable color
        candidate. Coordinator publishes active=False after the corresponding
        color epoch has produced a plan or fallback plan.
        """

        command = safe_json_loads(msg.data, None)

        if not isinstance(command, dict):
            return

        hold_id = str(command.get('hold_id', ''))
        active = bool(command.get('active', False))

        with self.lock:
            state = self.states.get(name)

            if state is None:
                return

            current_id = str(state.get('color_hold_id') or '')

            if active:
                state['color_hold_active'] = True
                state['color_hold_id'] = hold_id
                return

            # Old release commands must never release a newer color hold.
            if current_id and hold_id and current_id != hold_id:
                return

            state['color_hold_active'] = False

            clear_frames = max(
                1,
                int(self.color_cfg.get('clear_frames', 3)),
            )

            # If the old color target has already left the image, re-arm future
            # color triggers after receiving the matching release command.
            if int(state.get('color_clears', 0)) >= clear_frames:
                self._reset_color_event_locked(state)


    def _publish_color_hold(
            self,
            name: str,
            active: bool,
            hold_id: str,
            reason: str
    ) -> None:
        """Command the corresponding executor to stop/hover or resume."""

        pub = self.color_hold_pubs.get(name)

        if pub is None:
            return

        command = {
            'robot_id': name,
            'active': bool(active),
            'hold_id': str(hold_id),
            'reason': str(reason),
            'issued_wall_time': round(now_wall(), 3),
            'max_hold_sec': float(
                self.color_cfg.get('hold_max_sec', 300.0)
            ),
        }

        pub.publish(compact_json(command))

        rospy.loginfo(
            '%s color observation hold active=%s id=%s reason=%s.',
            name,
            active,
            hold_id,
            reason,
        )


    def _freeze_and_publish_color_snapshot(
            self,
            name: str,
            candidate: Dict[str, Any]
    ) -> Optional[Dict[str, Any]]:
        """Freeze current RGB-D, CameraInfo and map pose for delayed VLM review.

        The Local VLM must later analyze this frozen observation, not a new image
        captured after the robot has already passed the object.
        """

        with self.lock:
            state = self.states[name]

            image = (
                state['image'].copy()
                if state.get('image') is not None
                else None
            )

            depth = (
                state['depth'].copy()
                if state.get('depth') is not None
                else None
            )

            camera_info = (
                copy.deepcopy(state['camera_info'])
                if state.get('camera_info') is not None
                else None
            )

            pose = (
                copy.deepcopy(state['pose'])
                if state.get('pose') is not None
                else None
            )

        if (
            image is None
            or depth is None
            or camera_info is None
            or pose is None
        ):
            rospy.logwarn(
                '%s cannot freeze color snapshot: RGB-D/CameraInfo/Pose incomplete.',
                name,
            )
            return None

        snapshot_id = 'color_%s_%s' % (
            name,
            uuid.uuid4().hex[:10],
        )

        frozen_candidate = dict(candidate)
        frozen_candidate['snapshot_id'] = snapshot_id
        frozen_candidate['capture_wall_time'] = round(
            now_wall(),
            3,
        )

        stamp = rospy.Time.now()

        try:
            rgb_msg = self.bridge.cv2_to_imgmsg(
                image,
                encoding='bgr8',
            )

            depth_msg = self.bridge.cv2_to_imgmsg(
                depth,
                encoding='passthrough',
            )

            info_msg = copy.deepcopy(camera_info)
            pose_msg = copy.deepcopy(pose)

            # Snapshot ID is written into header.frame_id so Local VLM can match
            # RGB, Depth, CameraInfo and Pose without defining a custom ROS message.
            rgb_msg.header.stamp = stamp
            rgb_msg.header.frame_id = snapshot_id

            depth_msg.header.stamp = stamp
            depth_msg.header.frame_id = snapshot_id

            info_msg.header.stamp = stamp
            info_msg.header.frame_id = snapshot_id

            pose_msg.header.stamp = stamp
            pose_msg.header.frame_id = snapshot_id

            # Publish data first and metadata last.
            self.color_snapshot_rgb_pubs[name].publish(rgb_msg)
            self.color_snapshot_depth_pubs[name].publish(depth_msg)
            self.color_snapshot_info_pubs[name].publish(info_msg)
            self.color_snapshot_pose_pubs[name].publish(pose_msg)

            self.color_snapshot_meta_pubs[name].publish(
                compact_json({
                    'snapshot_id': snapshot_id,
                    'robot_id': name,
                    'capture_wall_time': frozen_candidate[
                        'capture_wall_time'
                    ],
                    'color_candidate': frozen_candidate,
                    'original_pose_frame': (
                        pose.header.frame_id or 'map'
                    ),
                })
            )

        except Exception as exc:
            rospy.logwarn(
                '%s failed to publish color snapshot: %r',
                name,
                exc,
            )
            return None

        return frozen_candidate


    def _emit_pending_color_event(self, name: str) -> bool:
        """Emit a saved color event even while another VLM epoch is active."""

        with self.lock:
            state = self.states[name]

            event = state.get('pending_color_event')

            if (
                not isinstance(event, dict)
                or not bool(state.get('color_trigger_pending', False))
                or bool(state.get('color_trigger_consumed', False))
            ):
                return False

            color_name = str(
                event.get('color', 'unknown')
            ).upper()

            details = {
                'color_candidate': dict(event),
                'snapshot_id': event.get('snapshot_id'),
                'hold_id': event.get('snapshot_id'),
                'snapshot_policy': 'frozen_color_candidate_snapshot',
            }

        reason = 'COLOR_CANDIDATE_%s' % color_name

        # allow_while_epoch_active=True is the key change:
        # Coordinator will queue this event instead of losing it.
        if self._emit(
            reason,
            name,
            details,
            allow_while_epoch_active=True,
        ):
            with self.lock:
                state = self.states[name]

                if (
                    state.get('pending_color_event', {})
                    .get('snapshot_id')
                    == event.get('snapshot_id')
                ):
                    state['color_trigger_pending'] = False
                    state['color_trigger_consumed'] = True

            return True

        return False

    def _depth_cb(self, name, msg):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception:
            return
        with self.lock:
            self.states[name]['depth'] = np.array(depth, copy=True)

    def _info_cb(self, name, msg):
        with self.lock:
            self.states[name]['camera_info'] = msg

    def _reached_cb(self, name, msg):
        """Consume a raw executor completion only for an armed dispatched goal.

        Executors deliberately publish False when they accept any mission goal,
        then True when they are inside tolerance.  Therefore raw False->True
        edges alone are insufficient: a repeated zero-displacement goal would
        re-trigger VLM epochs forever.  The validator dispatch event arms one
        completion only for a *new*, motion-requiring task.
        """
        current = bool(msg.data)
        with self.lock:
            state = self.states[name]
            previous = bool(state.get('reached_level', False))
            state['reached_level'] = current
            if (current and not previous and bool(state.get('completion_armed', False))
                    and state.get('completion_pending_dispatch')
                    and state.get('last_completed_dispatch') != state.get('completion_pending_dispatch')):
                state['goal_reached'] = True
                state['completion_armed'] = False
                state['last_completed_dispatch'] = state.get('completion_pending_dispatch')

    def _local_ready_cb(self, name, msg):
        with self.lock:
            self.states[name]['local_vlm_ready'] = bool(msg.data)

    @staticmethod
    def _dispatch_signature(dispatch):
        goal = dispatch.get('goal') or {}
        return '%s|%s|%.3f|%.3f|%.3f|%.3f' % (
            dispatch.get('candidate_id', ''), dispatch.get('task_type', ''),
            float(goal.get('x', 0.0)), float(goal.get('y', 0.0)),
            float(goal.get('z', 0.0)), float(goal.get('yaw_rad', 0.0)))

    def _goal_dispatch_cb(self, msg):
        dispatch = safe_json_loads(msg.data, None)
        if not isinstance(dispatch, dict):
            return
        name = str(dispatch.get('robot_id', ''))
        if name not in self.states:
            return
        task_type = str(dispatch.get('task_type', '')).upper()
        signature = self._dispatch_signature(dispatch)
        trigger_tasks = {str(x).upper() for x in self.cfg.get(
            'goal_completion_trigger_task_types',
            ['EXPLORE', 'INSPECT', 'GROUND_VERIFY', 'AERIAL_INSPECT'])}
        with self.lock:
            state = self.states[name]
            # Republished identical plans must not re-arm the completion event.
            if signature == state.get('last_dispatch_signature'):
                return
            state['last_dispatch_signature'] = signature
            state['last_dispatch_task_type'] = task_type
            state['goal_reached'] = False
            state['reached_level'] = False
            state['completion_pending_dispatch'] = signature
            state['completion_armed'] = task_type in trigger_tasks
            # A zero-displacement scan/hold is already semantically handled in
            # its dispatch epoch, so it must not create a new GOAL_REACHED epoch.
            if not state['completion_armed']:
                rospy.loginfo('VLM completion gate disarmed for %s task=%s candidate=%s.',
                              name, task_type, dispatch.get('candidate_id', ''))

    def _blocked_cb(self, name, msg):
        current = bool(msg.data)

        with self.lock:
            state = self.states[name]
            previous = bool(state.get('blocked', False))

            state['blocked'] = current

            # 仅记录 false → true 的阻塞边沿。
            if current and not previous:
                state['blocked_event_pending'] = True

            # 清除后才允许下一次新的阻塞事件重新触发。
            if not current:
                state['blocked_event_pending'] = False
                state['blocked_event_consumed'] = False

    def _takeoff_cb(self, name, msg):
        with self.lock:
            self.states[name]['takeoff_ready'] = bool(msg.data)

    def _map_cb(self, key, msg):
        with self.lock:
            self.maps[key] = msg

    def _epoch_cb(self, msg):
        with self.lock:
            self.epoch_active = bool(msg.data)

    def _query_cb(self, msg):
        """Handle latched initial target query without producing a false epoch.

        target_query_manager publishes the active query with latch=True during
        startup.  Every new scheduler subscriber receives that message once.
        It initializes local state only; an epoch is emitted only for a later
        operator update after INITIAL_CONTEXT_READY has completed.
        """
        query = safe_json_loads(msg.data, None)
        if not isinstance(query, dict):
            return
        with self.lock:
            first_message = not self.initial_query_received
            self.initial_query_received = True
            self.query = query
            self.query_version = int(query.get('query_version', self.query_version + 1))
            initial_fired = bool(self.initial_fired)
            active = bool(self.epoch_active)
            if not first_message and not initial_fired:
                # The initial context epoch will use the newest query; no extra
                # target-change epoch is necessary before the system is ready.
                self.pending_query_change = False
                self.status_pub.publish('TARGET_QUERY_UPDATED_BEFORE_INITIAL_CONTEXT:v%d' % self.query_version)
                return
            if first_message:
                self.status_pub.publish('TARGET_QUERY_INITIALIZED:v%d' % self.query_version)
                return
            if active:
                # Preserve a real user update that arrives while another epoch
                # is running; _tick will emit it after the active epoch ends.
                self.pending_query_change = True
                self.status_pub.publish('TARGET_QUERY_CHANGE_QUEUED:v%d' % self.query_version)
                return
        if not self._emit('TARGET_QUERY_CHANGE', 'all', {'query_version': self.query_version}):
            with self.lock:
                self.pending_query_change = True

    def _semantic_cb(self, msg):
        report = safe_json_loads(msg.data, None)
        if not isinstance(report, dict):
            return
        name = report.get('robot_id')
        if name not in self.states:
            return
        with self.lock:
            state = self.states[name]
            state['last_sem_pose'] = state['pose']
            state['last_sem_yaw'] = yaw_from_pose(state['pose']) if state['pose'] is not None else None
            state['last_sem_hist'] = normalized_hsv_histogram(state['image'])
            state['last_sem_coverage'] = {tuple(cell) for cell in report.get('coverage_cells', []) if isinstance(cell, list) and len(cell) == 2}
            map_key = 'ugv' if self.robot_by_name[name].get('type') == 'ugv' else 'uav'
            grid = grid_from_msg(self.maps.get(map_key))
            state['last_free_count'] = int((grid.data == 0).sum()) if grid is not None else 0
            # A local semantic report must not clear a pending goal-completion
            # event; completion is consumed only after the scheduler emits it.
            state['blocked'] = False
            # When a cloud VLM request fails or falls back to mock output, keep
            # the geometry/visual baselines but suppress automatic retriggers for
            # this robot. Otherwise a slow endpoint creates a failure -> retrigger
            # loop as soon as the robot moves a few cells.
            report_status = str(report.get('status', 'OK')).upper()
            if report_status != 'OK' or report.get('backend_error'):
                cooldown = float(self.cfg.get('backend_failure_cooldown_sec', 60.0))
                state['cooldown_until'] = max(float(state.get('cooldown_until', 0.0)), now_wall() + cooldown)
                rospy.logwarn('VLM scheduler cooldown for %s: %.1fs after local report status=%s.',
                              name, cooldown, report_status)

    def _ready(self) -> bool:
        for robot in self.robots:
            st = self.states[robot['name']]
            if st['pose'] is None or st['image'] is None or st['depth'] is None or st['camera_info'] is None:
                return False
        if bool(self.cfg.get('require_all_local_vlm_ready', True)):
            for robot in self.robots:
                if not self.states[robot['name']].get('local_vlm_ready', False):
                    return False
        if self.maps['uav'] is None or self.maps['ugv'] is None:
            return False
        if bool(self.cfg.get('require_all_uav_takeoff_ready', True)):
            for robot in self.robots:
                if robot.get('type') == 'uav' and not self.states[robot['name']].get('takeoff_ready', False):
                    return False
        ug = grid_from_msg(self.maps['uav'])
        gg = grid_from_msg(self.maps['ugv'])
        return bool(ug and gg and int((ug.data >= 0).sum()) >= int(self.cfg.get('min_known_cells_uav', 20)) and
                    int((gg.data >= 0).sum()) >= int(self.cfg.get('min_known_cells_ugv', 20)))

    @staticmethod
    def _distance(a: PoseStamped, b: PoseStamped) -> float:
        return float(math.hypot(a.pose.position.x-b.pose.position.x, a.pose.position.y-b.pose.position.y))

    def _current_coverage(self, robot, state, grid):
        return set(tuple(c) for c in sampled_depth_coverage_cells(
            robot, state['pose'], state['camera_info'], state['depth'], grid,
            int(self.cfg.get('coverage_depth_stride_px', 32)),
            float(self.local_cfg.get('min_depth_m', 0.30)),
            float(self.local_cfg.get('max_depth_m', 8.0)),
            int(self.cfg.get('max_coverage_cells_per_observation', 280))))

    def _topology_signature(self, grid, pose: PoseStamped, radius_m: float) -> Optional[tuple]:
        if grid is None or pose is None:
            return None
        cell = grid.world_to_cell(pose.pose.position.x, pose.pose.position.y)
        if cell is None:
            return None
        radius = max(1, int(radius_m / grid.resolution))
        sectors = []
        for idx in range(16):
            theta = 2.0 * math.pi * idx / 16.0
            x = int(round(cell[0] + radius * math.cos(theta)))
            y = int(round(cell[1] + radius * math.sin(theta)))
            if 0 <= x < grid.width and 0 <= y < grid.height:
                sectors.append(1 if grid.data[y, x] == 0 else 0)
            else:
                sectors.append(0)
        branches = 0
        for i, value in enumerate(sectors):
            previous = sectors[i - 1]
            if value and not previous:
                branches += 1
        free_local = int((grid.data[max(0, cell[1]-radius):min(grid.height, cell[1]+radius+1),
                                    max(0, cell[0]-radius):min(grid.width, cell[0]+radius+1)] == 0).sum())
        return (branches, free_local // 8)

    def _emit(
            self,
            reason: str,
            robot_id: str,
            details: Optional[Dict[str, Any]] = None,
            allow_while_epoch_active: bool = False
    ) -> bool:
        with self.lock:
            if (
                self.epoch_active
                and not allow_while_epoch_active
            ):
                return False

            wall = now_wall()

            bypass_interval = (
                reason == 'TARGET_QUERY_CHANGE'
                or str(reason).startswith(
                    'COLOR_CANDIDATE_'
                )
            )

            if (
                not bypass_interval
                and wall - self.last_trigger_wall
                < float(
                    self.cfg.get(
                        'min_trigger_interval_sec',
                        2.0,
                    )
                )
            ):
                return False

            epoch = 'epoch_%s' % uuid.uuid4().hex[:10]

            event = {
                'epoch_id': epoch,
                'reason': reason,
                'robot_id': robot_id,
                'query': self.query,
                'query_version': self.query_version,
                'details': details or {},
                'trigger_wall_time': wall,
            }

            # 颜色事件不应该压制后续普通事件的触发间隔。
            if not str(reason).startswith(
                'COLOR_CANDIDATE_'
            ):
                self.last_trigger_wall = wall

        self.trigger_pub.publish(
            compact_json(event)
        )

        self.status_pub.publish(
            'TRIGGERED:%s:%s' % (
                reason,
                robot_id,
            )
        )

        rospy.loginfo(
            'VLM trigger: %s from %s',
            reason,
            robot_id,
        )

        return True

    def _tick(self, _event):
        with self.lock:
            ready = self._ready()
            epoch_active = bool(self.epoch_active)

        if not ready:
            self.status_pub.publish(
                'WAITING_FOR_READY_CONTEXT'
            )
            return

        # 初始 context epoch 只在没有活动 epoch 时启动。
        if not epoch_active and not self.initial_fired:
            self.initial_fired = True
            self._emit(
                'INITIAL_CONTEXT_READY',
                'all',
            )
            return

        # 颜色事件具有特殊权限：
        # 即使当前已有 epoch，也允许发布给 Coordinator 排队。
        if epoch_active:
            for robot in self.robots:
                if self._emit_pending_color_event(
                    robot['name']
                ):
                    return
            return

        # 以下逻辑只在当前没有活动 epoch 时执行。
        with self.lock:
            if self.pending_query_change:
                self.pending_query_change = False

                if self._emit(
                    'TARGET_QUERY_CHANGE',
                    'all',
                    {
                        'query_version': self.query_version,
                    },
                ):
                    return

                self.pending_query_change = True
                return

        for robot in self.robots:
            name = robot['name']

            with self.lock:
                st = self.states[name]

                map_key = (
                    'ugv'
                    if robot.get('type') == 'ugv'
                    else 'uav'
                )

                grid = grid_from_msg(
                    self.maps[map_key]
                )

                if grid is None or st['pose'] is None:
                    continue

                blocked = bool(st['blocked'])
                goal_reached = bool(
                    st['goal_reached']
                )

            # 安全阻塞仍然优先于颜色事件。
            if (
                bool(
                    self.cfg.get(
                        'trigger_on_robot_blocked',
                        True,
                    )
                )
                and blocked
            ):
                if self._emit(
                    'ROBOT_BLOCKED',
                    name,
                ):
                    return

            # 颜色事件优先于 GOAL_REACHED。
            if self._emit_pending_color_event(name):
                return

            if (
                bool(
                    self.cfg.get(
                        'trigger_on_goal_reached',
                        True,
                    )
                )
                and goal_reached
            ):
                with self.lock:
                    details = {
                        'dispatch_signature': st.get(
                            'last_completed_dispatch'
                        ),
                        'task_type': st.get(
                            'last_dispatch_task_type',
                            '',
                        ),
                    }

                if self._emit(
                    'GOAL_REACHED',
                    name,
                    details,
                ):
                    with self.lock:
                        self.states[name][
                            'goal_reached'
                        ] = False
                    return

            with self.lock:
                st = self.states[name]

                if st['last_sem_pose'] is None:
                    continue

                if now_wall() < float(
                    st.get(
                        'cooldown_until',
                        0.0,
                    )
                ):
                    continue

                pose = st['pose']
                last_sem_pose = st['last_sem_pose']
                last_sem_yaw = st['last_sem_yaw']
                image = st['image']
                last_sem_hist = st['last_sem_hist']
                previous_coverage = st[
                    'last_sem_coverage'
                ]
                previous_free_count = int(
                    st['last_free_count']
                )

            travel = self._distance(
                pose,
                last_sem_pose,
            )

            threshold = float(
                self.cfg.get(
                    (
                        'ugv_distance_trigger_m'
                        if robot.get('type') == 'ugv'
                        else 'uav_distance_trigger_m'
                    ),
                    0.5,
                )
            )

            if (
                bool(
                    self.cfg.get(
                        'trigger_on_distance_in_uncertain_space',
                        True,
                    )
                )
                and travel >= threshold
            ):
                if self._emit(
                    'DISTANCE_IN_UNCERTAIN_SPACE',
                    name,
                    {
                        'travel_m': round(
                            travel,
                            3,
                        ),
                    },
                ):
                    return

            yaw_delta = abs(
                math.atan2(
                    math.sin(
                        yaw_from_pose(pose)
                        - float(last_sem_yaw)
                    ),
                    math.cos(
                        yaw_from_pose(pose)
                        - float(last_sem_yaw)
                    ),
                )
            )

            yaw_threshold = math.radians(
                float(
                    self.cfg.get(
                        (
                            'ugv_yaw_trigger_deg'
                            if robot.get('type') == 'ugv'
                            else 'uav_yaw_trigger_deg'
                        ),
                        35.0,
                    )
                )
            )

            if (
                bool(
                    self.cfg.get(
                        'trigger_on_new_observation_sector',
                        True,
                    )
                )
                and yaw_delta >= yaw_threshold
            ):
                coverage = self._current_coverage(
                    robot,
                    st,
                    grid,
                )

                gain = len(
                    coverage - previous_coverage
                ) / max(
                    1,
                    len(coverage),
                )

                if gain >= float(
                    self.cfg.get(
                        'semantic_coverage_gain_trigger',
                        0.32,
                    )
                ):
                    if self._emit(
                        'NEW_OBSERVATION_SECTOR',
                        name,
                        {
                            'coverage_gain': round(
                                gain,
                                3,
                            ),
                        },
                    ):
                        return

            hist = normalized_hsv_histogram(image)

            novelty = histogram_novelty(
                hist,
                last_sem_hist,
            )

            if (
                bool(
                    self.cfg.get(
                        'trigger_on_visual_novelty',
                        True,
                    )
                )
                and novelty >= float(
                    self.cfg.get(
                        'visual_novelty_hist_trigger',
                        0.28,
                    )
                )
            ):
                if self._emit(
                    'VISUAL_NOVELTY',
                    name,
                    {
                        'histogram_novelty': round(
                            novelty,
                            3,
                        ),
                    },
                ):
                    return

            free_count = int(
                (grid.data == 0).sum()
            )

            if (
                bool(
                    self.cfg.get(
                        'trigger_on_map_free_space_expanded',
                        True,
                    )
                )
                and free_count - previous_free_count
                >= int(
                    self.cfg.get(
                        'map_new_free_cells_trigger',
                        28,
                    )
                )
            ):
                with self.lock:
                    self.states[name][
                        'last_free_count'
                    ] = free_count

                if self._emit(
                    'MAP_FREE_SPACE_EXPANDED',
                    name,
                    {
                        'new_free_cells': free_count,
                    },
                ):
                    return

            signature = self._topology_signature(
                grid,
                pose,
                float(
                    self.cfg.get(
                        'local_free_sector_radius_m',
                        1.2,
                    )
                ),
            )

            with self.lock:
                old = self.states[name][
                    'topology_signature'
                ]

                self.states[name][
                    'topology_signature'
                ] = signature

            if (
                bool(
                    self.cfg.get(
                        'trigger_on_topology_cue_changed',
                        self.cfg.get(
                            'topology_signature_change_trigger',
                            True,
                        ),
                    )
                )
                and old is not None
                and signature is not None
                and signature[0]
                >= int(
                    self.cfg.get(
                        'branch_sector_count_trigger',
                        2,
                    )
                )
                and signature != old
            ):
                self._emit(
                    'TOPOLOGY_CUE_CHANGED',
                    name,
                    {
                        'old': old,
                        'new': signature,
                    },
                )
                return


def main():
    rospy.init_node('vlm_trigger_scheduler')
    TriggerScheduler()
    rospy.spin()


if __name__ == '__main__':
    main()
