#!/usr/bin/env python3
"""Shared UAV/UGV RGB-D red-surrogate detector with asynchronous RGB-D handling.

Why this version exists
-----------------------
Gazebo Classic's OpenNI/Kinect camera commonly publishes RGB, depth and CameraInfo
on different schedules.  Requiring a three-way message_filters synchronisation can
leave the callback idle even though all three topics are active.  This node therefore
uses:

  * RGB callback as the primary processing trigger;
  * latest depth image cached independently;
  * latest CameraInfo cached independently.

A debug image is published whenever an RGB image arrives, independently of target
presence, depth availability or map pose availability.  This makes the visual chain
observable before target localisation is tested.
"""
import threading

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo, Image
from tf.transformations import euler_matrix, quaternion_matrix

from uav_semantic_search.msg import TargetObservation


class SemanticDetector:
    def __init__(self):
        # The existing launch files keep the private parameter name ``vehicle`` for
        # compatibility.  It now identifies either an aerial vehicle in /vehicles
        # or a ground robot in /ground_robots.
        self.vehicle = rospy.get_param('~vehicle')
        aerial = rospy.get_param('/vehicles', [])
        ground = rospy.get_param('/ground_robots', [])
        robot_configs = {item['name']: item for item in (aerial + ground)}
        if self.vehicle not in robot_configs:
            raise RuntimeError(
                'Unknown semantic robot: %s. Available: %s' %
                (self.vehicle, sorted(robot_configs)))

        self.vehicle_cfg = robot_configs[self.vehicle]
        required = ('rgb_topic', 'depth_topic', 'camera_info_topic',
                    'global_pose_topic', 'camera_xyz',
                    'camera_optical_to_body_rpy')
        missing = [key for key in required if key not in self.vehicle_cfg]
        if missing:
            raise RuntimeError(
                'Semantic configuration for %s is incomplete; missing %s.' %
                (self.vehicle, ', '.join(missing)))

        self.cfg = rospy.get_param('/semantic_detector')

        self.bridge = CvBridge()
        self.lock = threading.RLock()

        self.map_pose = None
        self.camera_info = None
        self.latest_depth = None
        self.latest_depth_stamp = rospy.Time(0)
        self.last_publication = rospy.Time(0)
        self.rgb_frames_received = 0

        ns = '/' + self.vehicle
        self.pub = rospy.Publisher(
            ns + '/semantic/target_observation', TargetObservation, queue_size=10)
        self.debug_pub = rospy.Publisher(
            ns + '/semantic/debug_image', Image, queue_size=1)

        # Large RGB-D messages need a buffer larger than rospy's small default.
        image_buff_size = 2 ** 24
        rospy.Subscriber(
            self.vehicle_cfg['rgb_topic'], Image, self._rgb_cb,
            queue_size=2, buff_size=image_buff_size)
        rospy.Subscriber(
            self.vehicle_cfg['depth_topic'], Image, self._depth_cb,
            queue_size=2, buff_size=image_buff_size)
        rospy.Subscriber(
            self.vehicle_cfg['camera_info_topic'], CameraInfo,
            self._camera_info_cb, queue_size=5, buff_size=2 ** 20)
        rospy.Subscriber(
            self.vehicle_cfg['global_pose_topic'], PoseStamped,
            self._map_pose_cb, queue_size=20)

        rospy.loginfo(
            '%s semantic detector uses asynchronous RGB-D cache. RGB=%s, depth=%s, info=%s',
            self.vehicle,
            self.vehicle_cfg['rgb_topic'],
            self.vehicle_cfg['depth_topic'],
            self.vehicle_cfg['camera_info_topic'])

    def _map_pose_cb(self, msg):
        with self.lock:
            self.map_pose = msg

    def _camera_info_cb(self, msg):
        with self.lock:
            self.camera_info = msg

    def _depth_cb(self, msg):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except CvBridgeError as exc:
            rospy.logwarn_throttle(
                3.0, '%s depth conversion failed: %s', self.vehicle, exc)
            return

        with self.lock:
            # Copy because the ROS message backing buffer may be released after callback exit.
            self.latest_depth = np.array(depth, copy=True)
            self.latest_depth_stamp = msg.header.stamp

    def _red_mask(self, bgr):
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        lo1 = np.asarray(self.cfg['red_hsv_lower_1'], dtype=np.uint8)
        hi1 = np.asarray(self.cfg['red_hsv_upper_1'], dtype=np.uint8)
        lo2 = np.asarray(self.cfg['red_hsv_lower_2'], dtype=np.uint8)
        hi2 = np.asarray(self.cfg['red_hsv_upper_2'], dtype=np.uint8)
        mask = cv2.inRange(hsv, lo1, hi1) | cv2.inRange(hsv, lo2, hi2)

        k = max(1, int(self.cfg.get('morphology_kernel_px', 5)))
        if k % 2 == 0:
            k += 1
        kernel = np.ones((k, k), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        return mask

    def _detect(self, bgr):
        mask = self._red_mask(bgr)
        contour_result = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        if len(contour_result) == 2:
            # OpenCV 4.x:
            # contours, hierarchy
            contours, _ = contour_result

        elif len(contour_result) == 3:
            # OpenCV 3.x:
            # image, contours, hierarchy
            _, contours, _ = contour_result

        else:
            raise RuntimeError(
                'Unexpected cv2.findContours return length: %d'
                % len(contour_result)
            )
        if not contours:
            return None, mask

        contour = max(contours, key=cv2.contourArea)
        area = float(cv2.contourArea(contour))
        if area < float(self.cfg['min_area_px']):
            return None, mask

        x, y, w, h = cv2.boundingRect(contour)
        moments = cv2.moments(contour)
        if abs(moments['m00']) < 1e-6:
            u, v = x + w // 2, y + h // 2
        else:
            u = int(moments['m10'] / moments['m00'])
            v = int(moments['m01'] / moments['m00'])

        return {
            'bbox': (x, y, w, h),
            'u': u,
            'v': v,
            'area': area,
        }, mask

    def _depth_at(self, depth, u, v):
        radius = int(self.cfg.get('depth_patch_radius_px', 4))
        height, width = depth.shape[:2]
        x0, x1 = max(0, u - radius), min(width, u + radius + 1)
        y0, y1 = max(0, v - radius), min(height, v + radius + 1)

        patch = depth[y0:y1, x0:x1].astype(np.float32)
        if depth.dtype == np.uint16:
            patch *= 0.001

        valid = patch[np.isfinite(patch)]
        valid = valid[
            (valid >= float(self.cfg['min_depth_m'])) &
            (valid <= float(self.cfg['max_depth_m']))
        ]
        return None if valid.size == 0 else float(np.median(valid))

    def _camera_point_to_map(self, camera_point):
        with self.lock:
            pose = self.map_pose
        if pose is None:
            return None

        camera_to_body = euler_matrix(
            *[float(value) for value in self.vehicle_cfg['camera_optical_to_body_rpy']])
        camera_to_body[:3, 3] = [
            float(value) for value in self.vehicle_cfg['camera_xyz']]

        q = pose.pose.orientation
        body_to_map = quaternion_matrix([q.x, q.y, q.z, q.w])
        body_to_map[:3, 3] = [
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z,
        ]

        homogeneous = np.array(
            [camera_point[0], camera_point[1], camera_point[2], 1.0],
            dtype=np.float64)
        return body_to_map.dot(camera_to_body).dot(homogeneous)[:3]

    def _publish_debug(self, bgr, detection):
        debug = bgr.copy()
        if detection is not None:
            x, y, w, h = detection['bbox']
            cv2.rectangle(debug, (x, y), (x + w, y + h), (0, 255, 255), 2)
            cv2.circle(debug, (detection['u'], detection['v']), 4, (255, 0, 0), -1)
            cv2.putText(
                debug, 'victim candidate', (x, max(20, y - 8)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
        try:
            msg = self.bridge.cv2_to_imgmsg(debug, encoding='bgr8')
            self.debug_pub.publish(msg)
        except CvBridgeError as exc:
            rospy.logwarn_throttle(
                3.0, '%s debug image conversion failed: %s', self.vehicle, exc)

    def _rgb_cb(self, rgb_msg):
        self.rgb_frames_received += 1
        rospy.loginfo_throttle(
            5.0, '%s receives RGB frames; latest count=%d',
            self.vehicle, self.rgb_frames_received)

        now = rospy.Time.now()
        max_rate = max(0.1, float(self.cfg.get('publish_rate_hz', 5.0)))
        if (now - self.last_publication).to_sec() < 1.0 / max_rate:
            return

        try:
            bgr = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
        except CvBridgeError as exc:
            rospy.logwarn_throttle(
                3.0, '%s RGB conversion failed: %s', self.vehicle, exc)
            return

        detection, _ = self._detect(bgr)

        # This line is deliberately before all target/depth/map checks. Therefore,
        # receiving RGB must always produce a debug stream.
        self._publish_debug(bgr, detection)
        self.last_publication = now

        if detection is None:
            return

        with self.lock:
            depth = None if self.latest_depth is None else self.latest_depth.copy()
            depth_stamp = self.latest_depth_stamp
            info_msg = self.camera_info

        if depth is None:
            rospy.logwarn_throttle(
                3.0, '%s red detection exists, but no depth frame has arrived yet.',
                self.vehicle)
            return

        rgb_stamp = rgb_msg.header.stamp
        max_age = float(self.cfg.get('max_rgb_depth_age_s', 0.50))
        if (not rgb_stamp.is_zero() and not depth_stamp.is_zero() and
                abs((rgb_stamp - depth_stamp).to_sec()) > max_age):
            rospy.logwarn_throttle(
                3.0,
                '%s drops detection because RGB-depth timestamp gap exceeds %.2f s.',
                self.vehicle, max_age)
            return

        if info_msg is None:
            rospy.logwarn_throttle(
                3.0, '%s red detection exists, but CameraInfo has not arrived yet.',
                self.vehicle)
            return

        depth_m = self._depth_at(depth, detection['u'], detection['v'])
        if depth_m is None:
            return

        fx, fy, cx, cy = (
            info_msg.K[0], info_msg.K[4], info_msg.K[2], info_msg.K[5])
        if fx <= 1e-6 or fy <= 1e-6:
            rospy.logwarn_throttle(
                3.0, '%s CameraInfo intrinsics are invalid.', self.vehicle)
            return

        u, v = float(detection['u']), float(detection['v'])
        point_optical = (
            (u - cx) * depth_m / fx,
            (v - cy) * depth_m / fy,
            depth_m,
        )
        point_map = self._camera_point_to_map(point_optical)
        if point_map is None:
            rospy.logwarn_throttle(
                3.0, '%s red detection exists, but global map pose is unavailable.',
                self.vehicle)
            return

        std = (float(self.cfg['position_std_base_m']) +
               float(self.cfg['position_std_depth_scale']) * depth_m)

        msg = TargetObservation()
        msg.header.stamp = rgb_stamp if not rgb_stamp.is_zero() else now
        msg.header.frame_id = 'map'
        msg.robot_id = self.vehicle
        msg.class_name = str(self.cfg['target_class'])

        image_area = float(bgr.shape[0] * bgr.shape[1])
        msg.confidence = min(
            0.99,
            max(0.05, detection['area'] / max(1.0, 0.02 * image_area)))
        msg.pose.pose.position.x = float(point_map[0])
        msg.pose.pose.position.y = float(point_map[1])
        msg.pose.pose.position.z = float(point_map[2])
        msg.pose.pose.orientation.w = 1.0
        for index in (0, 7, 14):
            msg.pose.covariance[index] = std * std

        msg.depth_m = depth_m
        msg.pixel_u = int(detection['u'])
        msg.pixel_v = int(detection['v'])
        x, y, w, h = detection['bbox']
        msg.bbox_xmin = float(x)
        msg.bbox_ymin = float(y)
        msg.bbox_xmax = float(x + w)
        msg.bbox_ymax = float(y + h)
        self.pub.publish(msg)

        rospy.loginfo_throttle(
            1.0,
            '%s detects %s at map [%.2f, %.2f, %.2f], depth=%.2f m, c=%.2f',
            self.vehicle, msg.class_name,
            point_map[0], point_map[1], point_map[2],
            depth_m, msg.confidence)


def main():
    rospy.init_node('semantic_detector')
    SemanticDetector()
    rospy.spin()


if __name__ == '__main__':
    main()