#!/usr/bin/env python3
"""Central association, temporal verification, and visualization of semantic targets."""
import math
import threading

import numpy as np
import rospy
from visualization_msgs.msg import Marker, MarkerArray

from uav_semantic_search.msg import TargetHypothesis, TargetHypothesisArray, TargetObservation


class Hypothesis:
    def __init__(self, target_id, obs, now):
        self.target_id = target_id
        self.class_name = obs.class_name
        self.mean = np.array([obs.pose.pose.position.x, obs.pose.pose.position.y, obs.pose.pose.position.z], dtype=np.float64)
        self.information = np.eye(3, dtype=np.float64) / max(1e-4, self._variance_from_obs(obs))
        self.observation_count = 0
        self.observed_by = set()
        self.confidence_complement = 1.0
        self.first_seen = now
        self.last_seen = now
        self.update(obs, now)

    @staticmethod
    def _variance_from_obs(obs):
        raw = [obs.pose.covariance[i] for i in (0, 7, 14)]
        values = [float(v) for v in raw if float(v) > 1e-8]
        return float(sum(values) / len(values)) if values else 0.25

    def update(self, obs, now):
        z = np.array([obs.pose.pose.position.x, obs.pose.pose.position.y, obs.pose.pose.position.z], dtype=np.float64)
        variance = max(1e-4, self._variance_from_obs(obs))
        measurement_information = np.eye(3, dtype=np.float64) / variance
        rhs = self.information.dot(self.mean) + measurement_information.dot(z)
        self.information = self.information + measurement_information
        self.mean = np.linalg.solve(self.information, rhs)
        self.observation_count += 1
        self.observed_by.add(obs.robot_id)
        self.confidence_complement *= max(0.01, 1.0 - min(0.99, max(0.0, float(obs.confidence))))
        self.last_seen = now

    @property
    def covariance(self):
        return np.linalg.inv(self.information)

    @property
    def confidence(self):
        return float(1.0 - self.confidence_complement)


class TargetFusion:
    def __init__(self):
        self.cfg = rospy.get_param('/target_fusion')

        # Fuse semantic observations from both aerial robots and ground robots.
        # De-duplicate names defensively because the topic namespace is derived
        # from the robot name.
        all_robots = (rospy.get_param('/vehicles', []) +
                      rospy.get_param('/ground_robots', []))
        seen = set()
        self.robots = []
        for robot in all_robots:
            name = robot.get('name')
            if not name or name in seen:
                continue
            seen.add(name)
            self.robots.append(robot)

        self.lock = threading.RLock()
        self.hypotheses = []
        self.next_id = 1
        self.pub = rospy.Publisher('/semantic_map/target_hypotheses', TargetHypothesisArray, queue_size=5, latch=True)
        self.confirmed_pub = rospy.Publisher('/semantic_map/confirmed_targets', TargetHypothesisArray, queue_size=5, latch=True)
        self.marker_pub = rospy.Publisher('/semantic_map/target_markers', MarkerArray, queue_size=5, latch=True)
        for robot in self.robots:
            rospy.Subscriber('/%s/semantic/target_observation' % robot['name'],
                             TargetObservation, self._obs_cb, queue_size=30)
        rospy.Timer(rospy.Duration(1.0 / max(0.1, float(self.cfg['publish_rate_hz']))), self._publish)
        rospy.loginfo(
            'Target fusion node subscribes to %d semantic observation streams: %s',
            len(self.robots), [robot['name'] for robot in self.robots])

    def _association_distance(self, hypo, obs):
        position = np.array([obs.pose.pose.position.x, obs.pose.pose.position.y, obs.pose.pose.position.z], dtype=np.float64)
        return float(np.linalg.norm(hypo.mean - position))

    def _obs_cb(self, obs):
        now = rospy.Time.now()
        with self.lock:
            candidates = [h for h in self.hypotheses if h.class_name == obs.class_name]
            nearest = min(candidates, key=lambda h: self._association_distance(h, obs), default=None)
            if nearest is not None and self._association_distance(nearest, obs) <= float(self.cfg['association_radius_m']):
                nearest.update(obs, now)
            else:
                self.hypotheses.append(Hypothesis(self.next_id, obs, now))
                self.next_id += 1

    def _status(self, hypo):
        n_obs = hypo.observation_count
        n_agents = len(hypo.observed_by)
        if n_agents >= int(self.cfg['confirm_min_robots']) and n_obs >= int(self.cfg['confirm_min_observations']):
            return 'confirmed'
        if n_obs >= int(self.cfg['force_confirm_observations']):
            return 'confirmed'
        if n_obs >= int(self.cfg['verify_min_observations']):
            return 'verified'
        return 'candidate'

    def _to_msg(self, hypo, now):
        msg = TargetHypothesis()
        msg.header.stamp = now
        msg.header.frame_id = 'map'
        msg.target_id = hypo.target_id
        msg.class_name = hypo.class_name
        msg.pose.pose.position.x = float(hypo.mean[0])
        msg.pose.pose.position.y = float(hypo.mean[1])
        msg.pose.pose.position.z = float(hypo.mean[2])
        msg.pose.pose.orientation.w = 1.0
        covariance = hypo.covariance
        for index, value in zip((0, 7, 14), np.diag(covariance)):
            msg.pose.covariance[index] = float(min(float(self.cfg['max_covariance_m2']), max(1e-5, value)))
        msg.confidence = hypo.confidence
        msg.observation_count = hypo.observation_count
        msg.observed_by = sorted(hypo.observed_by)
        msg.status = self._status(hypo)
        return msg

    @staticmethod
    def _marker_color(status):
        if status == 'confirmed':
            return 0.1, 0.9, 0.1
        if status == 'verified':
            return 1.0, 0.7, 0.1
        return 1.0, 0.15, 0.1

    def _markers(self, messages, now):
        result = MarkerArray()
        for msg in messages:
            r, g, b = self._marker_color(msg.status)
            sphere = Marker()
            sphere.header = msg.header
            sphere.ns = 'semantic_targets'
            sphere.id = int(msg.target_id) * 2
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose = msg.pose.pose
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.42
            sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = r, g, b, 0.90
            sphere.lifetime = rospy.Duration(0.0)
            result.markers.append(sphere)

            label = Marker()
            label.header = msg.header
            label.ns = 'semantic_target_labels'
            label.id = int(msg.target_id) * 2 + 1
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose = msg.pose.pose
            label.pose.position.z += 0.55
            label.scale.z = 0.32
            label.color.r, label.color.g, label.color.b, label.color.a = 1.0, 1.0, 1.0, 1.0
            label.text = '#%d %s\n%s c=%.2f n=%d' % (msg.target_id, msg.class_name, msg.status,
                                                       msg.confidence, msg.observation_count)
            result.markers.append(label)
        return result

    def _publish(self, _event):
        now = rospy.Time.now()
        stale = float(self.cfg['stale_timeout_sec'])
        remove = float(self.cfg['remove_timeout_sec'])
        with self.lock:
            live = []
            for hypo in self.hypotheses:
                age = (now - hypo.last_seen).to_sec()
                if age <= remove:
                    live.append(hypo)
            self.hypotheses = live
            messages = []
            for hypo in self.hypotheses:
                age = (now - hypo.last_seen).to_sec()
                if age <= stale:
                    messages.append(self._to_msg(hypo, now))
            array = TargetHypothesisArray()
            array.header.stamp = now
            array.header.frame_id = 'map'
            array.hypotheses = messages
            confirmed = TargetHypothesisArray()
            confirmed.header = array.header
            confirmed.hypotheses = [msg for msg in messages if msg.status == 'confirmed']
            self.pub.publish(array)
            self.confirmed_pub.publish(confirmed)
            self.marker_pub.publish(self._markers(messages, now))


def main():
    rospy.init_node('target_fusion_node')
    TargetFusion()
    rospy.spin()


if __name__ == '__main__':
    main()