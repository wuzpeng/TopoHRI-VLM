#!/usr/bin/env python3
"""Record and visualize the historical trajectories of all configured robots.

Each robot is published as a coloured nav_msgs/Path.  A Marker POINTS layer
marks map cells visited by two or more different robots, so coincident paths
remain visible even when one coloured line covers another in RViz.
"""

import math
import threading

import rospy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from std_srvs.srv import Trigger, TriggerResponse
from visualization_msgs.msg import Marker, MarkerArray


class RobotTrajectoryVisualizer:
    DEFAULT_COLOURS = [
        (0.10, 0.45, 1.00),  # blue
        (1.00, 0.20, 0.15),  # red
        (0.10, 0.80, 0.30),  # green
    ]

    def __init__(self):
        self.frame_id = str(rospy.get_param('/map/frame_id', 'map'))
        # UAVs and UGVs are intentionally stored in separate parameter lists
        # by the heterogeneous launch stack.
        vehicles = list(rospy.get_param('/vehicles', []))
        vehicles.extend(rospy.get_param('/ground_robots', []))
        wanted = rospy.get_param('~robots', ['uav0', 'uav1', 'ugv0'])
        by_name = {str(v.get('name')): v for v in vehicles if v.get('name')}
        self.robots = [name for name in wanted if name in by_name]
        if not self.robots:
            raise rospy.ROSInitException('No requested robots are present in /vehicles')

        self.min_step = max(0.0, float(rospy.get_param('~min_sample_distance_m', 0.08)))
        self.overlap_radius = max(0.01, float(rospy.get_param('~overlap_radius_m', 0.30)))
        self.line_width = max(0.01, float(rospy.get_param('~line_width_m', 0.08)))
        self.overlap_size = max(0.02, float(rospy.get_param('~overlap_marker_size_m', 0.16)))
        self.publish_rate = max(0.2, float(rospy.get_param('~publish_rate_hz', 2.0)))
        self.max_points = max(0, int(rospy.get_param('~max_points_per_robot', 0)))

        colours = rospy.get_param('~colours', {})
        self.colours = {}
        for index, name in enumerate(self.robots):
            value = colours.get(name, self.DEFAULT_COLOURS[index % len(self.DEFAULT_COLOURS)])
            if not isinstance(value, (list, tuple)) or len(value) < 3:
                value = self.DEFAULT_COLOURS[index % len(self.DEFAULT_COLOURS)]
            self.colours[name] = tuple(float(max(0.0, min(1.0, x))) for x in value[:3])

        self.paths = {name: Path() for name in self.robots}
        self.last_xy = {name: None for name in self.robots}
        self.visited = {}  # quantised cell -> set(robot names)
        self.lock = threading.RLock()
        self.path_pubs = {
            name: rospy.Publisher('/trajectory/%s/path' % name, Path, queue_size=1, latch=True)
            for name in self.robots
        }
        self.marker_pub = rospy.Publisher('/trajectory/markers', MarkerArray, queue_size=1, latch=True)

        for name in self.robots:
            topic = str(by_name[name].get('global_pose_topic', '/%s/global_pose' % name))
            rospy.Subscriber(topic, PoseStamped, self._pose_cb, callback_args=name, queue_size=100)
            rospy.loginfo('Trajectory recorder: %s <- %s', name, topic)

        rospy.Service('~reset', Trigger, self._reset)
        rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self._publish)
        rospy.loginfo('Robot trajectory visualizer ready for: %s', ', '.join(self.robots))

    def _pose_cb(self, msg, name):
        if not (math.isfinite(msg.pose.position.x) and math.isfinite(msg.pose.position.y)):
            return
        xy = (msg.pose.position.x, msg.pose.position.y)
        with self.lock:
            previous = self.last_xy[name]
            if previous is not None and math.hypot(xy[0] - previous[0], xy[1] - previous[1]) < self.min_step:
                return
            sample = PoseStamped()
            sample.header = msg.header
            sample.header.frame_id = self.frame_id
            sample.pose = msg.pose
            self.paths[name].poses.append(sample)
            if self.max_points and len(self.paths[name].poses) > self.max_points:
                del self.paths[name].poses[:-self.max_points]
                self._rebuild_visited_locked()
            else:
                self._mark_visited_locked(name, xy)
            self.last_xy[name] = xy

    def _mark_visited_locked(self, name, xy):
        key = (int(round(xy[0] / self.overlap_radius)), int(round(xy[1] / self.overlap_radius)))
        self.visited.setdefault(key, set()).add(name)

    def _rebuild_visited_locked(self):
        self.visited = {}
        for name, path in self.paths.items():
            for pose in path.poses:
                self._mark_visited_locked(name, (pose.pose.position.x, pose.pose.position.y))

    def _publish(self, _event):
        now = rospy.Time.now()
        with self.lock:
            paths = {}
            for name, path in self.paths.items():
                out = Path()
                out.header.stamp = now
                out.header.frame_id = self.frame_id
                out.poses = list(path.poses)
                paths[name] = out
            visited = {key: set(names) for key, names in self.visited.items()}

        for name, path in paths.items():
            self.path_pubs[name].publish(path)
        self.marker_pub.publish(self._make_markers(now, visited))

    def _make_markers(self, now, visited):
        array = MarkerArray()
        delete = Marker()
        delete.action = Marker.DELETEALL
        array.markers.append(delete)

        # Legend labels are placed beside the first recorded point.
        with self.lock:
            first_points = {
                name: path.poses[0].pose.position if path.poses else None
                for name, path in self.paths.items()
            }
        marker_id = 1
        for index, name in enumerate(self.robots):
            p = first_points[name]
            if p is None:
                continue
            label = Marker()
            label.header.frame_id = self.frame_id
            label.header.stamp = now
            label.ns = 'trajectory_labels'
            label.id = marker_id
            marker_id += 1
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = p.x
            label.pose.position.y = p.y
            label.pose.position.z = 0.25 + 0.08 * index
            label.pose.orientation.w = 1.0
            label.scale.z = 0.35
            label.color.r, label.color.g, label.color.b = self.colours[name]
            label.color.a = 1.0
            label.text = name
            array.markers.append(label)

        overlap2 = Marker()
        overlap2.header.frame_id = self.frame_id
        overlap2.header.stamp = now
        overlap2.ns = 'trajectory_overlap'
        overlap2.id = marker_id
        marker_id += 1
        overlap2.type = Marker.POINTS
        overlap2.action = Marker.ADD
        overlap2.pose.orientation.w = 1.0
        overlap2.scale.x = self.overlap_size
        overlap2.scale.y = self.overlap_size
        overlap2.color.r, overlap2.color.g, overlap2.color.b, overlap2.color.a = (1.0, 0.85, 0.0, 1.0)

        overlap3 = Marker()
        overlap3.header.frame_id = self.frame_id
        overlap3.header.stamp = now
        overlap3.ns = 'trajectory_overlap'
        overlap3.id = marker_id
        overlap3.type = Marker.POINTS
        overlap3.action = Marker.ADD
        overlap3.pose.orientation.w = 1.0
        overlap3.scale.x = self.overlap_size * 1.45
        overlap3.scale.y = self.overlap_size * 1.45
        overlap3.color.r, overlap3.color.g, overlap3.color.b, overlap3.color.a = (1.0, 1.0, 1.0, 1.0)

        for (ix, iy), names in visited.items():
            if len(names) < 2:
                continue
            point = Point()
            point.x = ix * self.overlap_radius
            point.y = iy * self.overlap_radius
            point.z = 0.12
            (overlap3 if len(names) >= 3 else overlap2).points.append(point)
        array.markers.extend([overlap2, overlap3])
        return array

    def _reset(self, _request):
        with self.lock:
            self.paths = {name: Path() for name in self.robots}
            self.last_xy = {name: None for name in self.robots}
            self.visited = {}
        self._publish(None)
        return TriggerResponse(success=True, message='All recorded trajectories were cleared.')


if __name__ == '__main__':
    rospy.init_node('robot_trajectory_visualizer')
    RobotTrajectoryVisualizer()
    rospy.spin()
