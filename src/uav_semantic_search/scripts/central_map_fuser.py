#!/usr/bin/env python3
"""Build UAV 2.5D occupancy maps using timestamped tf2 point-cloud alignment."""
from __future__ import annotations

import json
import math
import threading
import time

import numpy as np
import rospy
import sensor_msgs.point_cloud2 as pc2
import tf2_ros
from geometry_msgs.msg import PoseStamped, PoseArray
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud


def line(x0, y0, x1, y1):
    dx = abs(x1 - x0)
    sx = 1 if x0 < x1 else -1
    dy = -abs(y1 - y0)
    sy = 1 if y0 < y1 else -1
    err = dx + dy
    while True:
        yield x0, y0
        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x0 += sx
        if e2 <= dx:
            err += dx
            y0 += sy


class Fuser:
    def __init__(self):
        cfg = rospy.get_param('/map')
        runtime = rospy.get_param('/experiment_runtime', {})
        self.sync_cfg = dict(runtime.get('mapping_time_sync', {}))
        self.frame = cfg.get('frame_id', 'map')
        self.res = float(cfg['resolution'])
        self.w = int(cfg['width'])
        self.h = int(cfg['height'])
        self.ox = float(cfg['origin_x'])
        self.oy = float(cfg['origin_y'])
        self.zmin = float(cfg['z_min'])
        self.zmax = float(cfg['z_max'])
        self.rmax = float(cfg['lidar_max_range'])
        self.stride = max(1, int(cfg['point_stride']))
        self.fi = int(cfg['free_increment'])
        self.oi = int(cfg['occupied_increment'])
        self.ratio = float(cfg['occupied_ratio'])
        self.vehicles = rospy.get_param('/vehicles', [])
        self.lock = threading.RLock()
        self.pose = {}
        self.free = {v['name']: np.zeros((self.h, self.w), np.int32) for v in self.vehicles}
        self.occ = {v['name']: np.zeros((self.h, self.w), np.int32) for v in self.vehicles}
        self.local = {
            v['name']: rospy.Publisher('/%s/local_map_2d' % v['name'], OccupancyGrid,
                                       queue_size=1, latch=True)
            for v in self.vehicles
        }
        self.global_pub = rospy.Publisher('/global_map_2d', OccupancyGrid,
                                          queue_size=1, latch=True)
        self.pose_pub = rospy.Publisher('/global_map_uav_poses', PoseArray, queue_size=1)
        self.status_pub = rospy.Publisher('/mapping/uav_tf_sync_status', String,
                                          queue_size=2, latch=True)
        cache = float(self.sync_cfg.get('tf_cache_sec', 10.0))
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(cache))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.stats = {v['name']: {'processed': 0, 'dropped_tf': 0, 'last_error': ''}
                      for v in self.vehicles}
        self.last_status_wall = 0.0

        for vehicle in self.vehicles:
            rospy.Subscriber(vehicle['global_pose_topic'], PoseStamped,
                             lambda msg, name=vehicle['name']: self.pose_cb(name, msg),
                             queue_size=20)
            rospy.Subscriber(vehicle['lidar_topic'], PointCloud2,
                             lambda msg, v=vehicle: self.cloud_cb(v, msg),
                             queue_size=2, buff_size=2 ** 24)
        rospy.Timer(rospy.Duration(1.0 / max(0.1, float(cfg['publish_rate']))), self.publish)
        rospy.loginfo('Timestamp-aligned UAV 2.5D map fuser: %dx%d @ %.2fm.',
                      self.w, self.h, self.res)

    def cell(self, x, y):
        ix = int(math.floor((x - self.ox) / self.res))
        iy = int(math.floor((y - self.oy) / self.res))
        return (ix, iy) if 0 <= ix < self.w and 0 <= iy < self.h else None

    def pose_cb(self, name, msg):
        with self.lock:
            self.pose[name] = msg

    def _source_frame(self, vehicle, msg):
        if bool(self.sync_cfg.get('override_cloud_frame_from_robot_config', True)):
            return str(vehicle.get('lidar_frame_id', '%s/lidar_link' % vehicle['name']))
        return str(msg.header.frame_id).lstrip('/')

    def _transform_cloud(self, vehicle, msg):
        source = self._source_frame(vehicle, msg)
        stamp = msg.header.stamp if not msg.header.stamp.is_zero() else rospy.Time(0)
        timeout = rospy.Duration(float(self.sync_cfg.get('tf_lookup_timeout_sec', 0.12)))
        transform = self.tf_buffer.lookup_transform(self.frame, source, stamp, timeout)
        return do_transform_cloud(msg, transform), transform

    def _publish_status(self):
        now = time.monotonic()
        if now - self.last_status_wall < float(
                self.sync_cfg.get('status_publish_period_sec', 2.0)):
            return
        self.last_status_wall = now
        self.status_pub.publish(json.dumps(self.stats, sort_keys=True))

    def cloud_cb(self, vehicle, msg):
        name = str(vehicle['name'])
        try:
            cloud, transform = self._transform_cloud(vehicle, msg)
        except Exception as exc:
            with self.lock:
                self.stats[name]['dropped_tf'] += 1
                self.stats[name]['last_error'] = repr(exc)
            rospy.logwarn_throttle(2.0, '%s cloud dropped: timestamped TF unavailable: %s',
                                   name, exc)
            self._publish_status()
            return

        origin = transform.transform.translation
        c0 = self.cell(origin.x, origin.y)
        if c0 is None:
            return
        updates_free = []
        updates_occ = []
        for index, point in enumerate(pc2.read_points(
                cloud, field_names=('x', 'y', 'z'), skip_nans=True)):
            if index % self.stride:
                continue
            x, y, z = point
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue
            distance = math.sqrt((x - origin.x) ** 2 + (y - origin.y) ** 2 +
                                 (z - origin.z) ** 2)
            if distance < 0.1 or distance > self.rmax or z < self.zmin or z > self.zmax:
                continue
            c1 = self.cell(x, y)
            if c1 is None:
                continue
            cells = list(line(c0[0], c0[1], c1[0], c1[1]))
            if len(cells) < 2:
                continue
            updates_free.extend(cells[:-1])
            updates_occ.append(cells[-1])

        with self.lock:
            for cx, cy in updates_free:
                self.free[name][cy, cx] += self.fi
            for cx, cy in updates_occ:
                self.occ[name][cy, cx] += self.oi
            self.free[name][c0[1], c0[0]] += 3 * self.fi
            self.stats[name]['processed'] += 1
            self.stats[name]['last_error'] = ''
        self._publish_status()

    def msg(self, free, occ):
        observed = (free + occ) > 0
        values = np.full((self.h, self.w), -1, np.int8)
        hit = observed & (occ > self.ratio * np.maximum(free, 1))
        values[observed & ~hit] = 0
        values[hit] = 100
        msg = OccupancyGrid()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame
        msg.info.resolution = self.res
        msg.info.width = self.w
        msg.info.height = self.h
        msg.info.origin.position.x = self.ox
        msg.info.origin.position.y = self.oy
        msg.info.origin.orientation.w = 1.0
        msg.data = values.reshape(-1).tolist()
        return msg

    def publish(self, _event):
        with self.lock:
            free = np.zeros((self.h, self.w), np.int32)
            occ = np.zeros((self.h, self.w), np.int32)
            for name in self.free:
                self.local[name].publish(self.msg(self.free[name], self.occ[name]))
                free += self.free[name]
                occ += self.occ[name]
            self.global_pub.publish(self.msg(free, occ))
            poses = PoseArray()
            poses.header.stamp = rospy.Time.now()
            poses.header.frame_id = self.frame
            poses.poses = [self.pose[v['name']].pose for v in self.vehicles
                           if v['name'] in self.pose]
            self.pose_pub.publish(poses)


def main():
    rospy.init_node('central_map_fuser')
    Fuser()
    rospy.spin()


if __name__ == '__main__':
    main()
