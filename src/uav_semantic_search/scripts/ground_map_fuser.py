#!/usr/bin/env python3
"""Timestamp-aligned Go2 ground-traversability occupancy mapper."""
from __future__ import annotations

import json
import math
import threading
import time

import numpy as np
import rospy
import sensor_msgs.point_cloud2 as pc2
import tf2_ros
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud


def bresenham(x0, y0, x1, y1):
    cells = []
    dx, dy = abs(x1 - x0), -abs(y1 - y0)
    sx, sy = (1 if x0 < x1 else -1), (1 if y0 < y1 else -1)
    err = dx + dy
    while True:
        cells.append((x0, y0))
        if x0 == x1 and y0 == y1:
            return cells
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x0 += sx
        if e2 <= dx:
            err += dx
            y0 += sy


class GroundMapFuser:
    def __init__(self):
        robots = rospy.get_param('/ground_robots', [])
        if not robots:
            raise RuntimeError('Missing /ground_robots configuration.')
        self.robots = {str(robot['name']): robot for robot in robots}
        self.cfg = rospy.get_param('/ground_map')
        runtime = rospy.get_param('/experiment_runtime', {})
        self.sync_cfg = dict(runtime.get('mapping_time_sync', {}))
        self.frame = self.cfg.get('frame_id', 'map')
        self.res = float(self.cfg['resolution'])
        self.w = int(self.cfg['width'])
        self.h = int(self.cfg['height'])
        self.ox = float(self.cfg['origin_x'])
        self.oy = float(self.cfg['origin_y'])
        self.zmin = float(self.cfg['obstacle_z_min'])
        self.zmax = float(self.cfg['obstacle_z_max'])
        self.rmax = float(self.cfg['lidar_max_range'])
        self.body_filter_radius = float(
            self.cfg.get('robot_body_filter_radius', 0.65)
        )
        self.stride = max(1, int(self.cfg.get('point_stride', 1)))
        self.fi = int(self.cfg.get('free_increment', 1))
        self.oi = int(self.cfg.get('occupied_increment', 5))
        self.ratio = float(self.cfg.get('occupied_ratio', 0.75))

        self.max_range_margin = float(
            self.cfg.get('max_range_no_hit_margin', 0.20)
        )

        # 自由射线对历史占据证据的主动清除强度
        self.occ_clear_increment = int(
            self.cfg.get('occupied_clear_increment', 2)
        )

        # 新障碍命中时，降低该栅格旧的自由证据
        self.free_clear_increment = int(
            self.cfg.get('free_clear_increment', 1)
        )

        # 防止证据无限累积，导致历史误检无法清除
        self.max_evidence = int(
            self.cfg.get('max_evidence', 100)
        )

        self.lock = threading.RLock()
        self.free = {name: np.zeros((self.h, self.w), np.int32) for name in self.robots}
        self.occ = {name: np.zeros((self.h, self.w), np.int32) for name in self.robots}
        self.pubs = {
            name: rospy.Publisher(robot['ground_map_topic'], OccupancyGrid,
                                  queue_size=1, latch=True)
            for name, robot in self.robots.items()
        }
        self.global_pub = rospy.Publisher('/ground_map_2d', OccupancyGrid,
                                          queue_size=1, latch=True)
        self.status_pub = rospy.Publisher('/mapping/ugv_tf_sync_status', String,
                                          queue_size=2, latch=True)
        cache = float(self.sync_cfg.get('tf_cache_sec', 10.0))
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(cache))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.stats = {name: {'processed': 0, 'dropped_tf': 0, 'last_error': ''}
                      for name in self.robots}
        self.last_status_wall = 0.0
        for name, robot in self.robots.items():
            rospy.Subscriber(robot['lidar_topic'], PointCloud2,
                             lambda msg, r=robot: self._cloud_cb(r, msg),
                             queue_size=2, buff_size=2 ** 24)
        rate = max(0.1, float(self.cfg.get('publish_rate', 2.0)))
        rospy.Timer(rospy.Duration(1.0 / rate), self._publish)
        rospy.loginfo('Timestamp-aligned Go2 ground map: %dx%d @ %.2fm, z=[%.2f, %.2f].',
                      self.w, self.h, self.res, self.zmin, self.zmax)

    def _cell(self, x, y):
        ix = int(math.floor((x - self.ox) / self.res))
        iy = int(math.floor((y - self.oy) / self.res))
        return (ix, iy) if 0 <= ix < self.w and 0 <= iy < self.h else None

    def _source_frame(self, robot, msg):
        if bool(self.sync_cfg.get('override_cloud_frame_from_robot_config', True)):
            return str(robot.get('lidar_frame_id', '%s/lidar_link' % robot['name']))
        return str(msg.header.frame_id).lstrip('/')

    def _publish_status(self):
        now = time.monotonic()
        if now - self.last_status_wall < float(
                self.sync_cfg.get('status_publish_period_sec', 2.0)):
            return
        self.last_status_wall = now
        self.status_pub.publish(json.dumps(self.stats, sort_keys=True))

    def _cloud_cb(self, robot, msg):
        name = str(robot['name'])
        source = self._source_frame(robot, msg)
        stamp = msg.header.stamp if not msg.header.stamp.is_zero() else rospy.Time(0)
        timeout = rospy.Duration(float(self.sync_cfg.get('tf_lookup_timeout_sec', 0.12)))
        try:
            transform = self.tf_buffer.lookup_transform(self.frame, source, stamp, timeout)
            cloud = do_transform_cloud(msg, transform)
        except Exception as exc:
            with self.lock:
                self.stats[name]['dropped_tf'] += 1
                self.stats[name]['last_error'] = repr(exc)
            rospy.logwarn_throttle(2.0, '%s ground cloud dropped: timestamped TF unavailable: %s',
                                   name, exc)
            self._publish_status()
            return

        origin = transform.transform.translation
        c0 = self._cell(origin.x, origin.y)
        if c0 is None:
            return
        free_updates = []
        occ_updates = []
        for index, point in enumerate(pc2.read_points(
                cloud, field_names=('x', 'y', 'z'), skip_nans=True)):
            if index % self.stride:
                continue
            x, y, z = point
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue
            distance = math.sqrt(
                (x - origin.x) ** 2
                + (y - origin.y) ** 2
                + (z - origin.z) ** 2
            )

            # 过滤UGV车体、腿部和雷达安装结构产生的近距离回波。
            # 必须在生成占据端点之前执行，否则机器人运动后会留下黑色轨迹。
            horizontal_distance = math.hypot(
                x - origin.x,
                y - origin.y
            )

            if horizontal_distance < self.body_filter_radius:
                continue

            # 明显超过配置量程的异常点直接丢弃
            if distance > self.rmax + self.max_range_margin:
                continue

            # 接近最大量程的点通常代表无有效回波，
            # 只能用于清理自由空间，不能标记为障碍物
            is_max_range_no_hit = (
                distance >= self.rmax - self.max_range_margin
            )
            c1 = self._cell(x, y)
            if c1 is None:
                continue

            cells = bresenham(c0[0], c0[1], c1[0], c1[1])
            if len(cells) < 2:
                continue
            if is_max_range_no_hit:
                # 无回波：整条射线都表示自由空间
                free_updates.extend(cells)

            elif self.zmin <= z <= self.zmax:
                # 有效低矮障碍：命中点之前为空闲，端点为占据
                free_updates.extend(cells[:-1])
                occ_updates.append(cells[-1])

            else:
                # 地面点或高于UGV障碍高度的点不作为二维障碍
                free_updates.extend(cells)

        with self.lock:
            # 当前射线确认是自由空间时：
            # 增加自由证据，同时主动清除历史占据证据
            for cx, cy in free_updates:
                self.free[name][cy, cx] = min(
                    self.max_evidence,
                    self.free[name][cy, cx] + self.fi
                )

            # 当前射线确认命中障碍时：
            # 增加占据证据，同时降低旧的自由证据
            for cx, cy in occ_updates:
                self.occ[name][cy, cx] = min(
                    self.max_evidence,
                    self.occ[name][cy, cx] + self.oi
                )

                # 障碍命中后减少旧自由证据，但不必一次清零
                self.free[name][cy, cx] = max(
                    0,
                    self.free[name][cy, cx] - self.free_clear_increment
                )

            self.free[name][c0[1], c0[0]] = min(
                self.max_evidence,
                self.free[name][c0[1], c0[0]] + 3 * self.fi
            )

            self.stats[name]['processed'] += 1
            self.stats[name]['last_error'] = ''

        self._publish_status()

    def _msg(self, free, occ):
        observed = (free > 0) | (occ > 0)

        occupied = (
            (occ > 0)
            & (occ > self.ratio * np.maximum(free, 1))
        )

        values = np.full(
            (self.h, self.w),
            -1,
            dtype=np.int8
        )
        values[observed & ~occupied] = 0
        values[occupied] = 100

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
    
    def _publish(self, _event):
        with self.lock:
            total_free = np.zeros((self.h, self.w), np.int32)
            total_occ = np.zeros((self.h, self.w), np.int32)
            for name in self.robots:
                msg = self._msg(self.free[name], self.occ[name])
                self.pubs[name].publish(msg)
                total_free += self.free[name]
                total_occ += self.occ[name]
            self.global_pub.publish(self._msg(total_free, total_occ))


def main():
    rospy.init_node('ground_map_fuser')
    GroundMapFuser()
    rospy.spin()


if __name__ == '__main__':
    main()
