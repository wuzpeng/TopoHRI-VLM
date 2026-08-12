#!/usr/bin/env python3
"""Publish Stage-3 frontier viewpoints and RViz markers from /global_map_2d.

This is a diagnostic node. The central manager uses the same functions directly
so it can make a consistent atomic allocation decision, while this node exposes
frontier quality and candidate viewpoints for RViz inspection.
"""
# Import the helper module from scripts/ (or the installed package bin dir),
# not from a catkin executable wrapper in devel/lib.
import os
import sys
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)

import threading

import rospy
from geometry_msgs.msg import Point, Pose, PoseArray
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray

from frontier_core import extract_frontier_clusters, occupancy_grid_from_flat
from frontier_topology import (associate_frontier_clusters,
                               build_free_space_topology, topology_summary)


class FrontierExtractor:
    def __init__(self):
        # UAV frontier、障碍膨胀和可视化参数。
        self.racer_cfg = rospy.get_param('/racer_stage3', {})

        # UGV专用的障碍膨胀、frontier和A*参数。
        self.heterogeneous_cfg = rospy.get_param('/heterogeneous_fuel', {})

        # 为了兼容原有 _topology_markers() 中对 self.cfg 的访问。
        self.cfg = self.racer_cfg

        # Central VLM候选生成器使用的拓扑参数。
        vlm_root = rospy.get_param('/vlm_semantic_search', {})
        self.topology_cfg = vlm_root.get('candidate_builder', {})

        self.lock = threading.RLock()

        # 分别保存UAV全局地图和UGV地面地图。
        self.map_msgs = {
            'uav': None,
            'ugv': None,
        }

        self.frame_id = rospy.get_param('/map/frame_id', 'map')

        # UAV继续使用原来的话题，保持向后兼容。
        self.viewpoints_pubs = {
            'uav': rospy.Publisher(
                '/search/frontier_viewpoints',
                PoseArray,
                queue_size=1,
                latch=True,
            ),
            'ugv': rospy.Publisher(
                '/search/ugv_frontier_viewpoints',
                PoseArray,
                queue_size=1,
                latch=True,
            ),
        }

        self.markers_pubs = {
            'uav': rospy.Publisher(
                '/search/frontier_markers',
                MarkerArray,
                queue_size=1,
                latch=True,
            ),
            'ugv': rospy.Publisher(
                '/search/ugv_frontier_markers',
                MarkerArray,
                queue_size=1,
                latch=True,
            ),
        }

        rospy.Subscriber(
            '/global_map_2d',
            OccupancyGrid,
            lambda msg: self._map_cb('uav', msg),
            queue_size=1,
        )

        rospy.Subscriber(
            '/ugv0/ground_map_2d',
            OccupancyGrid,
            lambda msg: self._map_cb('ugv', msg),
            queue_size=1,
        )

        rate = max(
            0.1,
            float(self.racer_cfg.get('frontier_visualization_rate_hz', 1.0)),
        )

        rospy.Timer(
            rospy.Duration(1.0 / rate),
            self._tick,
        )

        rospy.loginfo(
            'Frontier extractor ready for UAV and UGV topology maps.'
        )

    def _map_cb(self, layer, msg):
        if layer not in self.map_msgs:
            return

        with self.lock:
            self.map_msgs[layer] = msg

    def _frontier_profile(self, layer):
        """返回UAV或UGV对应的frontier提取参数。"""

        if layer == 'ugv':
            return {
                'occupied_threshold': int(
                    self.racer_cfg.get('occupied_threshold', 65)
                ),
                'inflation_radius_m': float(
                    self.heterogeneous_cfg.get(
                        'ugv_obstacle_inflation_m',
                        0.54,
                    )
                ),
                'min_frontier_length_m': float(
                    self.heterogeneous_cfg.get(
                        'ugv_min_frontier_length_m',
                        0.60,
                    )
                ),
                'gain_radius_m': float(
                    self.heterogeneous_cfg.get(
                        'ugv_gain_radius_m',
                        2.2,
                    )
                ),
                'min_clearance_m': float(
                    self.heterogeneous_cfg.get(
                        'ugv_min_clearance_m',
                        0.52,
                    )
                ),
                'hgrid_size_m': float(
                    self.heterogeneous_cfg.get(
                        'ugv_hgrid_size_m',
                        3.0,
                    )
                ),
                'sample_stride': int(
                    self.heterogeneous_cfg.get(
                        'ugv_frontier_sample_stride',
                        1,
                    )
                ),
            }

        return {
            'occupied_threshold': int(
                self.racer_cfg.get('occupied_threshold', 65)
            ),
            'inflation_radius_m': float(
                self.racer_cfg.get('obstacle_inflation_m', 0.55)
            ),
            'min_frontier_length_m': float(
                self.racer_cfg.get('min_frontier_length_m', 0.60)
            ),
            'gain_radius_m': float(
                self.racer_cfg.get('gain_radius_m', 3.20)
            ),
            'min_clearance_m': float(
                self.racer_cfg.get('min_clearance_m', 0.60)
            ),
            'hgrid_size_m': float(
                self.racer_cfg.get('hgrid_size_m', 4.0)
            ),
            'sample_stride': int(
                self.racer_cfg.get('frontier_sample_stride', 2)
            ),
        }

    def _clusters(self, msg, layer):
        grid = occupancy_grid_from_flat(
            msg.data,
            msg.info.width,
            msg.info.height,
            msg.info.resolution,
            msg.info.origin.position.x,
            msg.info.origin.position.y,
            msg.header.frame_id or self.frame_id,
        )

        profile = self._frontier_profile(layer)

        clusters, passable = extract_frontier_clusters(
            grid,
            occupied_threshold=profile['occupied_threshold'],
            inflation_radius_m=profile['inflation_radius_m'],
            min_frontier_length_m=profile['min_frontier_length_m'],
            gain_radius_m=profile['gain_radius_m'],
            min_clearance_m=profile['min_clearance_m'],
            hgrid_size_m=profile['hgrid_size_m'],
            sample_stride=profile['sample_stride'],
        )

        topology = None
        associations = {}

        topology_enabled = bool(
            self.topology_cfg.get(
                'enable_topology_frontier_regions',
                self.racer_cfg.get(
                    'enable_topology_frontier_regions',
                    True,
                ),
            )
        )

        if topology_enabled:
            topology_layer = (
                'ugv_topology'
                if layer == 'ugv'
                else 'uav_topology'
            )

            topology = build_free_space_topology(
                passable=passable,
                resolution=grid.resolution,
                layer_id=topology_layer,
                min_free_component_area_m2=float(
                    self.topology_cfg.get(
                        'topology_min_free_component_area_m2',
                        self.racer_cfg.get(
                            'topology_min_free_component_area_m2',
                            0.8,
                        ),
                    )
                ),
                spur_prune_length_m=float(
                    self.topology_cfg.get(
                        'topology_spur_prune_length_m',
                        self.racer_cfg.get(
                            'topology_spur_prune_length_m',
                            0.6,
                        ),
                    )
                ),
                min_branch_length_m=float(
                    self.topology_cfg.get(
                        'topology_min_branch_length_m',
                        self.racer_cfg.get(
                            'topology_min_branch_length_m',
                            0.4,
                        ),
                    )
                ),
                thinning_max_iterations=int(
                    self.topology_cfg.get(
                        'topology_thinning_max_iterations',
                        self.racer_cfg.get(
                            'topology_thinning_max_iterations',
                            500,
                        ),
                    )
                ),
            )

            associations = associate_frontier_clusters(
                clusters,
                topology,
                max_distance_m=float(
                    self.topology_cfg.get(
                        'topology_frontier_association_max_distance_m',
                        self.racer_cfg.get(
                            'topology_frontier_association_max_distance_m',
                            4.0,
                        ),
                    )
                ),
                high_confidence_distance_m=float(
                    self.topology_cfg.get(
                        'topology_high_confidence_distance_m',
                        self.racer_cfg.get(
                            'topology_high_confidence_distance_m',
                            1.5,
                        ),
                    )
                ),
            )

            for cluster in clusters:
                metadata = associations.get(
                    int(cluster.cluster_id),
                    {},
                )

                for key, value in metadata.items():
                    setattr(cluster, key, value)

        return (clusters, passable), grid, topology, associations

    @staticmethod
    def _marker(marker_id, frame, x, y, z, label, r, g, b, scale=0.28):
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = frame
        marker.ns = 'frontier_candidates'
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = scale
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = 0.90
        marker.lifetime = rospy.Duration(0.0)
        return marker

    @staticmethod
    def _region_color(region_id):
        palette = (
            (0.10, 0.65, 1.00), (0.20, 0.85, 0.35),
            (0.95, 0.45, 0.15), (0.65, 0.25, 0.90),
            (0.05, 0.80, 0.75), (0.95, 0.75, 0.10),
        )
        return palette[sum(ord(ch) for ch in str(region_id)) % len(palette)]

    def _topology_markers(
            self,
            topology,
            grid,
            stamp,
            z,
            layer,
    ):
        markers = []

        if topology is None:
            return markers

        prefix = (
            'ugv_topology'
            if layer == 'ugv'
            else 'uav_topology'
        )

        # UAV使用黄色，UGV使用紫红色，避免两套骨架重合时无法区分。
        if layer == 'ugv':
            skeleton_color = (0.95, 0.15, 0.85)
        else:
            skeleton_color = (1.0, 0.85, 0.05)

        if bool(
            self.racer_cfg.get(
                'topology_show_skeleton',
                True,
            )
        ):
            skeleton = Marker()

            skeleton.header.stamp = stamp
            skeleton.header.frame_id = grid.frame_id

            skeleton.ns = prefix + '_skeleton'
            skeleton.id = 900001

            skeleton.type = Marker.POINTS
            skeleton.action = Marker.ADD

            skeleton.pose.orientation.w = 1.0

            skeleton.scale.x = float(
                self.racer_cfg.get(
                    'topology_skeleton_point_scale_m',
                    0.08,
                )
            )
            skeleton.scale.y = skeleton.scale.x

            skeleton.color.r = skeleton_color[0]
            skeleton.color.g = skeleton_color[1]
            skeleton.color.b = skeleton_color[2]
            skeleton.color.a = 0.78

            ys, xs = topology.skeleton_mask.nonzero()

            for x, y in zip(
                    xs.tolist(),
                    ys.tolist(),
            ):
                wx, wy = grid.cell_to_world((x, y))

                # 注意：ROS Marker.points是Python列表，只能使用append，
                # 不能使用marker.points.add()。
                skeleton.points.append(
                    Point(
                        x=wx,
                        y=wy,
                        z=z - 0.05,
                    )
                )

            markers.append(skeleton)

        if bool(
            self.racer_cfg.get(
                'topology_show_junctions',
                True,
            )
        ):
            marker_specs = (
                (
                    prefix + '_junctions',
                    900002,
                    topology.junction_mask,
                    (1.0, 0.1, 0.1),
                    0.18,
                ),
                (
                    prefix + '_endpoints',
                    900003,
                    topology.endpoint_mask,
                    (1.0, 1.0, 1.0),
                    0.14,
                ),
            )

            for namespace, marker_id, mask, color, scale in marker_specs:
                marker = Marker()

                marker.header.stamp = stamp
                marker.header.frame_id = grid.frame_id

                marker.ns = namespace
                marker.id = marker_id

                marker.type = Marker.SPHERE_LIST
                marker.action = Marker.ADD

                marker.pose.orientation.w = 1.0

                marker.scale.x = scale
                marker.scale.y = scale
                marker.scale.z = scale

                marker.color.r = color[0]
                marker.color.g = color[1]
                marker.color.b = color[2]
                marker.color.a = 0.90

                ys, xs = mask.nonzero()

                for x, y in zip(
                        xs.tolist(),
                        ys.tolist(),
                ):
                    wx, wy = grid.cell_to_world((x, y))

                    marker.points.append(
                        Point(
                            x=wx,
                            y=wy,
                            z=z,
                        )
                    )

                markers.append(marker)

        return markers
    
    def _publish_layer(self, layer, msg):
        """计算并发布一张地图层的frontier和拓扑Marker。"""

        (clusters, _), grid, topology, associations = self._clusters(
            msg,
            layer,
        )

        stamp = rospy.Time.now()

        pose_array = PoseArray()
        pose_array.header.stamp = stamp
        pose_array.header.frame_id = grid.frame_id

        marker_array = MarkerArray()

        # UAV和UGV使用不同Marker话题，因此各自的DELETEALL不会互相删除。
        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        marker_array.markers.append(delete_all)

        if layer == 'ugv':
            z = float(
                self.racer_cfg.get(
                    'ugv_frontier_marker_z_m',
                    0.12,
                )
            )
        else:
            z = float(
                self.racer_cfg.get(
                    'frontier_marker_z_m',
                    0.20,
                )
            )

        marker_array.markers.extend(
            self._topology_markers(
                topology,
                grid,
                stamp,
                z,
                layer,
            )
        )

        for cluster in clusters:
            x, y = grid.cell_to_world(
                cluster.viewpoint_cell
            )

            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = z
            pose.orientation.w = 1.0

            pose_array.poses.append(pose)

            region_id = (
                cluster.topology_region_id
                or (
                    '%s:UNASSIGNED:F%03d'
                    % (
                        layer,
                        int(cluster.cluster_id),
                    )
                )
            )

            color = self._region_color(region_id)

            frontier_marker = self._marker(
                int(cluster.cluster_id),
                grid.frame_id,
                x,
                y,
                z,
                'F%d G=%.0f' % (
                    cluster.cluster_id,
                    cluster.information_gain,
                ),
                color[0],
                color[1],
                color[2],
            )

            frontier_marker.ns = (
                '%s_frontier_candidates' % layer
            )

            marker_array.markers.append(
                frontier_marker
            )

            label = Marker()

            label.header.stamp = stamp
            label.header.frame_id = grid.frame_id

            label.ns = '%s_frontier_labels' % layer
            label.id = int(cluster.cluster_id)

            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD

            label.pose = pose
            label.pose.position.z += 0.35

            label.scale.z = 0.24

            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 1.0

            region_label = str(region_id).split(':')[-1]

            label.text = '%s F%d / %s\nG %.0f L %.1f' % (
                layer.upper(),
                cluster.cluster_id,
                region_label,
                cluster.information_gain,
                cluster.frontier_length_m,
            )

            marker_array.markers.append(label)

        self.viewpoints_pubs[layer].publish(
            pose_array
        )

        self.markers_pubs[layer].publish(
            marker_array
        )

        if topology is not None:
            summary = topology_summary(
                topology,
                associations,
            )

            rospy.loginfo_throttle(
                5.0,
                'Topology[%s]: skeleton=%d branches=%d '
                'frontier=%d regions=%d unassigned=%d',
                layer,
                summary['skeleton_cells'],
                summary['branches'],
                summary['frontiers'],
                summary['regions'],
                summary['unassigned'],
            )

    def _tick(self, _event):
        with self.lock:
            map_msgs = dict(self.map_msgs)

        for layer in ('uav', 'ugv'):
            msg = map_msgs.get(layer)

            if msg is None or not msg.data:
                continue

            try:
                self._publish_layer(
                    layer,
                    msg,
                )

            except Exception as exc:
                rospy.logwarn_throttle(
                    3.0,
                    '%s frontier topology extraction failed: %r',
                    layer.upper(),
                    exc,
                )


def main():
    rospy.init_node('frontier_extractor')
    FrontierExtractor()
    rospy.spin()


if __name__ == '__main__':
    main()
