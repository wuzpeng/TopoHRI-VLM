"""Spawn an official Unitree Go2 visual model as a navigation-level Gazebo robot.

The official ``go2_description`` package provides the Go2 mesh/URDF.  This node
keeps that geometry, converts all articulated leg joints to fixed joints at their
URDF reference pose, disables gravity, and attaches a simulated 3-D LiDAR plus a forward-facing RGB-D camera.  Motion
is intentionally supplied by ``go2_kinematic_controller.py`` through Gazebo model
pose updates; it is not a gait controller.

Why convert joints to fixed joints?
-----------------------------------
The current project studies task planning, exploration and heterogeneous
coordination rather than low-level leg dynamics.  A vanilla Go2 URDF has unactuated
leg joints in Gazebo and will collapse under gravity without a full joint/gait
controller.  Fixing the joints preserves the official visual model and collision
outline while exposing a stable navigation-level mobile platform.
"""
from __future__ import annotations

import os
import threading
import xml.etree.ElementTree as ET
from pathlib import Path

import rospkg
import rospy
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool, String
from tf.transformations import quaternion_from_euler


class OfficialGo2Spawner:
    def __init__(self):
        robots = rospy.get_param('/ground_robots', [])
        if not robots:
            raise RuntimeError('Missing /ground_robots configuration.')
        self.robot = robots[0]
        self.name = self.robot['name']
        self.model_name = self.robot['gazebo_model']
        # self.spawn_cfg = self.robot.get('spawn', {})
        # 默认出生位置来自go2_nav.yaml
        self.spawn_cfg = dict(self.robot.get('spawn', {}))

        # 实验配置可以按地图覆盖出生位置
        experiment = rospy.get_param('/experiment', {})
        spawn_overrides = experiment.get('ground_robot_spawns', {})

        if isinstance(spawn_overrides, dict):
            robot_spawn = spawn_overrides.get(self.name, {})
            if isinstance(robot_spawn, dict):
                self.spawn_cfg.update(robot_spawn)

        rospy.loginfo(
            'UGV %s spawn pose: x=%.2f, y=%.2f, z=%.2f, yaw=%.3f',
            self.name,
            float(self.spawn_cfg.get('x', 0.80)),
            float(self.spawn_cfg.get('y', 0.00)),
            float(self.spawn_cfg.get('z', 0.34)),
            float(self.spawn_cfg.get('yaw', 0.0)),
        )
        self.lock = threading.RLock()
        self.model_present = False
        self.spawn_requested = False
        self.completed = False

        self.ready_pub = rospy.Publisher('/%s/model_ready' % self.name, Bool,
                                         queue_size=1, latch=True)
        self.status_pub = rospy.Publisher('/%s/model_spawn_status' % self.name,
                                          String, queue_size=5, latch=True)
        self.ready_pub.publish(False)
        self.status_pub.publish('WAIT_GAZEBO')

        self.spawn_srv = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        rospy.Subscriber('/gazebo/model_states', ModelStates, self._model_states_cb,
                         queue_size=2)
        rospy.Timer(rospy.Duration(1.0), self._try_spawn)
        rospy.loginfo('Official Go2 spawner waits for Gazebo and package go2_description.')

    def _model_states_cb(self, msg):
        present = self.model_name in msg.name
        with self.lock:
            self.model_present = present
            if present and not self.completed:
                self.completed = True
                self.ready_pub.publish(True)
                self.status_pub.publish('READY')
                rospy.loginfo('Official Go2 model %s is present in Gazebo.', self.model_name)

    def _find_description(self):
        try:
            return Path(rospkg.RosPack().get_path('go2_description'))
        except rospkg.ResourceNotFound:
            return None

    @staticmethod
    def _set_all_leg_joints_fixed(root):
        """Turn every articulated URDF joint into a fixed joint for nav-level use."""
        for joint in root.findall('joint'):
            if joint.get('type') != 'fixed':
                joint.set('type', 'fixed')
                for tag in ('limit', 'dynamics', 'safety_controller', 'calibration', 'mimic'):
                    for child in list(joint.findall(tag)):
                        joint.remove(child)

    def _append_gazebo_nav_extensions(self, root):
        # Disable gravity on every official visual link so it cannot collapse.
        for link in root.findall('link'):
            name = link.get('name')
            if not name:
                continue
            gazebo = ET.SubElement(root, 'gazebo', {'reference': name})
            ET.SubElement(gazebo, 'gravity').text = 'false'
            ET.SubElement(gazebo, 'selfCollide').text = 'false'

        lidar_link = '%s_lidar_link' % self.name
        base_link = self.robot.get('official_base_link', 'base')
        xyz = self.robot.get('lidar_xyz', [0.18, 0.0, 0.28])
        rpy = self.robot.get('lidar_rpy', [0.0, 0.0, 0.0])

        link = ET.SubElement(root, 'link', {'name': lidar_link})
        inertial = ET.SubElement(link, 'inertial')
        ET.SubElement(inertial, 'mass', {'value': '0.05'})
        ET.SubElement(inertial, 'inertia', {
            'ixx': '0.0001', 'ixy': '0', 'ixz': '0',
            'iyy': '0.0001', 'iyz': '0', 'izz': '0.0001'})
        visual = ET.SubElement(link, 'visual')
        geometry = ET.SubElement(visual, 'geometry')
        ET.SubElement(geometry, 'cylinder', {'radius': '0.055', 'length': '0.075'})

        joint = ET.SubElement(root, 'joint', {
            'name': '%s_lidar_fixed_joint' % self.name,
            'type': 'fixed'})
        ET.SubElement(joint, 'parent', {'link': base_link})
        ET.SubElement(joint, 'child', {'link': lidar_link})
        ET.SubElement(joint, 'origin', {
            'xyz': '%.6f %.6f %.6f' % (float(xyz[0]), float(xyz[1]), float(xyz[2])),
            'rpy': '%.6f %.6f %.6f' % (float(rpy[0]), float(rpy[1]), float(rpy[2]))})

        gazebo = ET.SubElement(root, 'gazebo', {'reference': lidar_link})
        sensor = ET.SubElement(gazebo, 'sensor', {
            'name': '%s_ground_3d_lidar' % self.name,
            'type': 'ray'})
        ET.SubElement(sensor, 'always_on').text = 'true'
        ET.SubElement(sensor, 'visualize').text = 'false'
        ET.SubElement(sensor, 'update_rate').text = '10.0'
        ray = ET.SubElement(sensor, 'ray')
        scan = ET.SubElement(ray, 'scan')
        horizontal = ET.SubElement(scan, 'horizontal')
        ET.SubElement(horizontal, 'samples').text = '720'
        ET.SubElement(horizontal, 'resolution').text = '1'
        ET.SubElement(horizontal, 'min_angle').text = '-3.1415926'
        ET.SubElement(horizontal, 'max_angle').text = '3.1415926'
        vertical = ET.SubElement(scan, 'vertical')
        ET.SubElement(vertical, 'samples').text = '16'
        ET.SubElement(vertical, 'resolution').text = '1'
        ET.SubElement(vertical, 'min_angle').text = '-0.6108652'
        ET.SubElement(vertical, 'max_angle').text = '0.3490659'
        range_node = ET.SubElement(ray, 'range')
        ET.SubElement(range_node, 'min').text = '0.18'
        ET.SubElement(range_node, 'max').text = '12.0'
        ET.SubElement(range_node, 'resolution').text = '0.02'

        plugin = ET.SubElement(sensor, 'plugin', {
            'name': '%s_ros_lidar' % self.name,
            'filename': 'libgazebo_ros_velodyne_laser.so'})
        ET.SubElement(plugin, 'robotNamespace').text = '/%s' % self.name
        ET.SubElement(plugin, 'topicName').text = 'lidar/points'
        ET.SubElement(plugin, 'frameName').text = '%s/lidar_link' % self.name
        ET.SubElement(plugin, 'min_range').text = '0.18'
        ET.SubElement(plugin, 'max_range').text = '12.0'
        ET.SubElement(plugin, 'gaussianNoise').text = '0.005'

        # The official Go2 URDF already defines ``front_camera`` as a fixed link
        # on the nose of the robot (base -> front_camera).  Attach the Gazebo
        # depth-camera sensor to that real link instead of to the obsolete
        # navigation-proxy SDF model.  This keeps the camera pose exactly aligned
        # with the official visual model.
        camera_link = self.robot.get('official_front_camera_link', 'front_camera')
        camera_link_element = root.find("link[@name='%s']" % camera_link)
        if camera_link_element is None:
            raise RuntimeError(
                'Official Go2 URDF has no front camera link %r.' % camera_link)

        rgbd_cfg = self.robot.get('rgbd', {})
        update_rate = float(rgbd_cfg.get('update_rate_hz', 15.0))
        width = int(rgbd_cfg.get('width', 640))
        height = int(rgbd_cfg.get('height', 480))
        horizontal_fov = float(rgbd_cfg.get('horizontal_fov_rad', 1.3962634))
        near_clip = float(rgbd_cfg.get('near_clip_m', 0.25))
        far_clip = float(rgbd_cfg.get('far_clip_m', 8.0))
        baseline = float(rgbd_cfg.get('baseline_m', 0.07))

        camera_gazebo = ET.SubElement(root, 'gazebo', {'reference': camera_link})
        camera_sensor = ET.SubElement(camera_gazebo, 'sensor', {
            'name': '%s_front_rgbd' % self.name,
            'type': 'depth'})
        ET.SubElement(camera_sensor, 'always_on').text = 'true'
        ET.SubElement(camera_sensor, 'visualize').text = 'false'
        ET.SubElement(camera_sensor, 'update_rate').text = '%.6f' % update_rate

        camera = ET.SubElement(camera_sensor, 'camera')
        ET.SubElement(camera, 'horizontal_fov').text = '%.7f' % horizontal_fov
        image = ET.SubElement(camera, 'image')
        ET.SubElement(image, 'width').text = str(width)
        ET.SubElement(image, 'height').text = str(height)
        ET.SubElement(image, 'format').text = 'R8G8B8'
        clip = ET.SubElement(camera, 'clip')
        ET.SubElement(clip, 'near').text = '%.6f' % near_clip
        ET.SubElement(clip, 'far').text = '%.6f' % far_clip

        camera_plugin = ET.SubElement(camera_sensor, 'plugin', {
            'name': '%s_front_ros_rgbd' % self.name,
            'filename': 'libgazebo_ros_openni_kinect.so'})
        ET.SubElement(camera_plugin, 'robotNamespace').text = '/%s' % self.name
        # gazebo_ros_openni_kinect creates a cameraName namespace before applying
        # imageTopicName.  Therefore topic names below are relative (rgb/...
        # rather than front_camera/rgb/...), avoiding the old camera/camera path.
        ET.SubElement(camera_plugin, 'cameraName').text = 'front_camera'
        ET.SubElement(camera_plugin, 'alwaysOn').text = 'true'
        # Parent sensor update_rate governs frame frequency.  Zero disables an
        # additional plugin-side rate limiter.
        ET.SubElement(camera_plugin, 'updateRate').text = '0.0'
        ET.SubElement(camera_plugin, 'baseline').text = '%.6f' % baseline
        ET.SubElement(camera_plugin, 'imageTopicName').text = 'rgb/image_raw'
        ET.SubElement(camera_plugin, 'cameraInfoTopicName').text = 'rgb/camera_info'
        ET.SubElement(camera_plugin, 'depthImageTopicName').text = 'depth/image_raw'
        ET.SubElement(camera_plugin, 'depthImageCameraInfoTopicName').text = 'depth/camera_info'
        ET.SubElement(camera_plugin, 'pointCloudTopicName').text = 'depth/points'
        ET.SubElement(camera_plugin, 'frameName').text = camera_link
        ET.SubElement(camera_plugin, 'pointCloudCutoff').text = '%.6f' % near_clip
        ET.SubElement(camera_plugin, 'pointCloudCutoffMax').text = '%.6f' % far_clip

    def _build_navigation_urdf(self, description_path):
        urdf_path = description_path / 'urdf' / 'go2_description.urdf'
        if not urdf_path.is_file():
            raise RuntimeError('Official Go2 URDF is missing: %s' % urdf_path)
        root = ET.fromstring(urdf_path.read_text())
        root.set('name', self.model_name)
        self._set_all_leg_joints_fixed(root)
        self._append_gazebo_nav_extensions(root)
        xml_text = ET.tostring(root, encoding='unicode')
        # Gazebo's URDF loader resolves file URIs deterministically even when its
        # package-path environment differs from the roslaunch terminal.
        xml_text = xml_text.replace(
            'package://go2_description/',
            'file://%s/' % str(description_path))
        return xml_text

    def _try_spawn(self, _event):
        with self.lock:
            if self.completed or self.model_present or self.spawn_requested:
                return
        try:
            rospy.wait_for_service('/gazebo/spawn_urdf_model', timeout=0.2)
        except rospy.ROSException:
            return

        description_path = self._find_description()
        if description_path is None:
            message = ('MISSING_GO2_DESCRIPTION: run '
                       'scripts/install_official_go2_description.sh and catkin_make.')
            self.status_pub.publish(message)
            rospy.logerr_throttle(8.0, message)
            return

        try:
            urdf_xml = self._build_navigation_urdf(description_path)
        except Exception as exc:
            self.status_pub.publish('URDF_BUILD_ERROR')
            rospy.logerr_throttle(5.0, 'Cannot prepare official Go2 navigation URDF: %r', exc)
            return

        pose = Pose()
        pose.position.x = float(self.spawn_cfg.get('x', 0.80))
        pose.position.y = float(self.spawn_cfg.get('y', 0.00))
        pose.position.z = float(self.spawn_cfg.get('z', 0.34))
        q = quaternion_from_euler(0.0, 0.0, float(self.spawn_cfg.get('yaw', 0.0)))
        pose.orientation.x, pose.orientation.y = q[0], q[1]
        pose.orientation.z, pose.orientation.w = q[2], q[3]

        with self.lock:
            self.spawn_requested = True
        self.status_pub.publish('SPAWNING_OFFICIAL_GO2')
        try:
            response = self.spawn_srv(self.model_name, urdf_xml, '/%s' % self.name,
                                      pose, 'world')
        except rospy.ServiceException as exc:
            with self.lock:
                self.spawn_requested = False
            self.status_pub.publish('SPAWN_SERVICE_ERROR')
            rospy.logwarn_throttle(3.0, 'Official Go2 spawn service error: %s', exc)
            return

        if response.success:
            self.status_pub.publish('SPAWN_REQUEST_ACCEPTED')
            rospy.loginfo('Official Unitree Go2 spawn request accepted for %s.', self.model_name)
        else:
            with self.lock:
                self.spawn_requested = False
            self.status_pub.publish('SPAWN_REJECTED')
            rospy.logerr_throttle(4.0, 'Official Go2 spawn rejected: %s', response.status_message)


def main():
    rospy.init_node('go2_nav_spawner')
    OfficialGo2Spawner()
    rospy.spin()


if __name__ == '__main__':
    main()