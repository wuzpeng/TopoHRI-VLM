#!/usr/bin/env python3
"""Publish UAV Gazebo truth poses and timestamped TF for simulation mapping.

Dynamic TF: map -> uavX/base_link
Static TF:  uavX/base_link -> uavX/lidar_link and camera optical frame
"""
import rospy
import tf2_ros
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf.transformations import quaternion_from_euler


class Bridge:
    def __init__(self):
        self.frame = rospy.get_param('/map/frame_id', 'map')
        self.vehicles = rospy.get_param('/vehicles', [])
        if not self.vehicles:
            raise RuntimeError('Missing /vehicles configuration.')
        self.by_model = {str(v['gazebo_model']): v for v in self.vehicles}
        self.pubs = {
            model: rospy.Publisher(str(v['global_pose_topic']), PoseStamped, queue_size=10)
            for model, v in self.by_model.items()
        }
        self.dynamic_tf = tf2_ros.TransformBroadcaster()
        self.static_tf = tf2_ros.StaticTransformBroadcaster()
        self.last_tf_stamp_ns = {str(v['name']): -1 for v in self.vehicles}
        self.last_clock_ns = -1
        self._publish_static_sensor_transforms()
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.cb, queue_size=2)
        rospy.loginfo('Pose bridge waits for Gazebo models: %s', list(self.by_model.keys()))

    @staticmethod
    def _static_transform(parent, child, xyz, rpy):
        tfm = TransformStamped()
        tfm.header.stamp = rospy.Time.now()
        tfm.header.frame_id = parent
        tfm.child_frame_id = child
        tfm.transform.translation.x = float(xyz[0])
        tfm.transform.translation.y = float(xyz[1])
        tfm.transform.translation.z = float(xyz[2])
        q = quaternion_from_euler(float(rpy[0]), float(rpy[1]), float(rpy[2]))
        tfm.transform.rotation.x, tfm.transform.rotation.y = q[0], q[1]
        tfm.transform.rotation.z, tfm.transform.rotation.w = q[2], q[3]
        return tfm

    def _publish_static_sensor_transforms(self):
        transforms = []
        for vehicle in self.vehicles:
            name = str(vehicle['name'])
            base = '%s/base_link' % name
            transforms.append(self._static_transform(
                base,
                str(vehicle.get('lidar_frame_id', '%s/lidar_link' % name)),
                vehicle.get('lidar_xyz', [0.0, 0.0, 0.16]),
                vehicle.get('lidar_rpy', [0.0, 0.0, 0.0]),
            ))
            transforms.append(self._static_transform(
                base,
                str(vehicle.get('camera_optical_frame_id', '%s/camera_optical_frame' % name)),
                vehicle.get('camera_xyz', [0.18, 0.0, -0.04]),
                vehicle.get('camera_optical_to_body_rpy',
                            [-1.57079632679, 0.0, -1.57079632679]),
            ))
        if transforms:
            self.static_tf.sendTransform(transforms)

    def _handle_clock_reset(self, stamp_ns):
        if self.last_clock_ns >= 0 and stamp_ns < self.last_clock_ns:
            rospy.logwarn('Gazebo simulation time reset: reset pose-bridge TF guards.')
            for vehicle in self.last_tf_stamp_ns:
                self.last_tf_stamp_ns[vehicle] = -1
            self._publish_static_sensor_transforms()
        self.last_clock_ns = stamp_ns

    def cb(self, msg):
        lookup = {model_name: index for index, model_name in enumerate(msg.name)}
        stamp = rospy.Time.now()
        if stamp.is_zero():
            return
        stamp_ns = stamp.to_nsec()
        self._handle_clock_reset(stamp_ns)
        for model, vehicle in self.by_model.items():
            if model not in lookup:
                continue
            name = str(vehicle['name'])
            pose = msg.pose[lookup[model]]
            pose_msg = PoseStamped()
            pose_msg.header.stamp = stamp
            pose_msg.header.frame_id = self.frame
            pose_msg.pose = pose
            self.pubs[model].publish(pose_msg)
            if stamp_ns <= self.last_tf_stamp_ns[name]:
                continue
            tf_msg = TransformStamped()
            tf_msg.header = pose_msg.header
            tf_msg.child_frame_id = '%s/base_link' % name
            tf_msg.transform.translation.x = pose.position.x
            tf_msg.transform.translation.y = pose.position.y
            tf_msg.transform.translation.z = pose.position.z
            tf_msg.transform.rotation = pose.orientation
            self.dynamic_tf.sendTransform(tf_msg)
            self.last_tf_stamp_ns[name] = stamp_ns


def main():
    rospy.init_node('gazebo_pose_bridge')
    Bridge()
    rospy.spin()


if __name__ == '__main__':
    main()
