#!/usr/bin/env python3
"""Install Stage-2 LiDAR/RGB-D blocks and the corridor world into one PX4 Gazebo Classic tree.

The script is idempotent. It preserves one backup named *.uav_semantic_search.bak and
updates an older v0.1 sensor block in place when the marker already exists.
"""
import argparse
import shutil
from pathlib import Path

MARK = '<!-- UAV_SEMANTIC_SEARCH_SENSOR_BLOCK -->'
BLOCK = r"""
  <!-- UAV_SEMANTIC_SEARCH_SENSOR_BLOCK -->
  <link name='uav_semantic_lidar_link'>
    <pose>0 0 0.16 0 0 0</pose>
    <inertial><mass>0.020</mass><inertia><ixx>0.00001</ixx><iyy>0.00001</iyy><izz>0.00001</izz></inertia></inertial>
    <sensor name='uav_semantic_3d_lidar' type='ray'>
      <always_on>1</always_on><visualize>false</visualize><update_rate>10</update_rate>
      <ray><scan><horizontal><samples>720</samples><resolution>1</resolution><min_angle>-3.1415926</min_angle><max_angle>3.1415926</max_angle></horizontal><vertical><samples>16</samples><resolution>1</resolution><min_angle>-0.436332</min_angle><max_angle>0.436332</max_angle></vertical></scan><range><min>0.30</min><max>20.0</max><resolution>0.02</resolution></range></ray>
      <plugin name='uav_semantic_ros_lidar' filename='libgazebo_ros_velodyne_laser.so'>
        <robotNamespace>/uav{{ (mavlink_id|int) - 1 }}</robotNamespace>
        <topicName>lidar/points</topicName><frameName>uav{{ (mavlink_id|int) - 1 }}/lidar_link</frameName>
        <min_range>0.30</min_range><max_range>20.0</max_range><gaussianNoise>0.01</gaussianNoise>
      </plugin>
    </sensor>
  </link>
  <joint name='uav_semantic_lidar_joint' type='fixed'><parent>base_link</parent><child>uav_semantic_lidar_link</child></joint>
  <link name='uav_semantic_camera_link'>
    <pose>0.18 0 -0.04 0 0 0</pose>
    <inertial><mass>0.035</mass><inertia><ixx>0.00002</ixx><iyy>0.00002</iyy><izz>0.00002</izz></inertia></inertial>
    <sensor name='uav_semantic_rgbd' type='depth'>
      <always_on>1</always_on><visualize>false</visualize><update_rate>15</update_rate>
      <camera><horizontal_fov>1.3962634</horizontal_fov><image><width>640</width><height>480</height><format>R8G8B8</format></image><clip><near>0.25</near><far>8.0</far></clip></camera>
      <plugin name='uav_semantic_ros_rgbd' filename='libgazebo_ros_openni_kinect.so'>
        <robotNamespace>/uav{{ (mavlink_id|int) - 1 }}</robotNamespace><cameraName>camera</cameraName><alwaysOn>true</alwaysOn><updateRate>15.0</updateRate>
        <imageTopicName>camera/rgb/image_raw</imageTopicName><cameraInfoTopicName>camera/rgb/camera_info</cameraInfoTopicName><depthImageTopicName>camera/depth/image_raw</depthImageTopicName><depthImageCameraInfoTopicName>camera/depth/camera_info</depthImageCameraInfoTopicName><pointCloudTopicName>camera/depth/points</pointCloudTopicName><frameName>uav{{ (mavlink_id|int) - 1 }}/camera_optical_frame</frameName><pointCloudCutoff>0.25</pointCloudCutoff><pointCloudCutoffMax>8.0</pointCloudCutoffMax>
      </plugin>
    </sensor>
  </link>
  <joint name='uav_semantic_camera_joint' type='fixed'><parent>base_link</parent><child>uav_semantic_camera_link</child></joint>
"""


def discover_layout(root):
    for base in (root / 'Tools' / 'sitl_gazebo',
                 root / 'Tools' / 'simulation' / 'gazebo-classic' / 'sitl_gazebo-classic'):
        iris = base / 'models' / 'iris'
        if iris.exists():
            return base, iris
    raise FileNotFoundError('Gazebo Classic iris model was not found under %s' % root)


def remove_existing_block(text):
    start = text.find(MARK)
    if start < 0:
        return text, False
    end = text.find('</model>', start)
    if end < 0:
        raise RuntimeError('Sensor marker exists but final </model> is missing.')
    return text[:start] + text[end:], True


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--px4-root', required=True)
    args = parser.parse_args()
    root = Path(args.px4_root).expanduser().resolve()
    base, iris = discover_layout(root)
    model = next((path for path in (iris / 'iris.sdf.jinja', iris / 'iris.sdf') if path.exists()), None)
    if model is None:
        raise FileNotFoundError('iris.sdf.jinja or iris.sdf does not exist in %s' % iris)

    text = model.read_text()
    backup = model.with_name(model.name + '.uav_semantic_search.bak')
    if not backup.exists():
        shutil.copy2(model, backup)
    text, had_old = remove_existing_block(text)
    end = text.rfind('</model>')
    if end < 0:
        raise RuntimeError('Could not find final </model> in %s' % model)
    model.write_text(text[:end] + BLOCK + '\n' + text[end:])
    print(('Updated' if had_old else 'Patched'), model)

    package_world = Path(__file__).resolve().parents[1] / 'worlds' / 'corridor_rooms.world'
    target_world = base / 'worlds' / 'corridor_rooms.world'
    target_world.parent.mkdir(parents=True, exist_ok=True)
    world_backup = target_world.with_name(target_world.name + '.uav_semantic_search.bak')
    if target_world.exists() and not world_backup.exists():
        shutil.copy2(target_world, world_backup)
    shutil.copy2(package_world, target_world)
    print('Installed', target_world)
    print('Restart all Gazebo/PX4 processes before launching the scenario.')


if __name__ == '__main__':
    main()
