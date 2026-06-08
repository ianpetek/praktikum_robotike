"""Publish the calibrated camera pose as a static transform.

Reads config/camera_extrinsics.yaml (written by calibrate_camera) and starts a
static_transform_publisher for parent_frame -> child_frame (base_link -> camera).
This is what puts the fixed camera into the robot TF tree so downstream nodes can
convert camera detections into base_link without any runtime marker.

    ros2 launch pr_calibration static_tf.launch.py
"""
import os
import yaml
from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    extr = os.path.join(
        get_package_share_directory('pr_calibration'), 'config', 'camera_extrinsics.yaml')

    if not os.path.exists(extr):
        return LaunchDescription([LogInfo(msg=(
            f'{extr} not found -- run `ros2 launch pr_calibration calibrate.launch.py` '
            'to produce the camera->base_link extrinsic first.'))])

    with open(extr) as f:
        c = yaml.safe_load(f)
    t, r = c['translation'], c['rotation']

    return LaunchDescription([
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='camera_extrinsic_tf', output='screen',
            arguments=[
                '--x', str(t['x']), '--y', str(t['y']), '--z', str(t['z']),
                '--qx', str(r['x']), '--qy', str(r['y']),
                '--qz', str(r['z']), '--qw', str(r['w']),
                '--frame-id', c['parent_frame'],
                '--child-frame-id', c['child_frame'],
            ],
        ),
    ])
