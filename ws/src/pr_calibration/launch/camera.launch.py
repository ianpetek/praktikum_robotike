"""Run the overhead camera with the calibrated intrinsics.

Publishes /image_raw and a calibrated /camera_info (from
config/camera_calibration_params.yaml) for the rest of the pipeline (AprilTag
extrinsic calibration, YOLO, path generation).

    ros2 launch pr_calibration camera.launch.py
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    camera_params = os.path.join(
        get_package_share_directory('pr_calibration'), 'config', 'camera_params.yaml')

    return LaunchDescription([
        Node(
            package='usb_cam', executable='usb_cam_node_exe', name='camera',
            output='screen', parameters=[camera_params],
        ),
    ])
