"""Intrinsic camera calibration with a checkerboard.

Runs the overhead camera and the OpenCV `cameracalibrator` GUI together:

    ros2 launch pr_calibration intrinsic_calibration.launch.py

Use the checkerboard sheet in pr_calibration/files (7x9 inner corners, 18.6 mm
squares). Move the board until the X/Y/Size/Skew bars turn green, click
Calibrate, then Save (writes /tmp/calibrationdata.tar.gz) and Commit. Extract the
tarball and copy the values from ost.yaml into
config/camera_calibration_params.yaml, then rebuild pr_calibration.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    camera_params = os.path.join(
        get_package_share_directory('pr_calibration'), 'config', 'camera_params.yaml')

    camera = Node(
        package='usb_cam', executable='usb_cam_node_exe', name='camera',
        output='screen', parameters=[camera_params],
    )

    calibrator = Node(
        package='camera_calibration', executable='cameracalibrator',
        name='camera_calib', output='screen',
        arguments=['--size', LaunchConfiguration('size'),
                   '--square', LaunchConfiguration('square')],
        remappings=[('image', '/image_raw')],
    )

    return LaunchDescription([
        DeclareLaunchArgument('size', default_value='7x9',
                              description='Checkerboard inner-corner count, e.g. 7x9'),
        DeclareLaunchArgument('square', default_value='0.0186',
                              description='Checkerboard square size in metres'),
        camera,
        calibrator,
    ])
