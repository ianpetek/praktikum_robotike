"""Bring up the overhead camera (with calibrated intrinsics), AprilTag detection,
and the extrinsic calibration node.

    ros2 launch pr_calibration calibrate.launch.py

Intrinsics come from config/camera_calibration_params.yaml (run
intrinsic_calibration.launch.py first if not yet calibrated). The node writes the
camera->base_link extrinsic to config/camera_extrinsics.yaml.

Pipeline: usb_cam (image_raw + camera_info) -> image_proc rectify (image_rect)
          -> apriltag_ros (TF camera -> tag) -> calibrate_camera (writes YAML).
"""
import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share = get_package_share_directory('pr_calibration')
    layout_file = os.path.join(share, 'config', 'layout.yaml')
    camera_params = os.path.join(share, 'config', 'camera_params.yaml')
    with open(layout_file) as f:
        cfg = yaml.safe_load(f)

    size_m = float(cfg['tag_size_mm']) / 1000.0
    tag_frame = cfg['frames']['tag']

    usb_cam = Node(
        package='usb_cam', executable='usb_cam_node_exe', name='camera',
        output='screen', parameters=[camera_params],
    )

    rectify = Node(
        package='image_proc', executable='rectify_node', name='rectify',
        output='screen',
        remappings=[
            ('image', '/image_raw'),
            ('camera_info', '/camera_info'),
            ('image_rect', '/image_rect'),
        ],
    )

    apriltag = Node(
        package='apriltag_ros', executable='apriltag_node', name='apriltag',
        output='screen',
        remappings=[
            ('image_rect', '/image_rect'),
            ('camera_info', '/camera_info'),
        ],
        parameters=[{
            'family': cfg['tag_family'],
            'size': size_m,
            'pose_estimation_method': 'pnp',
            'detector.threads': 2,
            'detector.decimate': 1.0,
            'detector.refine': True,
            'tag.ids': [int(cfg['tag_id'])],
            'tag.frames': [tag_frame],
            'tag.sizes': [size_m],
        }],
    )

    calibrate = Node(
        package='pr_calibration', executable='calibrate_camera.py',
        name='calibrate_camera', output='screen',
        parameters=[{'layout_file': layout_file}],
    )

    return LaunchDescription([
        usb_cam,
        rectify,
        apriltag,
        calibrate,
    ])
