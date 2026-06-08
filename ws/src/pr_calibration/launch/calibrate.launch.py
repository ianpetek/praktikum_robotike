"""Bring up the real overhead camera, AprilTag detection, and the calibration node.

    ros2 launch pr_calibration calibrate.launch.py video_device:=/dev/video0 \
        camera_info_url:=file:///path/to/overhead_camera.yaml

The camera MUST be intrinsically calibrated (pass a valid camera_info_url) for the
tag pose — and therefore the calibration — to be metrically correct. Use the
already-installed `camera_calibration` tool to produce that file if needed.

Pipeline: usb_cam (image_raw + camera_info) -> image_proc rectify (image_rect)
          -> apriltag_ros (TF camera -> tag) -> calibrate_camera (writes YAML).
"""
import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share = get_package_share_directory('pr_calibration')
    layout_file = os.path.join(share, 'config', 'layout.yaml')
    with open(layout_file) as f:
        cfg = yaml.safe_load(f)

    size_m = float(cfg['tag_size_mm']) / 1000.0
    camera_frame = cfg['frames']['camera']
    tag_frame = cfg['frames']['tag']

    video_device = LaunchConfiguration('video_device')
    camera_info_url = LaunchConfiguration('camera_info_url')

    usb_cam = Node(
        package='usb_cam', executable='usb_cam_node_exe', name='usb_cam',
        output='screen',
        parameters=[{
            'video_device': video_device,
            'frame_id': camera_frame,
            'camera_info_url': camera_info_url,
            'pixel_format': 'mjpeg2rgb',
        }],
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
        DeclareLaunchArgument('video_device', default_value='/dev/video0'),
        DeclareLaunchArgument(
            'camera_info_url', default_value='',
            description='file:// URL to the calibrated camera_info YAML (required for accuracy)'),
        usb_cam,
        rectify,
        apriltag,
        calibrate,
    ])
