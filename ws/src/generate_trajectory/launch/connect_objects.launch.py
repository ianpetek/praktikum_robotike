"""Persistent bringup: overhead camera -> YOLO detection -> static calibration ->
on-demand connect service that drives the arm.

Runs (and leaves running):
  * pr_calibration camera.launch.py    (usb_cam + calibrated /camera_info)
  * pr_calibration static_tf.launch.py (fixed base_link -> camera transform)
  * yolo_bringup yolov8.launch.py       (publishes /yolo/detections continuously)
  * connect_objects_server              (the ConnectObjects service)

Bring this up once, then connect objects on demand (repeatable) with:

    ros2 service call /connect_objects pr_interfaces/srv/ConnectObjects \
        "{start: apple, goal: cup, obstacle: bottle}"

The arm runs separately (pr_controller sim.launch.py / hardware.launch.py); the
service publishes the path to /inverse_kinematics_control/draw_path.

    ros2 launch generate_trajectory connect_objects.launch.py model:=yolov8m.pt
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pr_calib = get_package_share_directory('pr_calibration')
    yolo_bringup = get_package_share_directory('yolo_bringup')

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pr_calib, 'launch', 'camera.launch.py')))

    static_tf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pr_calib, 'launch', 'static_tf.launch.py')))

    yolo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(yolo_bringup, 'launch', 'yolov8.launch.py')),
        # yolov8.launch.py only forwards these args (no use_tracking/use_3d here;
        # use_3d defaults False, which is correct for our depth-less mono camera).
        launch_arguments={
            'model': LaunchConfiguration('model'),
            'input_image_topic': '/image_raw',
            'device': LaunchConfiguration('device'),
            'threshold': '0.5',
        }.items(),
    )

    connect_server = Node(
        package='generate_trajectory', executable='generate_path_node',
        name='connect_objects_server', output='screen',
        parameters=[{'table_z': LaunchConfiguration('table_z')}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('model', default_value='yolov8m.pt'),
        DeclareLaunchArgument('device', default_value='cuda:0'),
        DeclareLaunchArgument('table_z', default_value='0.02'),
        camera,
        static_tf,
        yolo,
        connect_server,
    ])
