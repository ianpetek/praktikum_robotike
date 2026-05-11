import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(
        package='pr_description'
    ).find('pr_description')
    default_model_path = os.path.join(pkg_share, 'urdf/pr_robot.xacro')
    default_rviz_config_path = os.path.join(pkg_share, 'config/display.rviz')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'use_sim_time': False,
            'robot_description': ParameterValue(
                Command(['xacro ', LaunchConfiguration('model')]),
                value_type=str,
            ),
        }],
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        parameters=[{'use_sim_time': False}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'model', default_value=default_model_path,
            description='Absolute path to robot URDF/xacro file',
        ),
        DeclareLaunchArgument(
            'rvizconfig', default_value=default_rviz_config_path,
            description='Absolute path to RViz config file',
        ),
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
    ])
