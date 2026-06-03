import os
from launch import LaunchDescription
from launch.actions import TimerAction
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pr_description_share = get_package_share_directory('pr_description')
    pr_controller_share = get_package_share_directory('pr_controller')

    urdf_file = os.path.join(pr_description_share, 'urdf', 'pr_robot.xacro')
    controllers_yaml = os.path.join(pr_description_share, 'config', 'controller.yaml')
    rviz_config = os.path.join(pr_controller_share, 'config', 'hardware.rviz')

    robot_description = ParameterValue(
        Command(['xacro ', urdf_file, ' use_sim:=false']),
        value_type=str,
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': False}],
    )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_description}, controllers_yaml],
        output='screen',
    )

    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    arm_spawner = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['arm_controller', '--controller-manager', '/controller_manager'],
                output='screen',
            )
        ],
    )

    pen_mount_broadcaster = Node(
        package='pr_controller',
        executable='pen_mount_broadcaster.py',
        name='pen_mount_broadcaster',
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='log',
        parameters=[{'use_sim_time': False}],
    )

    return LaunchDescription([
        robot_state_publisher,
        controller_manager,
        jsb_spawner,
        arm_spawner,
        pen_mount_broadcaster,
        rviz,
    ])
