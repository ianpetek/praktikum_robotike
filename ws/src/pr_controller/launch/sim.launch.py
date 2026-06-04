import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pr_description_share = get_package_share_directory('pr_description')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pr_description_share, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )

    jsb_spawner = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
                parameters=[{'use_sim_time': True}],
                output='screen',
            )
        ],
    )

    arm_spawner = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['arm_controller', '--controller-manager', '/controller_manager'],
                parameters=[{'use_sim_time': True}],
                output='screen',
            )
        ],
    )

    pen_mount_broadcaster = Node(
        package='pr_controller',
        executable='pen_mount_broadcaster.py',
        name='pen_mount_broadcaster',
        output='screen',
        parameters=[{'use_sim_time': True, 'use_sim': True}],
    )

    inverse_kinematics_control = Node(
        package='pr_controller',
        executable='inverse_kinematics_control.py',
        name='inverse_kinematics_control',
        output='screen',
        parameters=[{'use_sim_time': True, 'use_sim': True}],
    )

    pen_mount_spawner = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['pen_mount_controller', '--controller-manager', '/controller_manager'],
                parameters=[{'use_sim_time': True}],
                output='screen',
            )
        ],
    )

    return LaunchDescription([
        gazebo_launch,
        jsb_spawner,
        arm_spawner,
        pen_mount_spawner,
        pen_mount_broadcaster,
        inverse_kinematics_control,
    ])
