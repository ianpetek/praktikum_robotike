from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            'yaw_joint_position_controller',
            'bicep_joint_position_controller',
            'forearm_joint_position_controller',
            'pen_mount_joint_position_controller',
        ],
        output='screen',
    )

    return LaunchDescription([controller_spawner])
