from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # set arguments so they can be changed via command line
    camera_info_arg = DeclareLaunchArgument(
        'camera_info_topic',
        default_value='camera/camera_info',
        description='Topic for CameraInfo messages'
    )
    
    yolo_detections_arg = DeclareLaunchArgument(
        'yolo_topic',
        default_value='/yolo/detections',
        description='Topic for YOLO detections'
    )

    output_topic_arg = DeclareLaunchArgument(
        'output_topic',
        default_value='drawn_detections',
        description='Topic for the resulting image'
    )

    yolo_node = Node(
        package='yolo_processing', 
        executable='yolo_processor_node',
        name='yolo_processor',
        output='screen',
        parameters=[{
            'camera_info_topic': LaunchConfiguration('camera_info_topic'),
            'yolo_detections_topic': LaunchConfiguration('yolo_topic'),
            'output_topic': LaunchConfiguration('output_topic'),
        }]
    )

    return LaunchDescription([
        camera_info_arg,
        yolo_detections_arg,
        output_topic_arg,
        yolo_node
    ])