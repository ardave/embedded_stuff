from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    container = ComposableNodeContainer(
        name='data_logger_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='my_first_pkg',
                plugin='DataLogger',
                name='data_logger',
            ),
            ComposableNode(
                package='my_first_pkg',
                plugin='CameraNode',
                name='camera_node',
            ),
            ComposableNode(
                package='my_first_pkg',
                plugin='GPSNode',
                name='gps_node',
            ),
        ],
        output='screen',
    )

    return LaunchDescription([container])
