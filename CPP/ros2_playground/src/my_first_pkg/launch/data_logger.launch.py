from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # nmea_navsat_driver publishes to /fix
    nmea_driver = Node(
        package='nmea_navsat_driver',
        executable='nmea_serial_driver',
        name='nmea_serial_driver',
        parameters=[{
            'port': '/dev/ttyUSB0',
            'baud_rate': 9600,
        }],
        output='screen',
    )

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
        ],
        output='screen',
    )

    return LaunchDescription([nmea_driver, container])
