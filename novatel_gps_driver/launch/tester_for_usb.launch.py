"""Launch an example driver that communicates using TCP"""

from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
    container = launch_ros.actions.ComposableNodeContainer(
        node_name='novatel_gps_container',
        node_namespace='',
        package='rclcpp_components',
        node_executable='component_container',
        composable_node_descriptions=[
            launch_ros.descriptions.ComposableNode(
                package='novatel_gps_driver',
                node_plugin='novatel_gps_driver::NovatelGpsNode',
                node_name='novatel_gps',
                parameters=[{
                    'connection_type': 'serial',
                    'device': '/dev/ttyUSB0',
                    'verbose': True,
                    'publish_novatel_positions': True,
                    'frame_id': '/gps'
                }]
            )
        ],
        output='screen'
    )

    return LaunchDescription([container])
