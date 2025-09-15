from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    develeopment_orbit_controller = Node(
        package='development_orbit_controller_api',
        executable='develepment_orbit_controller',
    )

    blockly_server_node = Node(
        package='development_orbit_controller_api',
        executable='blockly_server',
    )

    orbit_controller = Node(
        package='orbit_controller',
        executable='orbit_controller',
    )

    return LaunchDescription([
        develeopment_orbit_controller,
        blockly_server_node,
        orbit_controller
    ])