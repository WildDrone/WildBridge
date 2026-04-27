"""
WildBridge MAVROS Bridge Launch File

Launch the MAVROS-compatible interface for WildBridge DJI drone control.

Usage:
    ros2 launch wildbridge_mavros mavros_bridge.launch.py drone_ip:=192.168.1.100
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument(
            'drone_ip',
            default_value='192.168.1.100',
            description='IP address of the DJI remote controller running WildBridge'
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Namespace for all topics (useful for multi-drone)'
        ),
        DeclareLaunchArgument(
            'system_id',
            default_value='1',
            description='MAVLink system ID'
        ),
        DeclareLaunchArgument(
            'component_id',
            default_value='1',
            description='MAVLink component ID'
        ),
        DeclareLaunchArgument(
            'telemetry_rate',
            default_value='20.0',
            description='Telemetry publishing rate in Hz'
        ),
        
        # WildBridge MAVROS Bridge node
        Node(
            package='wildbridge_mavros',
            executable='mavros_bridge',
            name='wildbridge_mavros',
            namespace=LaunchConfiguration('namespace'),
            parameters=[{
                'drone_ip': LaunchConfiguration('drone_ip'),
                'system_id': LaunchConfiguration('system_id'),
                'component_id': LaunchConfiguration('component_id'),
                'telemetry_rate': LaunchConfiguration('telemetry_rate'),
            }],
            output='screen',
            emulate_tty=True,
        ),
    ])
