"""
Multi-drone WildBridge MAVROS Bridge Launch File

Launch multiple MAVROS-compatible interfaces for controlling a swarm of DJI drones.

Usage:
    ros2 launch wildbridge_mavros multi_drone.launch.py
    
Configure drones in the config/drones.yaml file.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml
import os
from ament_index_python.packages import get_package_share_directory


def launch_drones(context, *args, **kwargs):
    """Dynamically launch nodes for each drone in config."""
    
    config_file = LaunchConfiguration('config_file').perform(context)
    
    # Load drone configuration
    with open(config_file, 'r') as f:
        config = yaml.safe_load(f)
    
    nodes = []
    
    for drone in config.get('drones', []):
        drone_id = drone.get('id', 1)
        drone_ip = drone.get('ip', '192.168.1.100')
        namespace = drone.get('namespace', f'drone_{drone_id}')
        
        node = Node(
            package='wildbridge_mavros',
            executable='mavros_bridge',
            name='wildbridge_mavros',
            namespace=namespace,
            parameters=[{
                'drone_ip': drone_ip,
                'system_id': drone_id,
                'component_id': 1,
                'telemetry_rate': 20.0,
            }],
            output='screen',
            emulate_tty=True,
        )
        nodes.append(node)
    
    return nodes


def generate_launch_description():
    pkg_share = get_package_share_directory('wildbridge_mavros')
    default_config = os.path.join(pkg_share, 'config', 'drones.yaml')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config,
            description='Path to drone configuration YAML file'
        ),
        
        OpaqueFunction(function=launch_drones),
    ])
