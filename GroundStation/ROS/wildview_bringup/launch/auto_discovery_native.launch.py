import sys
import os
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    # Try to import dji_interface for discovery
    try:
        from dji_controller.submodules.dji_interface import discover_all_drones
    except ImportError:
        # Try relative import if we are in the source tree
        current_dir = os.path.dirname(__file__)
        # Go up to ROS directory
        ros_dir = os.path.abspath(os.path.join(current_dir, '../../'))
        dji_controller_path = os.path.join(ros_dir, 'dji_controller')
        if dji_controller_path not in sys.path:
            sys.path.append(dji_controller_path)
        
        try:
            from dji_controller.submodules.dji_interface import discover_all_drones
        except ImportError:
            print("Could not import dji_controller.submodules.dji_interface.")
            return []

    print("Discovering drones...")
    drones = discover_all_drones(timeout=5.0)
    
    actions = []
    
    if not drones:
        print("No drones found via auto-discovery.")
        return []
        
    for i, (ip, name) in enumerate(drones):
        print(f"Found drone: {name} at {ip}")
        
        # Determine namespace
        clean_name = "".join(c if c.isalnum() or c == '_' else '_' for c in name)
        if not clean_name or clean_name == "UNKNOWN":
            ns = f"drone_{i+1}"
        else:
            ns = clean_name
        
        node = Node(
            package='dji_controller',
            executable='dji_node',
            name=f'dji_node_{ns}',
            namespace=ns,
            output='screen',
            parameters=[{
                'ip_rc': ip
            }]
        )
        actions.append(node)
        
    return actions

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
