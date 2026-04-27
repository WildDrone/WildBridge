import sys
import os
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    # Try to import dji_interface for discovery
    try:
        from wildbridge_mavros.dji_interface import discover_all_drones
    except ImportError:
        # Try relative import if we are in the source tree
        current_dir = os.path.dirname(__file__)
        # Go up to wildbridge_mavros package root
        package_dir = os.path.abspath(os.path.join(current_dir, '../'))
        if package_dir not in sys.path:
            sys.path.append(package_dir)
        
        try:
            from wildbridge_mavros.dji_interface import discover_all_drones
        except ImportError:
            print("Could not import wildbridge_mavros.dji_interface.")
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
            package='wildbridge_mavros',
            executable='mavros_bridge',
            name=f'wildbridge_mavros_{ns}',
            namespace=ns,
            output='screen',
            parameters=[{
                'drone_ip': ip,
                'namespace_mode': 'manual', # Since we set namespace in launch
                'system_id': i + 1,
                'component_id': 1
            }]
        )
        actions.append(node)
        
    return actions

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
