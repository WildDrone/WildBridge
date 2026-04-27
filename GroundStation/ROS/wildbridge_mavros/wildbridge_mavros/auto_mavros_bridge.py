#!/usr/bin/env python3
"""
Auto-discovery wrapper for MAVROS Bridge.
Discovers drone, queries configuration, then launches node with proper namespace.
"""

import sys
import rclpy
from rclpy.executors import SingleThreadedExecutor
from wildbridge_mavros.dji_interface import discover_all_drones, get_config
from wildbridge_mavros.mavros_bridge import WildBridgeMavrosNode


def main(args=None):
    """Main entry point with auto-discovery and dynamic namespace."""
    
    print("WildBridge MAVROS Bridge - Auto-Discovery Mode")
    print("=" * 60)
    
    # Discover drone
    print("Discovering drones...")
    drones = discover_all_drones(timeout=5.0, verbose=True)
    
    if not drones:
        print("ERROR: Failed to discover any drones!")
        return 1
    
    if len(drones) > 1:
        print(f"⚠ Found {len(drones)} drones! Using the first one.")
        for i, (ip, name) in enumerate(drones):
            print(f"  {i+1}. {name} at {ip}")
    
    discovered_ip, drone_name = drones[0]
    
    print(f"✓ Discovered drone at {discovered_ip}")
    
    # Query config endpoint for more reliable name
    config = get_config(discovered_ip)
    if config and "droneName" in config:
        drone_name = config["droneName"]
        print(f"✓ Retrieved drone name from config: {drone_name}")
    else:
        print(f"⚠ Could not query config, using discovery name: {drone_name}")
    
    # Determine namespace (use drone name directly)
    if drone_name and drone_name != "UNKNOWN":
        namespace = drone_name
    else:
        namespace = "drone_1"
        print(f"⚠ Using default namespace: {namespace}")
    
    print(f"✓ Launching with namespace: /{namespace}")
    print("=" * 60)
    
    # Build args for ROS with namespace and discovered IP
    node_args = [
        '--ros-args',
        '-r', f'__ns:=/{namespace}',
        '-p', f'drone_ip:={discovered_ip}'
    ]
    
    # Initialize ROS with namespace args
    rclpy.init(args=node_args)
    
    try:
        # Create executor
        executor = SingleThreadedExecutor()
        
        # Create node (it will pick up namespace and params from context)
        node = WildBridgeMavrosNode()
        
        # Add node to executor
        executor.add_node(node)
        
        # Spin
        try:
            executor.spin()
        except KeyboardInterrupt:
            pass
        finally:
            executor.shutdown()
            node.destroy_node()
            
    finally:
        rclpy.shutdown()
    
    return 0


if __name__ == '__main__':
    sys.exit(main())
