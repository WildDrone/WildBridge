#!/usr/bin/env python3
"""
Auto-Launch Script for WildBridge MAVROS Bridge

Discovers the drone to get its serial number, then launches the ROS node
with the appropriate namespace.
"""

import sys
import os
from dji_interface import discover_drone

def main():
    print("WildBridge Auto-Launch: Discovering drone...")
    
    # Discover drone to get IP and serial
    discovered_ip, drone_serial = discover_drone(timeout=5.0)
    
    if not discovered_ip:
        print("ERROR: Failed to discover drone!")
        return 1
    
    print(f"Discovered drone at {discovered_ip} with serial: {drone_serial}")
    
    # Determine namespace based on serial
    if drone_serial != "UNKNOWN":
        namespace = f"drone_{drone_serial}"
    else:
        namespace = "drone_1"
        print(f"WARNING: Could not determine drone serial, using default namespace: {namespace}")
    
    print(f"Launching node with namespace: {namespace}")
    
    # Now launch the actual ROS node
    import rclpy
    from mavros_bridge import WildBridgeMavrosNode
    
    rclpy.init(args=sys.argv)
    
    # Create node with namespace
    node = WildBridgeMavrosNode()
    node._parameters['drone_ip'] = rclpy.parameter.Parameter(
        'drone_ip',
        rclpy.Parameter.Type.STRING,
        discovered_ip
    )
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    
    return 0

if __name__ == '__main__':
    sys.exit(main())
