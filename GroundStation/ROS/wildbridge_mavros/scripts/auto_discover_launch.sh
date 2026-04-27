#!/bin/bash
# Auto-discovery launch script for WildBridge MAVROS Bridge
# Discovers drone and launches with appropriate namespace

set -e

# Source ROS setup
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

# Python script to discover drone and print namespace
NAMESPACE=$(python3 -c "
import sys
sys.path.insert(0, '/ros2_ws/install/wildbridge_mavros/lib/python3.10/site-packages')
from wildbridge_mavros.dji_interface import discover_drone

ip, serial = discover_drone(timeout=5.0)
if ip:
    print(f'drone_{serial}' if serial != 'UNKNOWN' else 'drone_1')
    sys.exit(0)
else:
    print('drone_1')
    sys.exit(1)
")

DISCOVER_EXIT=$?

if [ $DISCOVER_EXIT -eq 0 ]; then
    echo "Discovered drone, using namespace: $NAMESPACE"
else
    echo "Discovery failed, using default namespace: $NAMESPACE"
fi

# Launch the node with discovered namespace
exec ros2 run wildbridge_mavros mavros_bridge --ros-args -r __ns:=/$NAMESPACE
