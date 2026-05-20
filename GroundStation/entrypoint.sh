#!/bin/bash
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

echo "========================================================"
echo "   WildBridge Ground Station (ROS 2 Humble)"
echo "========================================================"
echo "The container is starting."
echo ""
echo "For video diagnostics, run the top-level compose.video-test.yaml stack."
echo "It starts MediaMTX plus the WildBridge video dashboard at http://localhost:8090."
echo ""
echo "Starting ROS node with auto-discovery..."
echo "========================================================"

# Launch with auto-discovery wrapper
exec ros2 run wildbridge_mavros auto_mavros_bridge

