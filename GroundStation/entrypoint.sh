#!/bin/bash
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

echo "========================================================"
echo "   WildBridge Ground Station (ROS 2 Humble)"
echo "========================================================"
echo "The container is starting."
echo ""
echo "To view the WebRTC video stream:"
echo "1. Open a new terminal in this container: docker exec -it <container_id> bash"
echo "2. Run: run_webrtc"
echo ""
echo "Starting ROS node with auto-discovery..."
echo "========================================================"

# Launch with auto-discovery wrapper
exec ros2 run wildbridge_mavros auto_mavros_bridge

