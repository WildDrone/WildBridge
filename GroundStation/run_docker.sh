#!/bin/bash

# Get the directory where the script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# Build the Docker image
echo "Building WildBridge Ground Station Docker image..."
docker build -t wildbridge-gs "$SCRIPT_DIR"

# Allow X11 connections (be careful with security on public networks)
xhost +local:docker

# Run the container
# --net=host: Required for UDP broadcast discovery and WebRTC
# -v /tmp/.X11-unix:/tmp/.X11-unix: Required for GUI display
# -e DISPLAY=$DISPLAY: Required for GUI display
echo "Running WildBridge Ground Station..."
docker run -it --rm \
    --net=host \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -e DISPLAY=$DISPLAY \
    wildbridge-gs
