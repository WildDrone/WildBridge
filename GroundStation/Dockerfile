FROM ros:humble

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONUNBUFFERED=1

# Create the dds logs directory with proper permissions
RUN mkdir -p /.dds/log && chmod 777 /.dds/log

# Create workspace directories
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws

# Install system dependencies
RUN apt-get update && apt-get install -y \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-cv-bridge \
    ros-humble-vision-opencv \
    ros-humble-image-transport \
    python3-pip \
    libgl1 \
    libglib2.0-0 \
    iputils-ping \
    net-tools \
    && rm -rf /var/lib/apt/lists/*

# Copy requirements
COPY ROS/requirements.txt /tmp/ros_requirements.txt

# Install Python dependencies
RUN pip3 install --no-cache-dir -r /tmp/ros_requirements.txt

# Copy ROS packages
COPY ROS /ros2_ws/src/

# Copy helper scripts
COPY Python /app/Python

# Build the ROS workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"

# Source the workspace in bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

# Set working directory
WORKDIR /app

# Copy entrypoint
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Default command
CMD ["/entrypoint.sh"]
