# Use the official ROS2 Humble image
FROM ros:humble

# Set up the workspace
RUN mkdir -p /root/igvc_2025/bot_ws/src
WORKDIR /root/igvc__2025/bot_ws/src

# Install dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    ros-humble-nav2-bringup \
    ros-humble-perception-pcl \
    ros-humble-vision-opencv \
    && rm -rf /var/lib/apt/lists/*

# Copy the repository into the container
COPY . /root/igvc_2025/

# Install Python dependencies
RUN pip3 install -r /root/igvc_2025/requirements.txt

# Build the workspace
WORKDIR /root/igvc_2025/bot_ws
RUN . /opt/ros/humble/setup.sh && colcon build

# Set up the entrypoint
COPY ./scripts/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
