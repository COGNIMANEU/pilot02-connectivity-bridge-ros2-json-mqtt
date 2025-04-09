# Use the ROS 2 Humble base image
FROM ros:humble

# Set up the ROS 2 workspace
WORKDIR /ros_ws

# Install system dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# Install necessary Python packages
RUN pip3 install --no-cache-dir paho-mqtt pyyaml

# Create and copy the code into the workspace
COPY . /ros_ws/src

# Ensure all Python scripts are executable
RUN chmod +x /ros_ws/src/json_mqtt_bridge/json_mqtt_bridge/*.py

# Set the Python path
ENV PYTHONPATH=/ros_ws/src:/ros_ws/src/json_mqtt_bridge:$PYTHONPATH

# Install the package in editable mode without dependencies
RUN cd /ros_ws/src/json_mqtt_bridge && \
    pip install --no-deps -e .

# Copy the entrypoint script
COPY entrypoint.sh /ros_ws/entrypoint.sh

# Grant execution permissions to the entrypoint script
RUN chmod +x /ros_ws/entrypoint.sh

# Build the workspace
WORKDIR /ros_ws
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

# Ensure the ROS2 environment is loaded
RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc
RUN echo 'source /ros_ws/install/setup.bash' >> /root/.bashrc

# Set the entrypoint script
ENTRYPOINT ["/ros_ws/entrypoint.sh"]
