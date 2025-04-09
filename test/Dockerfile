# Use the ROS 2 Humble base image
FROM ros:humble

# Install necessary dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    cmake \
    git \
    ros-humble-rosidl-default-generators \
    ros-humble-std-msgs \
    ros-humble-geometry-msgs \
    ros-humble-builtin-interfaces \
    && rm -rf /var/lib/apt/lists/*

# Install required Python packages
RUN pip3 install --no-cache-dir paho-mqtt pyyaml

# Set up the ROS 2 workspace
WORKDIR /ros_ws

# Create a src directory and copy the entire workspace content into it
RUN mkdir -p /ros_ws/src
COPY . /ros_ws/src

# Ensure all Python scripts are executable
RUN chmod +x /ros_ws/src/*.py

# Build the workspace from the correct directory
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

# Ensure the ROS 2 environment is loaded
RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc
RUN echo 'source /ros_ws/install/setup.bash' >> /root/.bashrc
