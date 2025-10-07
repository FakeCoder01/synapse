# Start from a base ROS 2 image (e.g., Humble, Iron, depending on your project)
# You can find official images on Docker Hub: https://hub.docker.com/_/ros
FROM osrf/ros:humble-desktop

# Set the working directory inside the container
WORKDIR /ros_ws

# Install any necessary dependencies that aren't in the base image
# Example:
RUN apt-get update && apt-get install -y \
    ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox \
    ros-humble-teleop-twist-keyboard ros-humble-xacro ros-humble-rqt* \
    python3-colcon-common-extensions python3-rosdep python3-vcstool git


RUN rosdep init || true
RUN rosdep update

RUN apt-get install -y ros-humble-realsense2-camera ros-humble-realsense2-description ros-humble-rplidar-ros ros-humble-urg-node

RUN apt-get install -y ros-humble-image-common ros-humble-image-transport-plugins

RUN apt-get install -y python3-venv python3-pip

RUN apt-get install -y build-essential cmake libopenblas-dev liblapack-dev libx11-dev libgtk-3-dev

# Copy your local ROS 2 project source code into the container's workspace
# The 'src' folder contains your packages
COPY src /ros_ws/src

# Build the ROS 2 workspace
# RUN . /opt/ros/humble/setup.bash && \
#     rosdep update && \
#     rosdep install --from-paths src --ignore-src -y && \
#     colcon build

# Optional: Set the entrypoint to a script or the ROS 2 setup file
# This is a common pattern to automatically source the workspace
# ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source install/setup.bash && exec \"$@\""]
CMD ["/bin/bash"]
