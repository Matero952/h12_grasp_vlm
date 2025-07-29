#!/bin/bash
set -e
cd /ros2_ws

# Clean previous builds
rm -rf build install log

# Source ROS 2 base setup
source /opt/ros/humble/setup.bash

# Install dependencies
rosdep install -i --from-path src --rosdistro humble -y

# Build workspace
colcon build --symlink-install

# Source overlay workspace
source install/setup.bash
