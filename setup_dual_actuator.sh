#!/bin/bash
# ============================================================
# ROS 2 Dual Actuator Setup Script
# Author: James Caddell
# ============================================================

# Stop on first error
set -e

echo "=== 1. Creating ROS 2 workspace directories ==="
mkdir -p ~/dual_actuator_ws/src
cd ~/dual_actuator_ws

echo "=== 2. Installing dependencies (rclpy, geometry_msgs, pygame) ==="
sudo apt update
sudo apt install -y python3-colcon-common-extensions python3-pip python3-pygame ros-${ROS_DISTRO}-geometry-msgs

echo "=== 3. Checking package presence ==="
if [ ! -d "src/dual_actuator_control" ]; then
    echo "ERROR: dual_actuator_control package not found in src/"
    echo "Please create it first, then rerun this script."
    exit 1
fi

echo "=== 4. Making Python files executable ==="
chmod +x ~/dual_actuator_ws/src/dual_actuator_control/dual_actuator_control/*.py || true

echo "=== 5. Building the workspace ==="
colcon build --symlink-install

echo "=== 6. Sourcing setup.bash ==="
source install/setup.bash

echo "=== 7. Verifying executables registered with ROS ==="
ros2 pkg executables dual_actuator_control || echo "No executables found — check setup.py entry_points."

echo "=== 8. Setup complete ==="
echo ""
echo "You can now run:"
echo "  ros2 run dual_actuator_control motor_x"
echo "  ros2 run dual_actuator_control motor_y"
echo "  ros2 run dual_actuator_control keyboard_controller"
echo "or:"
echo "  ros2 run dual_actuator_control joystick_controller"
echo ""
