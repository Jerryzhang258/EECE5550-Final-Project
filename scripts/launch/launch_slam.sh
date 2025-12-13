#!/usr/bin/env bash
echo "=========================================="
echo "Launching SLAM Toolbox"
echo "Creating real-time map..."
echo "=========================================="
ros2 launch turtlebot4_navigation slam.launch.py
