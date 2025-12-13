#!/usr/bin/env bash
echo "=========================================="
echo "Launching Foxglove Bridge"
echo "Connect at: ws://localhost:8765"
echo "=========================================="
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
