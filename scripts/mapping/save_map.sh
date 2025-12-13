#!/usr/bin/env bash

MAP_DIR="$HOME/turtlebot4_maps"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)

mkdir -p "$MAP_DIR"

echo "=========================================="
echo "Save SLAM Map"
echo "=========================================="
echo ""
read -p "Enter map name (default: map_$TIMESTAMP): " MAP_NAME

if [ -z "$MAP_NAME" ]; then
    MAP_NAME="map_$TIMESTAMP"
fi

MAP_PATH="$MAP_DIR/$MAP_NAME"

echo ""
echo "Saving map to: $MAP_PATH"
echo ""

ros2 run nav2_map_server map_saver_cli -f "$MAP_PATH"

if [ $? -eq 0 ]; then
    echo ""
    echo "=========================================="
    echo "✓ Map saved successfully!"
    echo "=========================================="
    echo "Files created:"
    echo "  - $MAP_PATH.pgm (image)"
    echo "  - $MAP_PATH.yaml (metadata)"
    echo ""
    echo "View map with: eog $MAP_PATH.pgm"
    echo "View metadata: cat $MAP_PATH.yaml"
    echo ""
else
    echo ""
    echo "✗ Map saving failed!"
    echo ""
    exit 1
fi
