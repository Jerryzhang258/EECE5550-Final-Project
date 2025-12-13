# Quick Reference Guide

## Launch Commands
```bash
# Complete system
./scripts/launch/start_all.sh

# Individual components
./scripts/launch/launch_foxglove.sh  # Visualization
./scripts/launch/launch_slam.sh      # Mapping
./scripts/launch/launch_teleop.sh    # Control
```

## Keyboard Controls

**Movement:**
- `i` - Forward
- `,` - Backward  
- `j` - Left
- `l` - Right
- `k` - STOP

**Speed:**
- `q/z` - All speeds ±10%
- `w/x` - Linear only
- `e/c` - Angular only

**Recommended:** Press `x` several times to slow to 0.2 m/s

## Foxglove Setup

1. Open: https://app.foxglove.dev
2. Connect: ws://localhost:8765
3. Add 3D panel
4. Set Fixed frame: "map" (CRITICAL!)
5. Subscribe to topics:
   - ✓ /map (nav_msgs/OccupancyGrid)
   - ✓ /scan (sensor_msgs/LaserScan)
   - ✓ /tf (tf2_msgs/TFMessage)

## Map Display Configuration

Click ⚙️ next to /map:
- Color mode: map
- Min color: #ffffff (white)
- Max color: #000000 (black)
- Unknown color: #808080 (gray)
- Alpha: 0.8

## LiDAR Display Configuration

Click ⚙️ next to /scan:
- Point size: 5
- Color mode: colormap
- Colormap: turbo
- Color field: intensity

## Mapping Best Practices

1. **Start slow** (0.2-0.3 m/s) - Press 'x' multiple times
2. **Outline perimeter** - Drive along walls first
3. **Fill interior** - Use zigzag or spiral pattern
4. **Loop closure** - Return to start via different path
5. **Smooth motion** - Avoid sudden turns and stops

## Coverage Strategy
```
Recommended Pattern:
┌─────────────────────────┐
│  1→→→→→→→→→→→→→→→→2    │
│  ↑                 ↓    │
│  ↑    ╱╲  ╱╲  ╱╲  ↓    │  1. Outline perimeter
│  ↑   ╱  ╲╱  ╲╱  ╲ ↓    │  2. Zigzag interior
│  ↑  ╱            ╲↓    │  3. Return to start
│  8←←←←←←←←←←←←←←←←3    │
└─────────────────────────┘
```

## Save Map
```bash
./scripts/mapping/save_map.sh
```

Maps saved to: `~/turtlebot4_maps/`
- `.pgm` - Map image file
- `.yaml` - Map metadata

## System Verification
```bash
./scripts/utils/verify_system.sh
```

## Manual System Checks
```bash
# Check active nodes
ros2 node list | grep slam

# Check map publishing rate
ros2 topic hz /map

# Check LiDAR data
ros2 topic echo /scan --once

# Check transforms
ros2 run tf2_ros tf2_echo map base_link

# Check all topics
ros2 topic list
```

## Troubleshooting

### No map visible in Foxglove?
- **Check Fixed frame** = "map" (most common!)
- Verify /map topic is checked in left panel
- Confirm SLAM is running: `ros2 node list | grep slam`

### No LiDAR points visible?
- Increase point size to 5-10
- Check /scan topic is checked
- Verify LiDAR: `ros2 topic echo /scan --once`

### Robot not moving?
- Click on teleop terminal to give it focus
- Test manually: `ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"`

### Blurry or poor quality map?
- Slow down: Press 'x' multiple times (target: 0.2 m/s)
- Return to known areas for loop closure
- Clean LiDAR lens

### Map drift?
- Return to previously mapped areas
- Ensure good loop closure detection
- Check that /tf is publishing: `ros2 topic hz /tf`

### Foxglove connection failed?
- Check bridge is running: `ros2 node list | grep foxglove`
- Verify port: `netstat -tulpn | grep 8765`
- Restart bridge: `./scripts/launch/launch_foxglove.sh`

## Performance Expectations

- **Mapping time**: 8-10 minutes for typical lab environment
- **Wall consistency**: 95%+ with proper technique
- **Loop closure**: 4-5 successful closures per session
- **Map resolution**: 0.05 m/cell
- **LiDAR rate**: ~5.5 Hz
- **Map update rate**: ~0.2 Hz

## Quality Indicators

**Good Map:**
- ✓ Sharp, continuous wall boundaries
- ✓ LiDAR points align with map
- ✓ No ghosting or duplicate features
- ✓ Clear free/occupied distinction

**Needs Improvement:**
- ✗ Blurry walls → Slow down
- ✗ Misaligned sections → Return to known areas
- ✗ Large blank areas → Need more exploration
- ✗ Multiple wall copies → Check for sensor issues

## Common ROS2 Commands
```bash
# List all nodes
ros2 node list

# List all topics
ros2 topic list

# Get topic info
ros2 topic info /map

# Echo topic data
ros2 topic echo /map

# Check message type
ros2 topic type /map

# Get topic frequency
ros2 topic hz /scan

# View TF tree
ros2 run tf2_tools view_frames

# Check transforms
ros2 run tf2_ros tf2_echo [source] [target]
```

## File Locations

- **Scripts**: `~/EECE5550-Final-Project/scripts/`
- **Maps**: `~/turtlebot4_maps/`
- **Config**: `~/EECE5550-Final-Project/config/`
- **Docs**: `~/EECE5550-Final-Project/docs/`

## Support

- **GitHub**: https://github.com/Jerryzhang258/EECE5550-Final-Project
- **Issues**: https://github.com/Jerryzhang258/EECE5550-Final-Project/issues
- **Documentation**: See `docs/` directory

---

**Pro Tips:**
- Always start with perimeter mapping
- Maintain consistent speed (0.2 m/s)
- Revisit the same locations multiple times
- Monitor Foxglove for map quality in real-time
- Save multiple versions of maps for comparison
