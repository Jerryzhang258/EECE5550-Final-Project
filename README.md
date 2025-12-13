# TurtleBot4 SLAM Mapping System

[![ROS2](https://img.shields.io/badge/ROS2-Jazzy-blue)](https://docs.ros.org/en/jazzy/)
[![Platform](https://img.shields.io/badge/Platform-TurtleBot4-green)](https://turtlebot.github.io/turtlebot4-user-manual/)
[![License](https://img.shields.io/badge/License-MIT-yellow)](LICENSE)

A complete SLAM-based mapping system for TurtleBot4 autonomous navigation, featuring real-time occupancy grid mapping with SLAM Toolbox, Foxglove Studio visualization, and automated deployment infrastructure.

## Features

- **Real-time SLAM Mapping**: Graph-based SLAM with loop closure detection
- **Automated Launch System**: One-command deployment of entire mapping pipeline
- **Live Visualization**: Foxglove Studio integration for real-time monitoring
- **High Accuracy**: 95% wall consistency with sub-10cm positioning accuracy
- **Robust Loop Closure**: 81% drift reduction through automatic loop detection
- **Production Ready**: Comprehensive error handling and system validation
- **Well Documented**: Complete setup guides and troubleshooting resources

## System Architecture
```
┌─────────────────────────────────────────────────────────┐
│                   TurtleBot4 Hardware                    │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐ │
│  │ RPLiDAR A1   │  │  Create 3    │  │   OAK-D      │ │
│  │   (Sensor)   │  │   (Base)     │  │  (Camera)    │ │
│  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘ │
└─────────┼──────────────────┼──────────────────┼─────────┘
          │                  │                  │
          ▼                  ▼                  ▼
┌─────────────────────────────────────────────────────────┐
│                    ROS2 Middleware                       │
│  ┌──────────────────────────────────────────────────┐  │
│  │              SLAM Toolbox Node                   │  │
│  │  • Scan Matching  • Loop Closure  • Map Update   │  │
│  └─────────────────────┬────────────────────────────┘  │
│                        │                                │
│  ┌────────────────────┼────────────────────────────┐  │
│  │     /map           │ /scan          /tf         │  │
│  └────────────────────┼────────────────────────────┘  │
└───────────────────────┼──────────────────────────────────┘
                        │
                        ▼
          ┌─────────────────────────┐
          │  Foxglove Bridge        │
          │  (WebSocket: 8765)      │
          └─────────────┬───────────┘
                        │
                        ▼
          ┌─────────────────────────┐
          │  Foxglove Studio        │
          │  (Web Visualization)    │
          └─────────────────────────┘
```

## Prerequisites

### Hardware Requirements
- **Robot**: TurtleBot4 (Standard or Lite)
- **LiDAR**: RPLiDAR A1 (included with TurtleBot4)
- **Computer**: Raspberry Pi 4B (4GB RAM minimum)

### Software Requirements
- **OS**: Ubuntu 24.04 LTS
- **ROS**: ROS2 Jazzy (2024.04 release)
- **Python**: 3.12 or later
- **Network**: WiFi connection for Foxglove visualization

## Installation

### Automated Setup (Recommended)
```bash
# Clone the repository
git clone https://github.com/Jerryzhang258/EECE5550-Final-Project.git
cd EECE5550-Final-Project

# Run automated setup
chmod +x scripts/setup/install_dependencies.sh
./scripts/setup/install_dependencies.sh

# Verify installation
./scripts/utils/verify_system.sh
```

### Manual Setup
```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install ROS2 Jazzy (if not already installed)
# Follow: https://docs.ros.org/en/jazzy/Installation.html

# Install required packages
sudo apt install -y \
    ros-jazzy-turtlebot4-navigation \
    ros-jazzy-slam-toolbox \
    ros-jazzy-foxglove-bridge \
    ros-jazzy-nav2-map-server \
    ros-jazzy-teleop-twist-keyboard

# Source ROS2
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Make scripts executable
find scripts/ -name "*.sh" -exec chmod +x {} \;
```

## Quick Start

### Launch Complete System
```bash
# Start all components (3 terminals automatically)
./scripts/launch/start_all.sh
```

This will open:
1. **Terminal 1**: Foxglove Bridge (visualization server)
2. **Terminal 2**: SLAM Toolbox (mapping algorithm)
3. **Terminal 3**: Keyboard Teleop (robot control)

### Connect Foxglove Studio

1. Open browser: [https://app.foxglove.dev](https://app.foxglove.dev)
2. Click "Open connection" → "Foxglove WebSocket"
3. Enter: `ws://localhost:8765`
4. Click "Open"

### Configure Visualization

1. Add "3D" panel (click `+` button)
2. Click ⚙️ on 3D panel
3. Set **Fixed frame** to `map` ← **CRITICAL!**
4. Check these topics:
   - ✓ `/map` (nav_msgs/OccupancyGrid)
   - ✓ `/scan` (sensor_msgs/LaserScan)
   - ✓ `/tf` (tf2_msgs/TFMessage)

### Drive and Map

Use keyboard controls in Terminal 3:
```
Movement:        Speed Control:
  i = forward      q/z = all speeds ±10%
  j = left         w/x = linear only
  l = right        e/c = angular only
  k = STOP
  , = backward

Recommended: Press 'x' several times to slow to 0.2 m/s
```

### Save Your Map
```bash
# In a new terminal
./scripts/mapping/save_map.sh

# Maps saved to: ~/turtlebot4_maps/
# Files: map_TIMESTAMP.pgm, map_TIMESTAMP.yaml
```

## Usage Guide

### Mapping Best Practices

1. **Start Slow**: Reduce speed to 0.2-0.3 m/s using `x` key
2. **Perimeter First**: Drive along walls to establish boundaries
3. **Fill Interior**: Use zigzag or spiral pattern for full coverage
4. **Loop Closure**: Return to starting point via different path
5. **Smooth Motion**: Avoid sudden turns or stops

### Coverage Strategy
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

### Quality Indicators

**Good Map Signs:**
- ✓ Walls are sharp and continuous
- ✓ LiDAR points align with map
- ✓ No ghosting or duplicate features
- ✓ Clear free/occupied distinction

**Needs Improvement:**
- ✗ Blurry walls → Slow down
- ✗ Misaligned sections → Return to known areas
- ✗ Large blank areas → Need more exploration

## Configuration

### SLAM Parameters

Edit `config/slam_params.yaml`:
```yaml
slam_toolbox:
  ros__parameters:
    resolution: 0.05              # meters per cell
    max_laser_range: 12.0        # meters
    minimum_travel_distance: 0.2  # meters
    minimum_travel_heading: 0.2   # radians
    loop_search_space_dimension: 8.0  # meters
    loop_match_minimum_score: 0.6     # confidence threshold
```

## Performance Metrics

From 5 experimental trials:

| Metric | Mean | Std Dev | Unit |
|--------|------|---------|------|
| Mapping time | 8.4 | 1.2 | minutes |
| Map resolution | 0.05 | 0 | m/cell |
| Loop closures | 4.2 | 1.1 | count |
| Wall consistency | 95.3 | 2.1 | % |
| Area mapped | 300 | 15 | m² |

### Loop Closure Performance

- Mean error before loop closure: **0.42 m**
- Mean error after loop closure: **0.08 m**
- Drift reduction: **81%**

## Troubleshooting

### Common Issues

<details>
<summary><b>No map visible in Foxglove</b></summary>

**Solution:**
1. Check Fixed frame = `map` (most common issue!)
2. Verify `/map` topic is checked in left panel
3. Confirm SLAM is running: `ros2 node list | grep slam`
4. Check map is publishing: `ros2 topic hz /map`
</details>

<details>
<summary><b>Robot not moving</b></summary>

**Solution:**
1. Click on teleop terminal to give it focus
2. Test manually: `ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"`
3. Check if cmd_vel is published: `ros2 topic echo /cmd_vel`
</details>

<details>
<summary><b>Blurry or poor quality map</b></summary>

**Solution:**
1. Slow down: Press `x` multiple times (target: 0.2 m/s)
2. Return to known areas for loop closure
3. Clean LiDAR lens
4. Check LiDAR data quality: `ros2 topic echo /scan --once`
</details>

<details>
<summary><b>Foxglove connection failed</b></summary>

**Solution:**
1. Check bridge is running: `ros2 node list | grep foxglove`
2. Verify port: `netstat -tulpn | grep 8765`
3. Restart bridge: `./scripts/launch/launch_foxglove.sh`
</details>

### System Verification

Run comprehensive diagnostics:
```bash
./scripts/utils/verify_system.sh
```

## Project Structure
```
EECE5550-Final-Project/
├── scripts/
│   ├── setup/          # Installation scripts
│   ├── launch/         # System launch scripts
│   ├── mapping/        # Map save/load utilities
│   └── utils/          # Verification tools
├── config/             # Configuration files
├── docs/               # Documentation
└── examples/           # Example maps
```

## Documentation

- [Quick Reference](docs/QUICK_REFERENCE.md)
- Full setup guide in README (you're reading it!)

## Contributing

Contributions are welcome! Please:
1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Submit a pull request

## License

This project is licensed under the MIT License - see [LICENSE](LICENSE) file for details.

## Citation

If you use this work in your research, please cite:
```bibtex
@misc{zhang2025turtlebot4slam,
  author = {Zhang, Rongxuan},
  title = {TurtleBot4 SLAM Mapping System},
  year = {2025},
  publisher = {GitHub},
  howpublished = {\url{https://github.com/Jerryzhang258/EECE5550-Final-Project}}
}
```

## Acknowledgments

- EECE5550 Mobile Robotics course staff at Northeastern University
- TurtleBot4 development team and documentation contributors
- SLAM Toolbox and Nav2 open-source communities
- Foxglove Studio visualization platform

---

**Project developed for EECE5550 Mobile Robotics, Fall 2025**
**Northeastern University, Seattle Campus*# EECE5550-Final-Project
