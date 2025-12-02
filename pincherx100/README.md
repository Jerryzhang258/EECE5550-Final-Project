# IDRIS PincherX100 Gesture Control

Hand gesture control for PincherX100 robotic arm. Part of IDRIS final project for EECE 5550 Mobile Robotics.

## Overview

Controls PincherX100 using hand gestures detected through webcam with MediaPipe. Move your hand to control the arm, pinch to control gripper.

## Requirements

Hardware:
- PincherX100 robotic arm
- USB webcam
- Ubuntu 22.04

Software:
- ROS2 Humble
- Python 3.10+
- MediaPipe, OpenCV, NumPy, SciPy

## Installation

Install dependencies:
```bash
pip3 install mediapipe opencv-python numpy scipy

sudo apt install ros-humble-moveit \
                 ros-humble-interbotix-xsarm-control \
                 ros-humble-interbotix-xsarm-moveit
```

## Usage

Terminal 1 - Launch robot:
```bash
ros2 launch interbotix_xsarm_moveit xsarm_moveit.launch.py robot_model:=px100 hardware_type:=actual
```

Terminal 2 - Run gesture control:
```bash
python3 puppet_control.py
```

## Controls

Hand Gestures:
- Move hand left/right - waist rotation
- Move hand up/down - shoulder angle
- Move hand forward/back - elbow angle
- Pinch thumb and index - close gripper
- Separate fingers - open gripper

Keyboard:
- + faster response
- - smoother motion
- t tighter grip
- g gentler grip
- r reset to home
- q quit

## How It Works

1. MediaPipe detects hand landmarks from webcam
2. Index finger position maps to arm joint angles
3. Exponential smoothing filter reduces jitter
4. Joint commands published to /px100/commands/joint_group
5. Pinch gesture triggers MoveIt2 gripper action

## Configuration

Parameters in gesture_control.py:
```python
self.smoothing = 0.7              # Motion smoothing (0.1=fast, 0.9=slow)
self.pinch_threshold = 0.06       # Pinch sensitivity
self.gripper_grasping = -0.037    # Gripper closed (radians)
self.gripper_released = 0.037     # Gripper open (radians)
self.wrist_angle = 1.23           # Fixed wrist angle (radians)
```

Joint Limits:
```
Waist:    -3.14 to  3.14 rad
Shoulder: -1.8  to  1.5  rad
Elbow:    -1.8  to  1.5  rad
Wrist:    -1.8  to  1.8  rad
```

Workspace:
```
X (reach):   0.08m to 0.25m
Y (lateral): -0.15m to 0.15m
Z (height):  0.05m to 0.25m
```

## Troubleshooting

Camera not found:
```bash
ls /dev/video*
```

Robot not responding:
```bash
ros2 topic list | grep px100
ros2 topic echo /px100/joint_states
```

Hand detection issues:
- Improve lighting
- Keep hand centered in frame
- Remove background clutter

## Performance

- Hand detection: 30 FPS
- Command latency: ~200ms
- Control rate: 30 Hz

## Limitations

- Single hand tracking
- Fixed wrist orientation
- Requires good lighting
- No collision avoidance

## Files
```
puppet_control.py       Main control script
README.md               This file
```

## Team

Mohammed Abdul Rahman  
Rongxuan Zhang (Jerry)  
Isaac Premkumar

EECE 5550 Mobile Robotics  
Northeastern University  
Fall 2025
