# ros-is-ros2

ROS1-style commands as thin shims over ROS2 to improve developer ergonomics.

## Overview

This package provides familiar ROS1 commands (`rostopic`, `rosnode`, etc.) that act as thin wrappers around their ROS2 equivalents (`ros2 topic`, `ros2 node`, etc.). This helps developers transition from ROS1 to ROS2 while maintaining muscle memory for common CLI operations.

## Installation

```bash
pip install ros-is-ros2
```

## Supported Commands

- `rostopic` → `ros2 topic`
- `rosnode` → `ros2 node`
- `rosservice` → `ros2 service`
- `rosparam` → `ros2 param`
- `rosrun` → `ros2 run`
- `rosbag` → `ros2 bag`
- `rosmsg` → `ros2 interface`
- `rossrv` → `ros2 interface`
- `roslaunch` → `ros2 launch`
- `roscore` → Migration guidance

## Usage

After installation, use ROS1 commands as you normally would:

```bash
rostopic list
rostopic echo /chatter
rosnode list
rosrun demo_nodes_cpp talker
```

Each command forwards to the appropriate ROS2 equivalent with mapped arguments.

## Requirements

- ROS2 (Humble, Iron, Jazzy, or Rolling)
- Python 3.10+

## Development Status

This project is in early development. See the project plan for current status and roadmap.
