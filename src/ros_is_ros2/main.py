"""Main dispatcher for ros-is-ros2 package."""

import sys


def main(args: list[str] | None = None) -> int:
    """Main entry point for python -m ros_is_ros2."""
    if args is None:
        args = sys.argv[1:]

    print("ros-is-ros2: ROS1-style commands as thin shims over ROS2")
    print("Available commands:")
    print("  rostopic    - Topic introspection and manipulation")
    print("  rosnode     - Node introspection")
    print("  rosservice  - Service introspection and calling")
    print("  rosparam    - Parameter manipulation")
    print("  rosrun      - Run packages")
    print("  rosbag      - Bag file handling")
    print("  rosmsg      - Message type introspection")
    print("  rossrv      - Service type introspection")
    print("  roslaunch   - Launch files")
    print("  roscore     - Migration guidance")
    print()
    print("Each command is a thin wrapper around the corresponding 'ros2' command.")
    print("For example: 'rostopic list' maps to 'ros2 topic list'")

    return 0


if __name__ == "__main__":
    sys.exit(main())
