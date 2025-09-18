"""Main dispatcher for ros-is-ros2 package."""

import sys


def main(args: list[str] | None = None) -> int:
    """Main entry point for python -m ros_is_ros2."""
    if args is None:
        args = sys.argv[1:]

    print("ros-is-ros2: ROS1-style commands as thin shims over ROS2")
    print("Available commands:")
    print("  rostopic    - Topic introspection and manipulation")
    print()
    print("This is a thin wrapper around the 'ros2 topic' command.")
    print("For example: 'rostopic list' maps to 'ros2 topic list'")

    return 0


if __name__ == "__main__":
    sys.exit(main())
