"""roscore command - provides migration guidance."""

import sys


def main(args: list[str] | None = None) -> int:
    """Main entry point for roscore command."""
    if args is None:
        args = sys.argv[1:]

    print("roscore: ROS2 does not use roscore!", file=sys.stderr)
    print()
    print("In ROS2, there is no central master node like roscore in ROS1.")
    print("ROS2 uses DDS (Data Distribution Service) for discovery.")
    print()
    print("Possible alternatives:")
    print("  - Start the ROS2 daemon: ros2 daemon start")
    print("  - Launch your nodes directly: ros2 run <package> <executable>")
    print("  - Use launch files: ros2 launch <package> <launch_file>")
    print()
    print("For more information, see:")
    print("  https://docs.ros.org/en/rolling/Concepts/About-ROS-2.html")

    return 1  # Exit with error code since roscore doesn't exist in ROS2


if __name__ == "__main__":
    sys.exit(main())
