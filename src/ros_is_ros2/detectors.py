"""Environment detection utilities."""

import os
import shutil
import subprocess


def detect_ros_version() -> str | None:
    """Detect if ROS/ROS2 is available and which version."""
    ros_version = os.environ.get("ROS_VERSION")
    if ros_version:
        return ros_version

    # Try to detect ROS2 by checking for ros2 command
    if shutil.which("ros2"):
        return "2"

    # Try to detect ROS1 by checking for roscore command
    if shutil.which("roscore"):
        return "1"

    return None


def ensure_ros2_available() -> bool:
    """Check if ros2 command is available."""
    return shutil.which("ros2") is not None


def get_ros2_version() -> str | None:
    """Get the ROS2 distribution name if available."""
    try:
        result = subprocess.run(
            ["ros2", "--version"],
            capture_output=True,
            text=True,
            check=True,
        )
        # Output format: "ros2 doctor 0.7.0 using distribution galactic"
        output = result.stdout.strip()
        if "using distribution" in output:
            return output.split("using distribution")[-1].strip()
    except (subprocess.CalledProcessError, FileNotFoundError):
        pass

    # Try ROS_DISTRO environment variable
    return os.environ.get("ROS_DISTRO")


def check_ros2_environment() -> dict:
    """Return comprehensive ROS2 environment information."""
    return {
        "ros_version": detect_ros_version(),
        "ros2_available": ensure_ros2_available(),
        "ros2_distribution": get_ros2_version(),
        "ros_distro": os.environ.get("ROS_DISTRO"),
        "ament_prefix_path": os.environ.get("AMENT_PREFIX_PATH"),
    }
