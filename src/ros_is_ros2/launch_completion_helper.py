#!/usr/bin/env python3
"""
Launch file argument completion helper for ROS 2
Copyright (C) 2025 ros-is-ros2 contributors

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
"""

from __future__ import annotations

import os
import re
import subprocess
import sys
import time
from pathlib import Path
from typing import List

# Cache configuration
CACHE_DIR = Path.home() / ".cache" / "ros-is-ros2" / "launch_args"
CACHE_TTL = int(os.environ.get("ROS_IS_ROS2_CACHE_TTL", "3600"))  # 1 hour default
TIMEOUT = int(os.environ.get("ROS_IS_ROS2_COMPLETION_TIMEOUT", "3"))  # 3 seconds


def get_cache_path(package: str, launch_file: str) -> Path:
    """Get the cache file path for a given package and launch file."""
    safe_package = package.replace("/", "_").replace(".", "_")
    safe_file = launch_file.replace("/", "_").replace(".", "_")
    return CACHE_DIR / f"{safe_package}_{safe_file}.cache"


def is_cache_valid(cache_path: Path) -> bool:
    """Check if cache file exists and is not expired."""
    if not cache_path.exists():
        return False

    # Check if cache is expired
    cache_age = time.time() - cache_path.stat().st_mtime
    return cache_age < CACHE_TTL


def read_cache(cache_path: Path) -> List[str]:
    """Read argument names from cache file."""
    try:
        with open(cache_path, "r") as f:
            return [line.strip() for line in f if line.strip()]
    except (OSError, UnicodeDecodeError):
        return []


def write_cache(cache_path: Path, arguments: List[str]) -> None:
    """Write argument names to cache file."""
    try:
        cache_path.parent.mkdir(parents=True, exist_ok=True)
        with open(cache_path, "w") as f:
            f.write("\n".join(arguments) + "\n")
    except OSError:
        # Silently fail if we can't write cache
        pass


def parse_launch_args(output: str) -> List[str]:
    """
    Parse the output of 'ros2 launch --show-args' to extract argument names.

    Expected format:
        Arguments (pass arguments as '<name>:=<value>'):

            'camera_name':
                camera unique name
                (default: 'camera')

            'camera_namespace':
                namespace for camera
                (default: 'camera')
    """
    arguments = []

    # Pattern to match argument names (they appear as 'name': at the start of a line)
    # Look for lines that start with whitespace, then a quoted string, then a colon
    pattern = r"^\s+'([^']+)':"

    for line in output.split("\n"):
        match = re.match(pattern, line)
        if match:
            arguments.append(match.group(1))

    return arguments


def get_launch_arguments(package: str, launch_file: str) -> List[str]:
    """
    Get launch file arguments by calling 'ros2 launch --show-args'.
    Uses caching to improve performance.
    """
    # Check cache first
    cache_path = get_cache_path(package, launch_file)
    if is_cache_valid(cache_path):
        cached_args = read_cache(cache_path)
        if cached_args is not None:
            return cached_args

    # Cache miss or invalid, query ros2 launch
    try:
        result = subprocess.run(
            ["ros2", "launch", package, launch_file, "--show-args"],
            capture_output=True,
            text=True,
            timeout=TIMEOUT,
        )

        if result.returncode == 0:
            arguments = parse_launch_args(result.stdout)
            # Cache the results
            write_cache(cache_path, arguments)
            return arguments
        else:
            # Launch file has errors or doesn't exist
            return []

    except subprocess.TimeoutExpired:
        # Launch file takes too long to introspect
        return []
    except (subprocess.SubprocessError, FileNotFoundError):
        # ros2 command not found or other error
        return []
    except Exception:
        # Catch-all for any other errors
        return []


def main() -> int:
    """Main entry point for the completion helper."""
    if len(sys.argv) != 3:
        print(
            "Usage: launch_completion_helper.py <package> <launch_file>",
            file=sys.stderr,
        )
        return 1

    package = sys.argv[1]
    launch_file = sys.argv[2]

    arguments = get_launch_arguments(package, launch_file)

    # Output one argument per line for easy parsing by shell
    for arg in arguments:
        print(arg)

    return 0


if __name__ == "__main__":
    sys.exit(main())
