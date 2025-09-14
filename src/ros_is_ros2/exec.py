"""Subprocess execution utilities."""

import subprocess
import sys


def exec_ros2_command(
    subcommand: str,
    args: list[str],
    passthrough_exit_code: bool = True
) -> int:
    """Execute a ros2 command with the given subcommand and arguments.

    Args:
        subcommand: The ros2 subcommand (e.g., 'topic', 'node')
        args: Arguments to pass to the subcommand
        passthrough_exit_code: Whether to return the subprocess exit code

    Returns:
        Exit code from the subprocess, or 0 if passthrough_exit_code is False
    """
    cmd = ["ros2", subcommand] + args

    try:
        # Use subprocess.run to execute and wait for completion
        result = subprocess.run(cmd, check=False)
        exit_code = result.returncode

        if passthrough_exit_code:
            return exit_code
        else:
            return 0

    except FileNotFoundError:
        print("Error: 'ros2' command not found.", file=sys.stderr)
        print("Please ensure ROS2 is installed and sourced.", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully
        return 130


def exec_and_replace(command: list[str]) -> None:
    """Execute a command by replacing the current process.

    This is useful for commands that need to handle signals properly
    or when we want the subprocess to have full control.
    """
    try:
        import os
        os.execvp(command[0], command)
    except FileNotFoundError:
        print(f"Error: '{command[0]}' command not found.", file=sys.stderr)
        sys.exit(1)


def capture_ros2_output(
    subcommand: str,
    args: list[str],
    timeout: float | None = None
) -> tuple[int, str, str]:
    """Capture output from a ros2 command.

    Args:
        subcommand: The ros2 subcommand (e.g., 'topic', 'node')
        args: Arguments to pass to the subcommand
        timeout: Optional timeout in seconds

    Returns:
        Tuple of (exit_code, stdout, stderr)
    """
    cmd = ["ros2", subcommand] + args

    try:
        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=timeout,
            check=False
        )
        return result.returncode, result.stdout, result.stderr

    except FileNotFoundError:
        error_msg = (
            "Error: 'ros2' command not found. "
            "Please ensure ROS2 is installed and sourced."
        )
        return 1, "", error_msg
    except subprocess.TimeoutExpired:
        error_msg = f"Command timed out after {timeout} seconds"
        return 124, "", error_msg
