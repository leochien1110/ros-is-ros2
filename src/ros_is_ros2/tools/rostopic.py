"""rostopic command - thin shim over 'ros2 topic'."""

import argparse
import os
import sys

from ..completion import (
    SubcommandCompleter,
    create_ros2_argument_completer,
    setup_rostopic_completion,
)
from ..detectors import ensure_ros2_available
from ..exec import exec_ros2_command
from ..mappings import map_ros1_to_ros2


def print_usage() -> None:
    """Print usage information for rostopic."""
    print("rostopic is a command-line tool for printing information about ROS Topics.")
    print()
    print("Commands:")
    print("\trostopic bw\tdisplay bandwidth used by topic")
    print("\trostopic echo\tprint messages to screen")
    print("\trostopic find\tfind topics by type")
    print("\trostopic hz\tdisplay publishing rate of topic")
    print("\trostopic info\tprint information about active topic")
    print("\trostopic list\tlist active topics")
    print("\trostopic pub\tpublish data to topic")
    print("\trostopic type\tprint topic or field type")
    print()
    print("Type rostopic <command> -h for more detailed usage, e.g. 'rostopic echo -h'")
    print()
    print("This is a ROS1-compatibility shim. Commands are forwarded to 'ros2 topic'.")


def print_environment_error() -> None:
    """Print error message when ROS2 is not available."""
    print("Error: ROS2 environment not detected.", file=sys.stderr)
    print("Please ensure ROS2 is installed and sourced:", file=sys.stderr)
    print("  source /opt/ros/<distro>/setup.bash", file=sys.stderr)
    print(
        "Or install ROS2: https://docs.ros.org/en/rolling/Installation.html",
        file=sys.stderr,
    )


def handle_special_cases(args: list[str]) -> int | None:
    """Handle special cases and ROS1-specific behaviors.

    Returns:
        Exit code if handled, None if should continue with normal processing
    """
    if not args:
        print_usage()
        return 0

    subcommand = args[0]

    # Handle help requests
    if subcommand in ["-h", "--help", "help"]:
        print_usage()
        return 0

    # Validate subcommand
    valid_subcommands = ["list", "echo", "hz", "info", "type", "find", "pub", "bw"]
    if subcommand not in valid_subcommands:
        print(f"rostopic: error: unknown command '{subcommand}'", file=sys.stderr)
        print(f"Valid commands are: {', '.join(valid_subcommands)}", file=sys.stderr)
        print("Use 'rostopic -h' for help.", file=sys.stderr)
        return 1

    # Special handling for 'bw' subcommand (not directly available in ROS2)
    if subcommand == "bw":
        print(
            "Warning: 'rostopic bw' is not directly supported in ROS2.",
            file=sys.stderr,
        )
        print(
            "Consider using 'ros2 topic hz' for frequency or other monitoring tools.",
            file=sys.stderr,
        )
        return 1

    return None


def create_parser():
    """Create argument parser with completion support."""
    parser = argparse.ArgumentParser(
        prog="rostopic",
        description=(
            "rostopic is a command-line tool for printing information about ROS Topics."
        ),
        add_help=False,  # We'll handle help manually to match ROS1 behavior
    )

    # Add subcommand argument
    valid_subcommands = ["list", "echo", "hz", "info", "type", "find", "pub", "bw"]

    subcommand_arg = parser.add_argument(
        "subcommand",
        nargs="?",
        choices=valid_subcommands + ["-h", "--help", "help"],
        help="rostopic subcommand"
    )
    subcommand_arg.completer = SubcommandCompleter(
        valid_subcommands + ["-h", "--help", "help"]
    )

    # Add remaining arguments
    args_arg = parser.add_argument(
        "args",
        nargs="*",
        help="Additional arguments for the subcommand"
    )

    # Set up completion for different subcommands
    def args_completer(prefix: str, parsed_args, **kwargs):
        """Complete arguments based on the subcommand."""
        if not parsed_args.subcommand:
            return []

        # For commands that take topic names, complete with topics
        if parsed_args.subcommand in ["echo", "hz", "info", "type", "pub"]:
            return create_ros2_argument_completer(
                "topic", parsed_args.subcommand
            )(prefix, parsed_args, **kwargs)

        return []

    args_arg.completer = args_completer

    # Enable argcomplete if available
    if setup_rostopic_completion():
        try:
            import argcomplete
            argcomplete.autocomplete(parser)
        except ImportError:
            pass  # argcomplete not available

    return parser


def main(args: list[str] | None = None) -> int:
    """Main entry point for rostopic command."""
    if args is None:
        args = sys.argv[1:]

    # Handle special cases first
    special_result = handle_special_cases(args)
    if special_result is not None:
        return special_result

    # Check ROS2 environment
    if not ensure_ros2_available():
        print_environment_error()
        return 1

    # Map ROS1 command to ROS2
    ros2_args = map_ros1_to_ros2("rostopic", args)
    if ros2_args is None:
        print(f"Error: Failed to map rostopic command: {args}", file=sys.stderr)
        return 1

    # Execute the mapped ROS2 command
    # ros2_args contains ['topic', 'list', ...] so we use ros2_args[0] as
    # subcommand and rest as args
    return exec_ros2_command(ros2_args[0], ros2_args[1:])


def main_with_completion():
    """Main entry point with argparse completion support."""
    # If we're being called for completion, handle it
    if "_ARGCOMPLETE" in os.environ:
        parser = create_parser()
        try:
            import argcomplete
            argcomplete.autocomplete(parser)
        except ImportError:
            pass

    # Otherwise, use the regular main function for actual execution
    return main()


if __name__ == "__main__":
    sys.exit(main_with_completion())
