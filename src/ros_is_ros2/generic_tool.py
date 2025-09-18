"""Generic tool handler for any ros<X> command."""

import argparse
import os
import sys
from pathlib import Path
from typing import List, Optional

from .completion import DynamicSubcommandCompleter, create_ros2_argument_completer
from .detectors import ensure_ros2_available
from .discovery import get_discovery, translate_ros1_to_ros2
from .exec import exec_ros2_command


def _setup_completion_once():
    """Set up tab completion automatically on first run."""
    # Check if we should set up completion
    completion_marker = "# ros-is-ros2 auto completion"
    bashrc_path = Path.home() / ".bashrc"
    
    # Skip if not in interactive terminal or bashrc doesn't exist
    if not (os.isatty(sys.stdin.fileno()) and bashrc_path.exists()):
        return
    
    try:
        # Check if completion is already set up
        with open(bashrc_path, 'r') as f:
            content = f.read()
        
        if completion_marker in content:
            return  # Already set up
        
        # Create completion lines
        completion_lines = [
            "",
            completion_marker,
            'eval "$(register-python-argcomplete rostopic)"',
            ""
        ]
        
        # Add completion to bashrc
        with open(bashrc_path, 'a') as f:
            f.write('\n'.join(completion_lines))
        
        print("✅ Tab completion for ros-is-ros2 has been automatically enabled!")
        print("💡 Restart your shell or run 'source ~/.bashrc' to activate completion")
        print("   Then try: rostopic <TAB> or rostopic li<TAB>")
        
    except (OSError, PermissionError):
        # Silently skip if we can't modify bashrc
        pass


def print_generic_usage(command_name: str) -> None:
    """Print usage information for a generic ros command."""
    discovery = get_discovery()
    ros2_cmd = discovery.get_ros1_equivalent(command_name)
    
    if not ros2_cmd:
        print(f"Error: '{command_name}' does not have a ROS2 equivalent.", file=sys.stderr)
        return
    
    print(f"{command_name} is a command-line tool for ROS {ros2_cmd.upper()}.")
    print()
    
    # Get available subcommands dynamically
    subcommands = discovery.get_valid_subcommands(command_name)
    
    if subcommands:
        print("Commands:")
        for subcommand in sorted(subcommands):
            print(f"\t{command_name} {subcommand}")
    else:
        print("No subcommands available.")
    
    print()
    print(f"Type {command_name} <command> -h for more detailed usage, e.g. '{command_name} list -h'")
    print()
    print(f"This is a ROS1-compatibility shim. Commands are forwarded to 'ros2 {ros2_cmd}'.")


def print_environment_error() -> None:
    """Print error message when ROS2 is not available."""
    print("Error: ROS2 environment not detected.", file=sys.stderr)
    print("Please ensure ROS2 is installed and sourced:", file=sys.stderr)
    print("  source /opt/ros/<distro>/setup.bash", file=sys.stderr)
    print(
        "Or install ROS2: https://docs.ros.org/en/rolling/Installation.html",
        file=sys.stderr,
    )


def handle_special_cases(command_name: str, args: List[str]) -> Optional[int]:
    """Handle special cases and ROS1-specific behaviors.

    Returns:
        Exit code if handled, None if should continue with normal processing
    """
    if not args:
        print_generic_usage(command_name)
        return 0

    subcommand = args[0]

    # Handle help requests
    if subcommand in ["-h", "--help", "help"]:
        print_generic_usage(command_name)
        return 0

    # No need to validate here - validation is done in main_generic
    return None


def create_generic_parser(command_name: str):
    """Create argument parser with completion support for any ros command."""
    discovery = get_discovery()
    
    parser = argparse.ArgumentParser(
        prog=command_name,
        description=f"{command_name} is a command-line tool for ROS.",
        add_help=False,  # We'll handle help manually to match ROS1 behavior
    )

    # Get valid subcommands dynamically
    valid_subcommands = discovery.get_valid_subcommands(command_name)

    # Add subcommand argument - don't use choices to allow flexibility
    subcommand_arg = parser.add_argument(
        "subcommand",
        nargs="?",
        help=f"{command_name} subcommand"
    )

    # Use dynamic completer for subcommands
    subcommand_arg.completer = DynamicSubcommandCompleter(command_name)

    # Add context-aware optional arguments based on the command type
    if command_name in ["rostopic"]:
        # Topic-specific flags
        parser.add_argument("-t", "--show-types", action="store_true", help="show types")
        parser.add_argument("-c", "--count-topics", action="store_true", help="count topics")
        parser.add_argument("-v", "--verbose", action="store_true", help="verbose output")
        parser.add_argument("-s", "--use-sim-time", action="store_true", help="use sim time")
        parser.add_argument("--no-daemon", action="store_true", help="no daemon")
        parser.add_argument("--spin-time", type=float, help="spin time")
        parser.add_argument("--include-hidden-topics", action="store_true", help="include hidden topics")
    else:
        # Generic flags for other commands
        parser.add_argument("-h", "--help", action="store_true", help="show help")
        parser.add_argument("-v", "--verbose", action="store_true", help="verbose output")

    # Add remaining arguments
    args_arg = parser.add_argument(
        "args",
        nargs="*",
        help="Additional arguments for the subcommand"
    )

    # Create a custom completer that handles topic names
    def smart_completer(prefix: str, parsed_args, **kwargs):
        """Smart completer for positional arguments (topics)."""
        # Get the ros2 equivalent command
        ros2_cmd = discovery.get_ros1_equivalent(command_name)
        if not ros2_cmd:
            return []

        # Handle commands that don't use subcommands
        if not hasattr(parsed_args, 'subcommand') or not parsed_args.subcommand:
            return []

        # Only provide topic completions, not flags
        # (flags are handled automatically by argparse)
        if prefix.startswith('-'):
            return []

        # Call our argument completer for non-flag completions
        return create_ros2_argument_completer(
            ros2_cmd, parsed_args.subcommand
        )(prefix, parsed_args, **kwargs)

    # Set the completer on the appropriate arguments
    args_arg.completer = smart_completer

    # Don't call autocomplete here - it will be called in main_with_completion
    return parser


def main_generic(command_name: str, args: Optional[List[str]] = None) -> int:
    """Main entry point for any generic ros command."""
    if args is None:
        args = sys.argv[1:]

    # Set up completion automatically on first run
    _setup_completion_once()

    # Check if this is a valid ros1-style command
    discovery = get_discovery()
    if not discovery.is_valid_ros1_command(command_name):
        print(f"Error: '{command_name}' is not a supported ROS command.", file=sys.stderr)
        available_commands = [f"ros{cmd}" for cmd in discovery.get_ros2_commands()]
        if available_commands:
            print(f"Available commands: {', '.join(sorted(available_commands))}", file=sys.stderr)
        return 1

    # Handle special cases first
    special_result = handle_special_cases(command_name, args)
    if special_result is not None:
        return special_result
    
    # Validate subcommand if present
    if args:
        subcommand = args[0]
        # Allow help flags to pass through
        if subcommand not in ['-h', '--help', 'help']:
            valid_subcommands = discovery.get_valid_subcommands(command_name)
            if subcommand not in valid_subcommands:
                print(f"{command_name}: error: unknown command '{subcommand}'", file=sys.stderr)
                if valid_subcommands:
                    print(f"Valid commands are: {', '.join(sorted(valid_subcommands))}", file=sys.stderr)
                print(f"Use '{command_name} -h' for help.", file=sys.stderr)
                return 1

    # Check ROS2 environment
    if not ensure_ros2_available():
        print_environment_error()
        return 1

    # Translate ROS1 command to ROS2
    ros2_args = translate_ros1_to_ros2(command_name, args)
    if ros2_args is None:
        print(f"Error: Failed to map {command_name} command: {args}", file=sys.stderr)
        return 1

    # Execute the mapped ROS2 command
    return exec_ros2_command(ros2_args[0], ros2_args[1:])


def main_with_completion(command_name: str):
    """Main entry point with argparse completion support."""
    import sys
    parser = create_generic_parser(command_name)
    
    # Enable argcomplete before parsing
    try:
        import argcomplete
        argcomplete.autocomplete(parser)
    except ImportError:
        pass  # argcomplete not available
    
    # If we're not in completion mode, parse and execute
    if "_ARGCOMPLETE" not in os.environ:
        # Use parse_known_args to be more flexible with unknown arguments
        args, unknown = parser.parse_known_args()
        
        # Handle help manually to match ROS1 behavior - only for top-level help
        if hasattr(args, 'subcommand') and args.subcommand in ['-h', '--help', 'help']:
            print_generic_usage(command_name)
            return 0
        
        # Build command arguments
        cmd_args = []
        if hasattr(args, 'subcommand') and args.subcommand:
            cmd_args.append(args.subcommand)
            
        # Add optional flags if they were specified
        if hasattr(args, 'show_types') and args.show_types:
            cmd_args.append("--show-types")
        if hasattr(args, 'count_topics') and args.count_topics:
            cmd_args.append("--count-topics")
        if hasattr(args, 'verbose') and args.verbose:
            cmd_args.append("--verbose")
        if hasattr(args, 'use_sim_time') and args.use_sim_time:
            cmd_args.append("--use-sim-time")
        if hasattr(args, 'no_daemon') and args.no_daemon:
            cmd_args.append("--no-daemon")
        if hasattr(args, 'spin_time') and args.spin_time:
            cmd_args.extend(["--spin-time", str(args.spin_time)])
        if hasattr(args, 'include_hidden_topics') and args.include_hidden_topics:
            cmd_args.append("--include-hidden-topics")
            
        if hasattr(args, 'args') and args.args:
            cmd_args.extend(args.args)
        # Add any unknown arguments (like --help after subcommand)
        if unknown:
            cmd_args.extend(unknown)
            
        return main_generic(command_name, cmd_args)
    
    # If we reach here, we're in completion mode and argcomplete handled it
    return 0
