"""Autocompletion support for ros-is-ros2 commands."""

import subprocess

from .detectors import ensure_ros2_available


class ROS2CompletionDelegate:
    """Delegates completion to ros2 command for version-agnostic completion."""

    def __init__(self, ros2_subcommand: str):
        """Initialize with the ros2 subcommand (e.g., 'topic', 'node')."""
        self.ros2_subcommand = ros2_subcommand

    def complete(self, prefix: str, parsed_args, **kwargs) -> list[str]:
        """Complete arguments by delegating to ros2 completion."""
        if not ensure_ros2_available():
            return self._get_static_completions(prefix, parsed_args)

        # Build the ros2 command equivalent 
        ros2_cmd_parts = ["ros2", self.ros2_subcommand]

        # Add subcommand if present
        if hasattr(parsed_args, "subcommand") and parsed_args.subcommand:
            ros2_cmd_parts.append(parsed_args.subcommand)

        # Add any additional arguments that have been parsed
        if hasattr(parsed_args, "args") and parsed_args.args:
            ros2_cmd_parts.extend(parsed_args.args)

        # Try to get completions by calling ros2's argcomplete directly
        return self._get_ros2_completions(ros2_cmd_parts, prefix)

    def _get_ros2_completions(self, ros2_cmd_parts: list[str], prefix: str) -> list[str]:
        """Get completions by calling ros2's argcomplete completion directly."""
        try:
            # Create the command line that ros2 would see
            ros2_cmd_line = " ".join(ros2_cmd_parts)
            if prefix:
                ros2_cmd_line += " " + prefix

            # Set up the environment for argcomplete
            env = dict(subprocess.os.environ)
            env.update({
                "_ARGCOMPLETE": "1",
                "_ARGCOMPLETE_COMP_WORDBREAKS": ' \t\n"`!@#$%^&*()=+[{]}\\|;:\'",<>?',
                "COMP_LINE": ros2_cmd_line,
                "COMP_POINT": str(len(ros2_cmd_line)),
            })

            # Call ros2 with argcomplete environment to get completions
            result = subprocess.run(
                ros2_cmd_parts[:2],  # Just ["ros2", "topic"] to trigger argcomplete
                env=env,
                capture_output=True,
                text=True,
                timeout=3
            )

            # Parse the completions from the output
            if result.stdout:
                completions = []
                for line in result.stdout.strip().split('\n'):
                    line = line.strip()
                    if line and not line.startswith('usage:'):
                        completions.append(line)
                
                # Filter by prefix if provided
                if prefix:
                    completions = [comp for comp in completions if comp.startswith(prefix)]
                
                if completions:
                    return completions

        except (subprocess.TimeoutExpired, subprocess.CalledProcessError, FileNotFoundError):
            pass

        # If argcomplete approach fails, try getting help output for flags
        return self._get_help_based_completions(ros2_cmd_parts, prefix)

    def _get_help_based_completions(self, ros2_cmd_parts: list[str], prefix: str) -> list[str]:
        """Get completions by parsing ros2 help output."""
        try:
            # Get help output from ros2 command
            result = subprocess.run(
                ros2_cmd_parts + ["--help"],
                capture_output=True,
                text=True,
                timeout=3
            )

            if result.returncode == 0:
                completions = []
                lines = result.stdout.split('\n')
                
                # Parse options from help output
                in_options = False
                for line in lines:
                    if 'options:' in line.lower() or 'optional arguments:' in line.lower():
                        in_options = True
                        continue
                    
                    if in_options and line.strip():
                        # Look for lines that start with flags
                        line = line.strip()
                        if line.startswith('-'):
                            # Extract flags like "-h, --help" or "--count-topics"
                            flag_parts = line.split()
                            for part in flag_parts:
                                if part.startswith('-') and not ',' in part:
                                    completions.append(part)
                        elif not line.startswith(' ') and line != '':
                            # New section started, stop parsing options
                            break

                # Also get topic/node/service names for appropriate commands
                if len(ros2_cmd_parts) >= 3:  # ros2 topic echo
                    entity_completions = []
                    if self.ros2_subcommand == "topic":
                        entity_completions = self._complete_topics(prefix)
                    elif self.ros2_subcommand == "node":
                        entity_completions = self._complete_nodes(prefix)
                    elif self.ros2_subcommand == "service":
                        entity_completions = self._complete_services(prefix)
                    
                    completions.extend(entity_completions)

                # Filter by prefix
                if prefix:
                    completions = [comp for comp in completions if comp.startswith(prefix)]
                
                return completions

        except (subprocess.TimeoutExpired, subprocess.CalledProcessError, FileNotFoundError):
            pass

        # Final fallback to static completions
        return self._get_static_completions(prefix, None)

    def _get_static_completions(self, prefix: str, parsed_args) -> list[str]:
        """Provide static completions when dynamic completion fails."""
        if self.ros2_subcommand == "topic":
            return self._complete_topics(prefix)
        elif self.ros2_subcommand == "node":
            return self._complete_nodes(prefix)
        elif self.ros2_subcommand == "service":
            return self._complete_services(prefix)
        return []

    def _complete_topics(self, prefix: str) -> list[str]:
        """Get topic names from ros2 topic list."""
        try:
            result = subprocess.run(
                ["ros2", "topic", "list"],
                capture_output=True,
                text=True,
                timeout=3
            )
            if result.returncode == 0:
                topics = result.stdout.strip().split("\n")
                return [topic for topic in topics if topic.startswith(prefix)]
        except (
            subprocess.TimeoutExpired,
            subprocess.CalledProcessError,
            FileNotFoundError,
        ):
            pass
        return []

    def _complete_nodes(self, prefix: str) -> list[str]:
        """Get node names from ros2 node list."""
        try:
            result = subprocess.run(
                ["ros2", "node", "list"],
                capture_output=True,
                text=True,
                timeout=3
            )
            if result.returncode == 0:
                nodes = result.stdout.strip().split("\n")
                return [node for node in nodes if node.startswith(prefix)]
        except (
            subprocess.TimeoutExpired,
            subprocess.CalledProcessError,
            FileNotFoundError,
        ):
            pass
        return []

    def _complete_services(self, prefix: str) -> list[str]:
        """Get service names from ros2 service list."""
        try:
            result = subprocess.run(
                ["ros2", "service", "list"],
                capture_output=True,
                text=True,
                timeout=3
            )
            if result.returncode == 0:
                services = result.stdout.strip().split("\n")
                return [service for service in services if service.startswith(prefix)]
        except (
            subprocess.TimeoutExpired,
            subprocess.CalledProcessError,
            FileNotFoundError,
        ):
            pass
        return []


class SubcommandCompleter:
    """Completes subcommands for ros-is-ros2 tools."""

    def __init__(self, valid_subcommands: list[str]):
        """Initialize with list of valid subcommands."""
        self.valid_subcommands = valid_subcommands

    def __call__(self, prefix: str, **kwargs) -> list[str]:
        """Return subcommands that match the prefix."""
        return [cmd for cmd in self.valid_subcommands if cmd.startswith(prefix)]


def setup_rostopic_completion():
    """Set up completion for rostopic command."""
    try:
        import argcomplete  # noqa: F401
        return True
    except ImportError:
        return False


def create_ros2_argument_completer(ros2_subcommand: str, tool_subcommand: str):
    """Create a completer for arguments that delegates to ros2.

    Args:
        ros2_subcommand: The ros2 subcommand (e.g., 'topic')
        tool_subcommand: The tool subcommand (e.g., 'echo')
    """
    def completer(prefix: str, parsed_args, **kwargs):
        """Complete by delegating to ros2 command."""
        if not ensure_ros2_available():
            return []

        # For topic names and other arguments, delegate to ros2
        delegate = ROS2CompletionDelegate(ros2_subcommand)

        # Create a mock parsed_args with the tool subcommand
        class MockArgs:
            def __init__(self):
                self.subcommand = tool_subcommand
                self.args = []

        mock_args = MockArgs()
        return delegate.complete(prefix, mock_args)

    return completer
