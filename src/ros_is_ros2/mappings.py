"""Dynamic command mappings between ROS1 and ROS2."""

from typing import List, Optional, Dict

from .discovery import get_discovery, translate_ros1_to_ros2


class CommandMapping:
    """Maps ROS1 commands to ROS2 equivalents with special flag handling."""

    def __init__(self, ros2_command: str, ros2_subcommand: str):
        self.ros2_command = ros2_command
        self.ros2_subcommand = ros2_subcommand
        self._flag_mappings: dict[str, str] = {}

    def add_flag_mapping(self, ros1_flag: str, ros2_flag: str) -> None:
        """Add a flag mapping from ROS1 to ROS2."""
        self._flag_mappings[ros1_flag] = ros2_flag

    def map_arguments(self, args: list[str]) -> list[str]:
        """Map ROS1-style arguments to ROS2-style arguments."""
        mapped_args = []
        i = 0

        while i < len(args):
            arg = args[i]

            # Handle flag mappings
            if arg in self._flag_mappings:
                mapped_args.append(self._flag_mappings[arg])
                # Check if this flag takes a value
                if i + 1 < len(args) and not args[i + 1].startswith("-"):
                    i += 1
                    mapped_args.append(args[i])
            else:
                # Pass through unchanged
                mapped_args.append(arg)

            i += 1

        return mapped_args


# Special case mappings for commands that don't translate directly
SPECIAL_MAPPINGS: Dict[str, Dict[str, CommandMapping]] = {}



def get_mapped_command(tool: str, subcommand: str) -> CommandMapping | None:
    """Get the ROS2 command mapping for a given ROS1 tool and subcommand."""
    # Check special mappings first
    if tool in SPECIAL_MAPPINGS:
        special_mapping = SPECIAL_MAPPINGS[tool].get(subcommand)
        if special_mapping:
            return special_mapping
    
    # For most commands, there's no special mapping needed
    return None


def map_ros1_to_ros2_with_special_cases(tool: str, args: list[str]) -> list[str] | None:
    """Map ROS1 command to ROS2 with special case handling."""
    # Use dynamic discovery for all translations
    return translate_ros1_to_ros2(tool, args)


def map_ros1_to_ros2(tool: str, args: list[str]) -> list[str] | None:
    """Map complete ROS1 command to ROS2 equivalent.

    Args:
        tool: The ROS1 tool name (e.g., 'rostopic')
        args: The arguments including subcommand

    Returns:
        Mapped ROS2 command as list of strings, or None if no mapping exists
    """
    return map_ros1_to_ros2_with_special_cases(tool, args)
