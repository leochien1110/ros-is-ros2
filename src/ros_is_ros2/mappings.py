"""Command and flag mappings between ROS1 and ROS2."""



class CommandMapping:
    """Maps ROS1 commands to ROS2 equivalents."""

    def __init__(self, ros2_command: str, ros2_subcommand: str):
        self.ros2_command = ros2_command
        self.ros2_subcommand = ros2_subcommand
        self._flag_mappings: dict[str, str] = {}
        self._positional_mappings: list[tuple[int, int]] = []

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


# Command mappings for each tool
COMMAND_MAPPINGS = {
    "rostopic": {
        "list": CommandMapping("ros2", "topic list"),
        "echo": CommandMapping("ros2", "topic echo"),
        "hz": CommandMapping("ros2", "topic hz"),
        "info": CommandMapping("ros2", "topic info"),
        "type": CommandMapping("ros2", "topic type"),
        "find": CommandMapping("ros2", "topic find"),
        "pub": CommandMapping("ros2", "topic pub"),
        "bw": CommandMapping("ros2", "topic bw"),
    },
    "rosnode": {
        "list": CommandMapping("ros2", "node list"),
        "info": CommandMapping("ros2", "node info"),
    },
    "rosservice": {
        "list": CommandMapping("ros2", "service list"),
        "type": CommandMapping("ros2", "service type"),
        "call": CommandMapping("ros2", "service call"),
        "find": CommandMapping("ros2", "service find"),
    },
    "rosparam": {
        "list": CommandMapping("ros2", "param list"),
        "get": CommandMapping("ros2", "param get"),
        "set": CommandMapping("ros2", "param set"),
        "dump": CommandMapping("ros2", "param dump"),
        "load": CommandMapping("ros2", "param load"),
        "delete": CommandMapping("ros2", "param delete"),
    },
    "rosbag": {
        "record": CommandMapping("ros2", "bag record"),
        "play": CommandMapping("ros2", "bag play"),
        "info": CommandMapping("ros2", "bag info"),
        "reindex": CommandMapping("ros2", "bag reindex"),
        "convert": CommandMapping("ros2", "bag convert"),
    },
    "rosmsg": {
        "list": CommandMapping("ros2", "interface list -m"),
        "show": CommandMapping("ros2", "interface show"),
        "info": CommandMapping("ros2", "interface show"),
        "package": CommandMapping("ros2", "interface package"),
    },
    "rossrv": {
        "list": CommandMapping("ros2", "interface list -s"),
        "show": CommandMapping("ros2", "interface show"),
        "info": CommandMapping("ros2", "interface show"),
        "package": CommandMapping("ros2", "interface package"),
    }
}


def get_mapped_command(tool: str, subcommand: str) -> CommandMapping | None:
    """Get the ROS2 command mapping for a given ROS1 tool and subcommand."""
    tool_mappings = COMMAND_MAPPINGS.get(tool)
    if not tool_mappings:
        return None

    return tool_mappings.get(subcommand)


def map_ros1_to_ros2(tool: str, args: list[str]) -> list[str] | None:
    """Map complete ROS1 command to ROS2 equivalent.

    Args:
        tool: The ROS1 tool name (e.g., 'rostopic')
        args: The arguments including subcommand

    Returns:
        Mapped ROS2 command as list of strings, or None if no mapping exists
    """
    if not args:
        return None

    subcommand = args[0]
    remaining_args = args[1:] if len(args) > 1 else []

    mapping = get_mapped_command(tool, subcommand)
    if not mapping:
        return None

    # Build the full ROS2 command
    ros2_cmd = [mapping.ros2_command]

    # Split the ROS2 subcommand if it contains multiple parts
    ros2_parts = mapping.ros2_subcommand.split()
    ros2_cmd.extend(ros2_parts)

    # Map the remaining arguments
    mapped_args = mapping.map_arguments(remaining_args)
    ros2_cmd.extend(mapped_args)

    return ros2_cmd[1:]  # Return without 'ros2' prefix as exec.py adds it
