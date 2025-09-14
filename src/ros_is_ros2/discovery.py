"""Dynamic discovery of ROS2 commands and subcommands."""

import subprocess
import re
import shutil
from typing import List, Dict, Optional, Set
from functools import lru_cache

from .detectors import ensure_ros2_available


class ROS2CommandDiscovery:
    """Discovers ROS2 commands and subcommands dynamically by introspecting ros2."""
    
    def __init__(self):
        self._ros2_available = None
        
    @property
    def ros2_available(self) -> bool:
        """Check if ros2 is available (cached)."""
        if self._ros2_available is None:
            self._ros2_available = ensure_ros2_available()
        return self._ros2_available
    
    @lru_cache(maxsize=32)
    def get_ros2_commands(self) -> List[str]:
        """Get all available ros2 top-level commands."""
        if not self.ros2_available:
            return self._get_fallback_ros2_commands()
            
        try:
            result = subprocess.run(
                ["ros2", "--help"],
                capture_output=True,
                text=True,
                timeout=5
            )
            
            if result.returncode != 0:
                return self._get_fallback_ros2_commands()
                
            # Parse commands from help output
            commands = []
            lines = result.stdout.split('\n')
            in_commands_section = False
            
            for line in lines:
                line = line.strip()
                if 'Commands:' in line or 'available commands:' in line.lower():
                    in_commands_section = True
                    continue
                    
                if in_commands_section:
                    if line == '' or line.startswith('Call '):
                        # End of commands section
                        break
                    
                    # Extract command name (first word on the line)
                    words = line.split()
                    if words and not words[0].startswith('-'):
                        command = words[0]
                        # Filter out non-command entries
                        if command not in ['ros2', 'Run'] and len(command) > 1:
                            commands.append(command)
            
            return commands if commands else self._get_fallback_ros2_commands()
            
        except (subprocess.TimeoutExpired, subprocess.CalledProcessError, FileNotFoundError):
            return self._get_fallback_ros2_commands()
    
    def _get_fallback_ros2_commands(self) -> List[str]:
        """Get fallback list of ros2 commands when discovery fails."""
        return [
            'action', 'bag', 'component', 'daemon', 'doctor', 'interface', 
            'launch', 'lifecycle', 'multicast', 'node', 'param', 'pkg', 
            'run', 'security', 'service', 'topic', 'wtf'
        ]
    
    @lru_cache(maxsize=128)
    def get_ros2_subcommands(self, command: str) -> List[str]:
        """Get all available subcommands for a ros2 command."""
        if not self.ros2_available:
            return self._get_fallback_subcommands(command)
            
        try:
            result = subprocess.run(
                ["ros2", command, "--help"],
                capture_output=True,
                text=True,
                timeout=5
            )
            
            if result.returncode != 0:
                return self._get_fallback_subcommands(command)
                
            # Parse subcommands from help output
            subcommands = []
            lines = result.stdout.split('\n')
            in_commands_section = False
            
            for line in lines:
                line = line.strip()
                
                # Look for different variations of command sections
                if any(keyword in line.lower() for keyword in [
                    'commands:', 'available commands:', 'subcommands:', 'actions:'
                ]):
                    in_commands_section = True
                    continue
                    
                if in_commands_section:
                    if line == '' or line.startswith('Call ') or line.startswith('usage:'):
                        # Check if we've moved to a new section
                        next_section_indicators = ['optional arguments:', 'options:', 'usage:']
                        if any(indicator in line.lower() for indicator in next_section_indicators):
                            break
                        continue
                    
                    # Extract subcommand name (first word on the line)
                    words = line.split()
                    if words and not words[0].startswith('-'):
                        subcommand = words[0]
                        # Filter out non-subcommand entries
                        if len(subcommand) > 0 and subcommand.isalpha():
                            subcommands.append(subcommand)
            
            return subcommands if subcommands else self._get_fallback_subcommands(command)
            
        except (subprocess.TimeoutExpired, subprocess.CalledProcessError, FileNotFoundError):
            return self._get_fallback_subcommands(command)
    
    def _get_fallback_subcommands(self, command: str) -> List[str]:
        """Get fallback subcommands when discovery fails."""
        fallback_subcommands = {
            'topic': ['list', 'echo', 'hz', 'info', 'type', 'find', 'pub', 'bw', 'delay'],
            'node': ['list', 'info'],
            'service': ['list', 'call', 'type', 'find'],
            'param': ['list', 'get', 'set', 'dump', 'load', 'delete', 'describe'],
            'bag': ['record', 'play', 'info', 'reindex', 'convert'],
            'interface': ['list', 'show', 'package', 'packages', 'proto'],
            'launch': [],  # launch takes files, not subcommands
            'run': [],     # run takes package and executable
            'pkg': ['list', 'create', 'executables']
        }
        return fallback_subcommands.get(command, [])
    
    def get_ros1_equivalent(self, ros1_command: str) -> Optional[str]:
        """Map ros1 command (e.g., 'rostopic') to ros2 command (e.g., 'topic')."""
        # Handle special cases first
        special_mappings = {
            'rosmsg': 'interface',
            'rossrv': 'interface'
        }
        
        if ros1_command in special_mappings:
            return special_mappings[ros1_command]
        
        # Extract the part after 'ros'
        if ros1_command.startswith('ros') and len(ros1_command) > 3:
            potential_ros2_cmd = ros1_command[3:]  # Remove 'ros' prefix
            
            # Check if this maps to a valid ros2 command
            available_commands = self.get_ros2_commands()
            
            if potential_ros2_cmd in available_commands:
                return potential_ros2_cmd
                
        return None
    
    def is_valid_ros1_command(self, ros1_command: str) -> bool:
        """Check if a ros1-style command has a valid ros2 equivalent."""
        return self.get_ros1_equivalent(ros1_command) is not None
    
    def get_valid_subcommands(self, ros1_command: str) -> List[str]:
        """Get valid subcommands for a ros1-style command."""
        ros2_cmd = self.get_ros1_equivalent(ros1_command)
        if ros2_cmd:
            return self.get_ros2_subcommands(ros2_cmd)
        return []
    
    @lru_cache(maxsize=256)
    def translate_command(self, ros1_command: str, args: tuple) -> Optional[List[str]]:
        """Translate a complete ros1 command to ros2 equivalent.
        
        Args:
            ros1_command: The ros1 command (e.g., 'rostopic')
            args: Arguments including subcommand (e.g., ('list', '--verbose'))
            
        Returns:
            Translated ros2 command parts (e.g., ['topic', 'list', '--verbose'])
            or None if translation not possible
        """
        ros2_cmd = self.get_ros1_equivalent(ros1_command)
        if not ros2_cmd:
            return None
            
        if not args:
            return [ros2_cmd]
            
        # For most commands, it's a direct translation
        # Just prefix with the ros2 command
        return [ros2_cmd] + list(args)
    
    def clear_cache(self):
        """Clear all cached discovery results."""
        self.get_ros2_commands.cache_clear()
        self.get_ros2_subcommands.cache_clear()
        self.translate_command.cache_clear()


# Global instance for reuse
_discovery = ROS2CommandDiscovery()


def get_discovery() -> ROS2CommandDiscovery:
    """Get the global ROS2CommandDiscovery instance."""
    return _discovery


def discover_ros2_commands() -> List[str]:
    """Get all available ros2 top-level commands."""
    return get_discovery().get_ros2_commands()


def discover_subcommands(command: str) -> List[str]:
    """Get all available subcommands for a ros2 command."""
    return get_discovery().get_ros2_subcommands(command)


def get_valid_ros1_commands() -> List[str]:
    """Get all ros1-style commands that have ros2 equivalents."""
    discovery = get_discovery()
    ros2_commands = discovery.get_ros2_commands()
    return [f"ros{cmd}" for cmd in ros2_commands]


def translate_ros1_to_ros2(ros1_command: str, args: List[str]) -> Optional[List[str]]:
    """Translate ros1 command to ros2 equivalent."""
    return get_discovery().translate_command(ros1_command, tuple(args))


def is_valid_subcommand(ros1_command: str, subcommand: str) -> bool:
    """Check if a subcommand is valid for the given ros1 command."""
    valid_subcommands = get_discovery().get_valid_subcommands(ros1_command)
    return subcommand in valid_subcommands
