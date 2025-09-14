"""Tests for mappings module."""

from ros_is_ros2.mappings import get_mapped_command, map_ros1_to_ros2


def test_rostopic_list_mapping():
    """Test rostopic list maps to ros2 topic list."""
    result = map_ros1_to_ros2("rostopic", ["list"])
    assert result == ["topic", "list"]


def test_rostopic_echo_mapping():
    """Test rostopic echo maps to ros2 topic echo."""
    result = map_ros1_to_ros2("rostopic", ["echo", "/chatter"])
    assert result == ["topic", "echo", "/chatter"]


def test_rostopic_hz_mapping():
    """Test rostopic hz maps to ros2 topic hz."""
    result = map_ros1_to_ros2("rostopic", ["hz", "/chatter"])
    assert result == ["topic", "hz", "/chatter"]


def test_rostopic_info_mapping():
    """Test rostopic info maps to ros2 topic info."""
    result = map_ros1_to_ros2("rostopic", ["info", "/chatter"])
    assert result == ["topic", "info", "/chatter"]


def test_invalid_tool():
    """Test invalid tool returns None."""
    result = map_ros1_to_ros2("invalid_tool", ["list"])
    assert result is None


def test_invalid_subcommand():
    """Test invalid subcommand returns None."""
    result = map_ros1_to_ros2("rostopic", ["invalid"])
    assert result is None


def test_get_mapped_command():
    """Test get_mapped_command function."""
    mapping = get_mapped_command("rostopic", "list")
    assert mapping is not None
    assert mapping.ros2_command == "ros2"
    assert mapping.ros2_subcommand == "topic list"
