"""Tests for completion module."""

from ros_is_ros2.completion import ROS2CompletionDelegate, SubcommandCompleter


def test_subcommand_completer():
    """Test subcommand completion."""
    completer = SubcommandCompleter(["list", "echo", "hz", "info"])

    # Test complete matches
    assert "list" in completer("l")
    assert "echo" in completer("e")
    assert "hz" in completer("h")

    # Test exact matches
    assert completer("list") == ["list"]
    assert completer("echo") == ["echo"]

    # Test no matches
    assert completer("xyz") == []

    # Test empty prefix
    result = completer("")
    assert len(result) == 4
    assert "list" in result
    assert "echo" in result


def test_ros2_completion_delegate():
    """Test ROS2 completion delegate."""
    delegate = ROS2CompletionDelegate("topic")

    # Create mock parsed args
    class MockArgs:
        def __init__(self):
            self.subcommand = "echo"
            self.args = []

    mock_args = MockArgs()

    # Test that completion function doesn't crash
    # (actual completion depends on ROS2 environment)
    result = delegate.complete("/", mock_args)
    assert isinstance(result, list)


def test_completion_import():
    """Test that completion module imports correctly."""
    from ros_is_ros2.completion import (
        setup_rostopic_completion,
    )

    # Test that setup function returns a boolean
    assert isinstance(setup_rostopic_completion(), bool)
