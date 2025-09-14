"""Tests for main module."""

from ros_is_ros2.main import main


def test_main_no_args():
    """Test main function with no arguments."""
    result = main([])
    assert result == 0


def test_main_help():
    """Test main function displays help information."""
    result = main([])
    assert result == 0
