#!/usr/bin/env python3
"""
Unit tests for launch_completion_helper
"""
import os
import tempfile
import time
from pathlib import Path

import pytest

from ros_is_ros2.launch_completion_helper import (
    get_cache_path,
    is_cache_valid,
    parse_launch_args,
    read_cache,
    write_cache,
)


class TestCacheOperations:
    """Test cache-related functions."""

    def test_get_cache_path(self):
        """Test cache path generation."""
        path = get_cache_path("test_package", "test_launch.py")
        assert "test_package" in str(path)
        assert "test_launch_py" in str(path)
        assert path.suffix == ".cache"

    def test_cache_path_sanitization(self):
        """Test that special characters in names are sanitized."""
        path = get_cache_path("my/package", "launch.file.py")
        assert "/" not in path.name
        assert "my_package" in str(path)
        assert "launch_file_py" in str(path)

    def test_write_and_read_cache(self):
        """Test writing and reading cache."""
        with tempfile.TemporaryDirectory() as tmpdir:
            cache_path = Path(tmpdir) / "test.cache"
            test_args = ["arg1", "arg2", "arg3"]

            write_cache(cache_path, test_args)
            assert cache_path.exists()

            read_args = read_cache(cache_path)
            assert read_args == test_args

    def test_is_cache_valid_nonexistent(self):
        """Test cache validation with nonexistent file."""
        cache_path = Path("/nonexistent/path/cache.cache")
        assert not is_cache_valid(cache_path)

    def test_is_cache_valid_fresh(self):
        """Test cache validation with fresh cache."""
        with tempfile.NamedTemporaryFile(delete=False) as tmpfile:
            cache_path = Path(tmpfile.name)
            try:
                assert is_cache_valid(cache_path)
            finally:
                cache_path.unlink()

    def test_is_cache_valid_expired(self):
        """Test cache validation with expired cache."""
        with tempfile.NamedTemporaryFile(delete=False) as tmpfile:
            cache_path = Path(tmpfile.name)
            try:
                # Set modification time to more than TTL in the past
                # Assuming default TTL is 3600 seconds
                old_time = time.time() - 7200  # 2 hours ago
                os.utime(cache_path, (old_time, old_time))
                assert not is_cache_valid(cache_path)
            finally:
                cache_path.unlink()


class TestParseArguments:
    """Test argument parsing from ros2 launch output."""

    def test_parse_single_argument(self):
        """Test parsing output with a single argument."""
        output = """Arguments (pass arguments as '<name>:=<value>'):

    'my_arg':
        description here
        (default: 'value')
"""
        args = parse_launch_args(output)
        assert args == ["my_arg"]

    def test_parse_multiple_arguments(self):
        """Test parsing output with multiple arguments."""
        output = """Arguments (pass arguments as '<name>:=<value>'):

    'camera_name':
        camera unique name
        (default: 'camera')

    'camera_namespace':
        namespace for camera
        (default: 'camera')

    'serial_no':
        choose device by serial number
        (default: '''')
"""
        args = parse_launch_args(output)
        assert args == ["camera_name", "camera_namespace", "serial_no"]

    def test_parse_no_arguments(self):
        """Test parsing output with no arguments."""
        output = """Arguments (pass arguments as '<name>:=<value>'):

"""
        args = parse_launch_args(output)
        assert args == []

    def test_parse_complex_argument_names(self):
        """Test parsing arguments with dots and special characters."""
        output = """Arguments (pass arguments as '<name>:=<value>'):

    'rgb_camera.color_profile':
        color stream profile
        (default: '0,0,0')

    'depth_module.exposure.1':
        first exposure value
        (default: '7500')
"""
        args = parse_launch_args(output)
        assert args == ["rgb_camera.color_profile", "depth_module.exposure.1"]

    def test_parse_empty_string(self):
        """Test parsing empty string."""
        args = parse_launch_args("")
        assert args == []

    def test_parse_invalid_output(self):
        """Test parsing invalid output doesn't crash."""
        output = "Some random text without proper format"
        args = parse_launch_args(output)
        assert args == []


def test_clean_cache_function(monkeypatch):
    """Test the clean_cache function from cli module."""
    from ros_is_ros2.cli import clean_cache as cli_clean_cache

    with tempfile.TemporaryDirectory() as tmpdir:
        # Mock the cache directory
        cache_dir = Path(tmpdir) / "test_cache"
        cache_dir.mkdir()
        (cache_dir / "test_file.cache").write_text("test")

        # Temporarily change HOME to use our temp directory
        monkeypatch.setenv("HOME", tmpdir)

        # Create the cache directory structure
        test_cache = Path(tmpdir) / ".cache" / "ros-is-ros2"
        test_cache.mkdir(parents=True)
        (test_cache / "test.cache").write_text("test")

        assert test_cache.exists()
        cli_clean_cache()
        assert not test_cache.exists()


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
