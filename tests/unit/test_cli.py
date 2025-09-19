"""Tests for CLI module."""

import os
import tempfile
from pathlib import Path
from unittest.mock import patch

import pytest

from ros_is_ros2.cli import (
    BLOCK_BEGIN,
    BLOCK_END,
    add_block,
    detect_package_manager,
    detect_shell,
    is_bash_completion_installed,
    main,
    rc_file,
    remove_block,
    shim_path,
)


class TestRcFile:
    """Test RC file detection."""

    def test_rc_file_bash(self):
        """Test RC file for bash."""
        result = rc_file("/bin/bash")
        assert result.name == ".bashrc"

    def test_rc_file_zsh(self):
        """Test RC file for zsh."""
        result = rc_file("/usr/bin/zsh")
        assert result.name == ".zshrc"


class TestShimPath:
    """Test shim path detection."""

    def test_shim_path_bash(self):
        """Test shim path for bash."""
        result = shim_path("/bin/bash")
        assert result.name == "bash.sh"

    def test_shim_path_zsh(self):
        """Test shim path for zsh."""
        result = shim_path("/usr/bin/zsh")
        assert result.name == "zsh.sh"


class TestDetectShell:
    """Test shell detection."""

    @patch.dict(os.environ, {"SHELL": "/bin/bash"})
    def test_detect_shell_from_env(self):
        """Test shell detection from environment."""
        result = detect_shell()
        assert result == "/bin/bash"

    @patch.dict(os.environ, {}, clear=True)
    @patch("ros_is_ros2.cli.shutil.which")
    def test_detect_shell_fallback(self, mock_which):
        """Test shell detection fallback."""
        mock_which.return_value = "/usr/bin/bash"
        result = detect_shell()
        assert result == "/usr/bin/bash"


class TestBlockManagement:
    """Test RC file block management."""

    def test_add_block_new_file(self):
        """Test adding block to new file."""
        with tempfile.NamedTemporaryFile(mode="w", delete=False) as tmp:
            tmp_path = Path(tmp.name)

        try:
            add_block(tmp_path, "/bin/bash", Path("/test/shim.sh"))
            content = tmp_path.read_text()

            assert BLOCK_BEGIN in content
            assert BLOCK_END in content
            assert '. "/test/shim.sh"' in content

        finally:
            tmp_path.unlink()

    def test_add_block_existing_file(self):
        """Test adding block to existing file."""
        with tempfile.NamedTemporaryFile(mode="w", delete=False) as tmp:
            tmp.write("existing content\n")
            tmp_path = Path(tmp.name)

        try:
            add_block(tmp_path, "/bin/bash", Path("/test/shim.sh"))
            content = tmp_path.read_text()

            assert "existing content" in content
            assert BLOCK_BEGIN in content
            assert BLOCK_END in content

        finally:
            tmp_path.unlink()

    def test_add_block_idempotent(self):
        """Test that adding block twice is idempotent."""
        with tempfile.NamedTemporaryFile(mode="w", delete=False) as tmp:
            tmp_path = Path(tmp.name)

        try:
            # Add block twice
            add_block(tmp_path, "/bin/bash", Path("/test/shim.sh"))
            add_block(tmp_path, "/bin/bash", Path("/test/shim.sh"))

            content = tmp_path.read_text()
            # Should only appear once
            assert content.count(BLOCK_BEGIN) == 1
            assert content.count(BLOCK_END) == 1

        finally:
            tmp_path.unlink()

    def test_remove_block(self):
        """Test removing block from file."""
        with tempfile.NamedTemporaryFile(mode="w", delete=False) as tmp:
            content = f"before\n{BLOCK_BEGIN}\nblock content\n{BLOCK_END}\nafter\n"
            tmp.write(content)
            tmp_path = Path(tmp.name)

        try:
            remove_block(tmp_path)
            result = tmp_path.read_text()

            assert BLOCK_BEGIN not in result
            assert BLOCK_END not in result
            assert "before" in result
            assert "after" in result

        finally:
            tmp_path.unlink()

    def test_remove_block_nonexistent_file(self):
        """Test removing block from nonexistent file."""
        # Should not raise error
        remove_block(Path("/nonexistent/file"))


class TestBashCompletionDetection:
    """Test bash completion detection."""

    @patch("ros_is_ros2.cli.Path.exists")
    def test_is_bash_completion_installed_true(self, mock_exists):
        """Test detecting installed bash completion."""
        mock_exists.return_value = True
        assert is_bash_completion_installed() is True

    @patch("ros_is_ros2.cli.subprocess.run")
    @patch("ros_is_ros2.cli.Path.exists")
    def test_is_bash_completion_installed_false(self, mock_exists, mock_run):
        """Test detecting missing bash completion."""
        mock_exists.return_value = False
        mock_run.side_effect = FileNotFoundError()
        assert is_bash_completion_installed() is False


class TestPackageManagerDetection:
    """Test package manager detection."""

    @patch("ros_is_ros2.cli.shutil.which")
    def test_detect_package_manager_apt(self, mock_which):
        """Test detecting apt package manager."""
        mock_which.side_effect = lambda cmd: "/usr/bin/apt" if cmd == "apt" else None
        assert detect_package_manager() == "apt"

    @patch("ros_is_ros2.cli.shutil.which")
    def test_detect_package_manager_brew(self, mock_which):
        """Test detecting brew package manager."""
        mock_which.side_effect = lambda cmd: (
            "/usr/local/bin/brew" if cmd == "brew" else None
        )
        assert detect_package_manager() == "brew"

    @patch("ros_is_ros2.cli.shutil.which")
    def test_detect_package_manager_none(self, mock_which):
        """Test when no package manager is found."""
        mock_which.return_value = None
        assert detect_package_manager() is None


class TestMainCLI:
    """Test main CLI functionality."""

    @patch("ros_is_ros2.cli.add_block")
    @patch("ros_is_ros2.cli.install_bash_completion")
    def test_main_install(self, mock_install_bash, mock_add_block):
        """Test main install command."""
        mock_install_bash.return_value = True

        with patch("sys.argv", ["ros-is-ros2", "install"]):
            result = main()
            assert result == 0
            mock_add_block.assert_called_once()

    @patch("ros_is_ros2.cli.remove_block")
    def test_main_uninstall(self, mock_remove_block):
        """Test main uninstall command."""
        with patch("sys.argv", ["ros-is-ros2", "uninstall"]):
            result = main()
            assert result == 0
            mock_remove_block.assert_called_once()

    @patch("ros_is_ros2.cli.shim_path")
    def test_main_print_path(self, mock_shim_path):
        """Test main print-path command."""
        mock_shim_path.return_value = Path("/test/path")

        with patch("sys.argv", ["ros-is-ros2", "print-path"]):
            with patch("builtins.print") as mock_print:
                result = main()
                assert result == 0
                mock_print.assert_called_once()

    def test_main_no_command(self):
        """Test main with no command shows help."""
        with patch("sys.argv", ["ros-is-ros2"]):
            with pytest.raises(SystemExit):
                main()
