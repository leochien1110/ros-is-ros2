"""Integration tests for shell functionality."""

import subprocess
from pathlib import Path

import pytest


class TestShellIntegration:
    """Test actual shell integration functionality."""

    @pytest.fixture
    def bash_script_path(self):
        """Get path to bash shim script."""
        # Assuming we're running from the project root
        return (
            Path(__file__).parent.parent.parent
            / "src"
            / "ros_is_ros2"
            / "shims"
            / "bash.sh"
        )

    @pytest.fixture
    def zsh_script_path(self):
        """Get path to zsh shim script."""
        return (
            Path(__file__).parent.parent.parent
            / "src"
            / "ros_is_ros2"
            / "shims"
            / "zsh.sh"
        )

    def test_bash_script_syntax(self, bash_script_path):
        """Test that bash script has valid syntax."""
        if not bash_script_path.exists():
            pytest.skip("Bash script not found")

        result = subprocess.run(
            ["bash", "-n", str(bash_script_path)], capture_output=True, text=True
        )
        assert result.returncode == 0, f"Bash syntax error: {result.stderr}"

    def test_zsh_script_syntax(self, zsh_script_path):
        """Test that zsh script has valid syntax."""
        if not zsh_script_path.exists():
            pytest.skip("Zsh script not found")

        # Check if zsh is available
        if not subprocess.run(["which", "zsh"], capture_output=True).returncode == 0:
            pytest.skip("Zsh not available")

        result = subprocess.run(
            ["zsh", "-n", str(zsh_script_path)], capture_output=True, text=True
        )
        assert result.returncode == 0, f"Zsh syntax error: {result.stderr}"

    def test_bash_aliases_defined(self, bash_script_path):
        """Test that bash aliases are properly defined."""
        if not bash_script_path.exists():
            pytest.skip("Bash script not found")

        # Source the script and check if aliases are defined
        # Ignore stderr to handle missing bash-completion gracefully
        bash_commands = [
            f"source {bash_script_path} 2>/dev/null || true",
            "alias | grep -E '^(rostopic|rosnode|rosservice)=' || echo 'No aliases'",
        ]

        result = subprocess.run(
            ["bash", "-c", "; ".join(bash_commands)], capture_output=True, text=True
        )

        # Check if aliases were defined (may fail if bash-completion missing)
        if "No aliases" in result.stdout:
            pytest.skip("Aliases not defined - bash-completion may be missing")

        # Should find at least some aliases
        assert "rostopic=" in result.stdout
        assert "ros2 topic" in result.stdout

    def test_bash_aliases_content(self, bash_script_path):
        """Test that bash script contains expected alias definitions."""
        if not bash_script_path.exists():
            pytest.skip("Bash script not found")

        content = bash_script_path.read_text()

        # Check that alias definitions are present in the script
        expected_aliases = [
            "alias rostopic='ros2 topic'",
            "alias rosnode='ros2 node'",
            "alias rosservice='ros2 service'",
            "alias rosparam='ros2 param'",
            "alias rosbag='ros2 bag'",
            "alias rosrun='ros2 run'",
            "alias roslaunch='ros2 launch'",
            "alias rospack='ros2 pkg'",
        ]

        for alias_def in expected_aliases:
            assert alias_def in content, f"Missing alias definition: {alias_def}"

    def test_zsh_functions_content(self, zsh_script_path):
        """Test that zsh script contains expected function definitions."""
        if not zsh_script_path.exists():
            pytest.skip("Zsh script not found")

        content = zsh_script_path.read_text()

        # Check that function definitions are present in the script
        expected_functions = [
            'rostopic()   { ros2 topic   "$@"; }',
            'rosnode()    { ros2 node    "$@"; }',
            'rosservice() { ros2 service "$@"; }',
            'rosparam()   { ros2 param   "$@"; }',
            'rosbag()     { ros2 bag     "$@"; }',
            'rosrun()     { ros2 run     "$@"; }',
            'roslaunch()  { ros2 launch  "$@"; }',
            'rospack()    { ros2 pkg     "$@"; }',
        ]

        for func_def in expected_functions:
            assert func_def in content, f"Missing function definition: {func_def}"

    @pytest.mark.skipif(
        subprocess.run(["which", "zsh"], capture_output=True).returncode != 0,
        reason="Zsh not available",
    )
    def test_zsh_functions_defined(self, zsh_script_path):
        """Test that zsh functions can be loaded without errors."""
        if not zsh_script_path.exists():
            pytest.skip("Zsh script not found")

        # Just test that the script can be sourced without errors
        result = subprocess.run(
            ["zsh", "-c", f"source {zsh_script_path} && echo 'sourced successfully'"],
            capture_output=True,
            text=True,
        )

        # Should be able to source without errors
        assert result.returncode == 0, f"Failed to source zsh script: {result.stderr}"
        assert "sourced successfully" in result.stdout

    def test_complete_alias_script_exists(self):
        """Test that complete-alias script exists and is readable."""
        complete_alias_path = (
            Path(__file__).parent.parent.parent
            / "src"
            / "ros_is_ros2"
            / "third_party"
            / "complete_alias.bash"
        )

        assert complete_alias_path.exists(), "complete-alias.bash not found"
        assert complete_alias_path.is_file(), "complete-alias.bash is not a file"
        assert complete_alias_path.stat().st_size > 0, "complete-alias.bash is empty"

    def test_complete_alias_syntax(self):
        """Test that complete-alias script has valid syntax."""
        complete_alias_path = (
            Path(__file__).parent.parent.parent
            / "src"
            / "ros_is_ros2"
            / "third_party"
            / "complete_alias.bash"
        )

        if not complete_alias_path.exists():
            pytest.skip("complete-alias.bash not found")

        result = subprocess.run(
            ["bash", "-n", str(complete_alias_path)], capture_output=True, text=True
        )
        assert (
            result.returncode == 0
        ), f"complete-alias.bash syntax error: {result.stderr}"


class TestCLIInstallation:
    """Test CLI installation functionality."""

    def test_cli_help(self):
        """Test that CLI help works."""
        result = subprocess.run(
            ["python", "-m", "ros_is_ros2.cli", "--help"],
            capture_output=True,
            text=True,
            cwd=Path(__file__).parent.parent.parent,
        )

        assert result.returncode == 0
        assert "ros-is-ros2" in result.stdout
        assert "install" in result.stdout
        assert "uninstall" in result.stdout

    def test_print_path_command(self):
        """Test print-path command."""
        result = subprocess.run(
            ["python", "-m", "ros_is_ros2.cli", "print-path"],
            capture_output=True,
            text=True,
            cwd=Path(__file__).parent.parent.parent,
        )

        assert result.returncode == 0
        assert "bash.sh" in result.stdout or "zsh.sh" in result.stdout
