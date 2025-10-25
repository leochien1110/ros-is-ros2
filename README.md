# ros-is-ros2

ROS 1-style CLI aliases for ROS 2, with working autocompletion.

> **Note**: Pre-built wheel files are available from [GitHub Releases](https://github.com/leochien1110/ros-is-ros2/releases). PyPI distribution is coming soon.

## Overview

This package provides familiar ROS1 commands (`rostopic`, `rosnode`, etc.) that act as aliases for their ROS2 equivalents (`ros2 topic`, `ros2 node`, etc.). Unlike complex Python wrappers, this project uses a lightweight approach for maximum compatibility and performance:

1.  **Simple Aliases**: Uses native shell aliases (`alias rostopic='ros2 topic'`) instead of Python wrappers. This means there is no performance overhead.
2.  **Complete-Alias**: Leverages the powerful `cykerway/complete-alias` tool (vendored in this project) to provide full, programmable tab completion for all aliased commands.
3.  **Shell Integration**: A simple installation script adds a single line to your shell RC file (`.bashrc` or `.zshrc`) to source the necessary shims.
4.  **Cross-Shell**: Supports both Bash and Zsh with tailored shims for each.

The core architecture is straightforward:

```
ros-is-ros2/
├── cli.py                 # Installation CLI
├── shims/
│   ├── bash.sh           # Bash aliases + completion setup
│   └── zsh.sh            # Zsh functions + completion setup
└── third_party/
    └── complete_alias.bash  # Vendored completion helper
```

## Installation

### Requirements

- ROS2 (Humble, Iron, Jazzy, or Rolling)
- Python 3.8+
- Bash or Zsh shell

**Note**: For bash users, `bash-completion` will be automatically installed during setup for an optimal tab completion experience.

### From GitHub Releases (Recommended)
Download the latest wheel file from [GitHub Releases](https://github.com/leochien1110/ros-is-ros2/releases):

```bash
# Download ros_is_ros2-X.X.X-py3-none-any.whl from releases, then:
pip install ros_is_ros2-0.4.0-py3-none-any.whl
ros-is-ros2 install    # Automatically installs bash-completion if needed
source ~/.bashrc       # or ~/.zshrc
```

### From PyPI (when published)
```bash
pip install ros-is-ros2
ros-is-ros2 install    # Automatically installs bash-completion if needed
source ~/.bashrc       # or ~/.zshrc
```

### From Source/Local Development
#### Prerequisites for Source Install

- Git (to clone the repository)
- Python 3.8+ with pip
- Build tools (automatically installed when needed)

#### Installation Methods
1.  **Direct Install (Recommended)**
    ```bash
    git clone https://github.com/leochien1110/ros-is-ros2.git
    cd ros-is-ros2
    pip install .
    ros-is-ros2 install
    source ~/.bashrc       # or ~/.zshrc
    ```

2.  **Build and Install Wheel**
    ```bash
    git clone https://github.com/leochien1110/ros-is-ros2.git
    cd ros-is-ros2
    pip install build
    python -m build
    pip install dist/ros_is_ros2-*.whl
    ros-is-ros2 install
    source ~/.bashrc       # or ~/.zshrc
    ```

3.  **Development Install (Editable)**
    For developers who want to make changes:
    ```bash
    git clone https://github.com/leochien1110/ros-is-ros2.git
    cd ros-is-ros2
    python -m pip install -e .   # Editable install (use python -m pip for better compatibility)
    ros-is-ros2 install
    source ~/.bashrc             # or ~/.zshrc

    # Test editable behavior - changes to source code are immediately reflected
    ros-is-ros2 --help           # Make changes to cli.py and run again to see updates
    ```

4.  **One-Line Install (from source)**
    ```bash
    git clone https://github.com/leochien1110/ros-is-ros2.git && cd ros-is-ros2 && pip install . && ros-is-ros2 install && source ~/.bashrc
    ```

### Verifying Installation

After installing from source, verify everything works:
```bash
# Check the command is available
ros-is-ros2 --help

# Check version
pip list | grep ros-is-ros2

# Test the full workflow
ros-is-ros2 install
source ~/.bashrc
rostopic --help    # Should show ROS2 topic help
```

## Updating the Package

If you have an older version installed (e.g., without new features like `rosdomainid` for managing ROS_DOMAIN_ID or `rosdepinstall` for quick dependency installation), follow these steps to update. This ensures the new macros take effect.

### For Editable Installs (Recommended for Development)
If you installed with `pip install -e .` (common for local development):

1. **Navigate to the project directory**:
   ```bash
   cd /path/to/ros-is-ros2  # e.g., /home/leo/git/ros-is-ros2
   ```

2. **Pull the latest changes** (if using Git):
   ```bash
   git pull origin main
   ```

3. **Reinstall the editable package** (updates the shims with new features):
   ```bash
   python -m pip install -e .
   ```

4. **Reload the shims in your current terminal** (without relaunching):
   ```bash
   # Option 1: Full reload (safest, reloads everything)
   source ~/.bashrc  # or ~/.zshrc for Zsh

   # Option 2: Direct reload of shims only (faster, avoids re-running other setup)
   ros-is-ros2 print-path | xargs source
   ```

5. **Verify the new features**:
   ```bash
   rosdomainid  # Should output "unset" or current value
   rosdepinstall --help  # Should error if not in workspace, but command exists
   ```

### For Non-Editable Installs (Regular pip install)
If you installed with `pip install .` or from a wheel:

1. **Update the source code** (if from Git):
   ```bash
   cd /path/to/ros-is-ros2
   git pull origin main
   ```

2. **Reinstall the package**:
   ```bash
   pip install .  # Or build and install wheel: python -m build && pip install dist/ros_is_ros2-*.whl
   ```

3. **Reload as above** (source ~/.bashrc or use ros-is-ros2 print-path).

### For PyPI Installs (When Available)
```bash
pip install --upgrade ros-is-ros2
ros-is-ros2 install  # Re-run to ensure shims are up-to-date
source ~/.bashrc     # Reload
```

### Notes
- **No Terminal Relaunch Needed**: Using `source` in your current session applies changes immediately.
- **If Shims Not Installed**: Run `ros-is-ros2 install` first to add the sourcing line to your shell RC.
- **Troubleshooting**: If functions don't appear, check `ros-is-ros2 print-path` outputs the correct shim path, and ensure it's sourced correctly in your RC file.

## Usage

After installation, use ROS1 commands as you normally would. All commands support full tab completion for subcommands, topics, nodes, packages, and executables.

### Supported Commands

- `rostopic` → `ros2 topic`
- `rosnode` → `ros2 node`
- `rosservice` → `ros2 service`
- `rosparam` → `ros2 param`
- `rosbag` → `ros2 bag`
- `rosrun` → `ros2 run`
- `roslaunch` → `ros2 launch`
- `rospack` → `ros2 pkg`
- `rossrv` → `ros2 interface`
- `rosmsg` → `ros2 interface`

### Examples

```bash
rostopic list<TAB>        # Tab completion for subcommands
rostopic echo /chatter<TAB>  # Tab completion for topics
rosnode list
rosrun demo_nodes_cpp talker<TAB>  # Tab completion for packages/executables
roslaunch realsense2_camera rs_launch.py <TAB>  # Tab completion for launch arguments
roslaunch realsense2_camera rs_launch.py cam<TAB>  # Partial completion for arguments
```

### Launch Argument Tab Completion

Tab-complete launch file arguments just like in ROS1:

```bash
roslaunch realsense2_camera rs_launch.py <TAB>
# Shows: camera_name:= camera_namespace:= serial_no:= ...

roslaunch realsense2_camera rs_launch.py cam<TAB>
# Shows: camera_name:= camera_namespace:=

roslaunch realsense2_camera rs_launch.py camera_name:=cam1 <TAB>
# Shows remaining arguments (filters out already-provided)
```

Features:
- Works with system packages and custom workspaces
- Supports Python, XML, and YAML launch files
- First completion ~1-2s, subsequent instant (cached for 1 hour)
- Configure via `ROS_IS_ROS2_CACHE_TTL` and `ROS_IS_ROS2_COMPLETION_TIMEOUT` env vars

### New Macros (v0.2.0+)
- `rosdomainid` [num]: Check or set ROS_DOMAIN_ID (e.g., `rosdomainid 42` sets it to 42; `rosdomainid` shows current).
- `rosdepinstall`: Run `rosdep update && rosdep install --from-paths src --ignore-src -r -y` in one command.

## Support

### Uninstallation

```bash
ros-is-ros2 uninstall  # Removes shell integration and cleans cache
pip uninstall ros-is-ros2
source ~/.bashrc    # or ~/.zshrc
```

The `uninstall` command will:
- Remove the sourcing line from your shell RC file
- Clean up the cache directory (`~/.cache/ros-is-ros2/`)

### Troubleshooting Local Install

<details>
<summary><strong>🔧 Quick Fix for Most Issues</strong></summary>

**TL;DR**: If `pip install -e .` fails, try this first:
```bash
pip install --user --upgrade pip
python -m pip install -e .
```

**Why**: Most editable install failures are caused by old pip versions that don't work well with modern setuptools.
</details>

<details>
<summary><strong>🔒 Permission Issues</strong></summary>

If you get permission errors during pip install:
```bash
pip install --user .    # Install to user directory
```
</details>

<details>
<summary><strong>📦 Build Dependencies Missing</strong></summary>

If the build step fails, ensure you have the build package:
```bash
pip install build wheel setuptools
```
</details>

<details>
<summary><strong>⚠️ Editable Install Failures (`pip install -e .`)</strong></summary>

**Common Issue**: Version mismatch between pip and setuptools causing editable install to fail.

**Symptoms**:
```
WARNING: The user site-packages directory is disabled.
error: can't create or remove files in install directory
[Errno 13] Permission denied: '/usr/local/lib/python3.10/dist-packages/...'
```

**Root Cause**: Your system has old pip (often from Ubuntu/Debian packages) but newer setuptools, causing incompatibility.

**Check your versions**:
```bash
pip --version                # Often shows old version like 22.0.2
python -c "import setuptools; print('Setuptools:', setuptools.__version__)"
```

**Solution 1: Upgrade pip (Recommended)**:
```bash
# Upgrade pip to user space
pip install --user --upgrade pip

# Now use the newer pip for editable installs
python -m pip install -e .
```

**Solution 2: Alternative methods if editable install still fails**:
```bash
# Method A: Direct install (need to reinstall after changes)
pip install .

# Method B: Build method
python -m build && pip install dist/ros_is_ros2-*.whl
```
</details>

<details>
<summary><strong>🤖 ROS Environment Issues</strong></summary>

If you're in a ROS environment (ROS_DISTRO set), it can interfere with Python packaging:

**Check if in ROS environment**:
```bash
echo $ROS_DISTRO    # Shows ROS version if set
```

**Solutions**:
```bash
# Always use python -m pip instead of pip
python -m pip install -e .

# Or temporarily disable ROS environment
unset ROS_DISTRO
pip install -e .
```
</details>

<details>
<summary><strong>🌐 Virtual Environment Issues</strong></summary>

If virtual environment creation fails:
```bash
# Ubuntu/Debian: Install venv package
sudo apt install python3.10-venv

# Then create and use virtual environment
python -m venv venv
source venv/bin/activate
pip install -e .
```
</details>

## Development

### Setting Up Development Environment

For contributors and developers working on `ros-is-ros2`:

1.  **Clone and install in development mode**:
    ```bash
    git clone https://github.com/leochien1110/ros-is-ros2.git
    cd ros-is-ros2
    pip install -e .
    ```

2.  **Install development dependencies**:
    ```bash
    pip install -r requirements-dev.txt
    # OR
    pip install -e .[dev]
    ```

3.  **Set up pre-commit hooks** (recommended):
    ```bash
    pre-commit install
    ```

### Pre-commit Hooks

This project uses pre-commit hooks to ensure code quality and run tests automatically before each commit. The hooks include:

- **Code Formatting**: `black` and `isort` for Python code formatting
- **Linting**: `flake8` for Python linting and `shellcheck` for shell scripts
- **Testing**: `pytest` runs the full test suite
- **General**: Trailing whitespace removal, YAML validation, etc.

**Manual execution**:
```bash
# Run all hooks on all files
pre-commit run --all-files

# Run specific hook
pre-commit run black
pre-commit run pytest
```

**What happens during commit**:
- All staged files are automatically formatted and linted
- Tests are run to ensure nothing is broken
- If any check fails, the commit is blocked
- Fix the issues and commit again

### Running Tests

```bash
# Run all tests
pytest

# Run with coverage
pytest --cov=src/ros_is_ros2

# Run specific test file
pytest tests/unit/test_cli.py
```

### Project Structure

```
ros-is-ros2/
├── src/ros_is_ros2/           # Main package
│   ├── cli.py                 # Installation CLI
│   ├── shims/                 # Shell integration scripts
│   │   ├── bash.sh           # Bash aliases + completion
│   │   └── zsh.sh            # Zsh functions + completion
│   └── third_party/          # Vendored dependencies
│       └── complete_alias.bash
├── tests/                     # Test suite
│   ├── unit/                 # Unit tests
│   └── integration/          # Integration tests
├── .pre-commit-config.yaml   # Pre-commit configuration
├── requirements-dev.txt      # Development dependencies
└── pyproject.toml           # Project configuration
```

## License and Attribution

This project is licensed under the **GNU General Public License v3.0 or later** (GPL-3.0-or-later).

### Third-Party Components

This project includes the following third-party software:

- **complete-alias** by Cyker Way
  - **License**: GNU General Public License v3.0
  - **Copyright**: Copyright (C) 2016-2021 Cyker Way
  - **Source**: https://github.com/cykerway/complete-alias
  - **Location**: `src/ros_is_ros2/third_party/complete_alias.bash`
  - **Purpose**: Provides bash completion for aliased commands

### License Summary

- ✅ **Use**: Free for personal, academic, and commercial use
- ✅ **Modify**: You can modify and distribute modifications
- ✅ **Distribute**: You can distribute original and modified versions
- ⚠️ **Copyleft**: Derivative works must also be GPL v3.0 compatible
- ⚠️ **Source Code**: Must provide source code when distributing

See the [LICENSE](LICENSE) file for the full license text.

### Original Method w/o Python
This idea can be simply achieve via alias and bash-completion. If you don't want to install this package, you can simply add the following to your shell RC file(`~/.bashrc` or `~/.zshrc`):
```bash
##### ROS 2 CLI Aliases #####
# Make sure to run `sudo apt install bash-completion`
# load complete-alias
# [ -f "$HOME/.complete_alias" ] && . "$HOME/.complete_alias"

# eval "$(register-python-argcomplete ros2)"

# # ROS1-like aliases
# alias rostopic='ros2 topic'
# alias rosnode='ros2 node'
# alias rosservice='ros2 service'
# alias rosparam='ros2 param'
# alias rosbag='ros2 bag'
# alias rosrun='ros2 run'
# alias roslaunch='ros2 launch'
# alias rospack='ros2 pkg'

# # give these aliases the same completion behavior as the expanded command
# complete -F _complete_alias rostopic
# complete -F _complete_alias rosnode
# complete -F _complete_alias rosservice
# complete -F _complete_alias rosparam
# complete -F _complete_alias rosbag
# complete -F _complete_alias rosrun
# complete -F _complete_alias roslaunch
# complete -F _complete_alias rospack
```
