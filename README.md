# ros-is-ros2

ROS 1-style CLI aliases for ROS 2, with working autocompletion.

## Overview

This package provides familiar ROS1 commands (`rostopic`, `rosnode`, etc.) that act as aliases around their ROS2 equivalents (`ros2 topic`, `ros2 node`, etc.). Unlike complex Python wrappers, this uses simple shell aliases with the `complete-alias` tool to provide full tab completion.

> **Note**: This package is currently available for installation from source only. PyPI distribution is coming soon.

## Installation

### From PyPI (when published)
```bash
pip install ros-is-ros2
ros-is-ros2 install    # Automatically installs bash-completion if needed
source ~/.bashrc       # or ~/.zshrc
```

### From Source/Local Development

#### Option 1: Direct Install (Recommended)
```bash
git clone https://github.com/leo/ros-is-ros2.git
cd ros-is-ros2
pip install .
ros-is-ros2 install
source ~/.bashrc       # or ~/.zshrc
```

#### Option 2: Build and Install Wheel
```bash
git clone https://github.com/leo/ros-is-ros2.git
cd ros-is-ros2
pip install build
python -m build
pip install dist/ros_is_ros2-*.whl
ros-is-ros2 install
source ~/.bashrc       # or ~/.zshrc
```

#### Option 3: Development Install (Editable)
For developers who want to make changes:
```bash
git clone https://github.com/leo/ros-is-ros2.git
cd ros-is-ros2
python -m pip install -e .   # Editable install (use python -m pip for better compatibility)
ros-is-ros2 install
source ~/.bashrc             # or ~/.zshrc

# Test editable behavior - changes to source code are immediately reflected
ros-is-ros2 --help           # Make changes to cli.py and run again to see updates
```

### One-Line Install (from source)
```bash
git clone https://github.com/leo/ros-is-ros2.git && cd ros-is-ros2 && pip install . && ros-is-ros2 install && source ~/.bashrc
```

### Prerequisites for Source Install

- Git (to clone the repository)
- Python 3.8+ with pip
- Build tools (automatically installed when needed)

### Troubleshooting Local Install

#### Quick Fix for Most Issues

**TL;DR**: If `pip install -e .` fails, try this first:
```bash
pip install --user --upgrade pip
python -m pip install -e .
```

**Why**: Most editable install failures are caused by old pip versions that don't work well with modern setuptools.

---

#### Permission Issues
If you get permission errors during pip install:
```bash
pip install --user .    # Install to user directory
```

#### Build Dependencies Missing
If the build step fails, ensure you have the build package:
```bash
pip install build wheel setuptools
```

#### Editable Install Failures (`pip install -e .`)

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

#### ROS Environment Issues
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

#### Virtual Environment Issues
If virtual environment creation fails:
```bash
# Ubuntu/Debian: Install venv package
sudo apt install python3.10-venv

# Then create and use virtual environment
python -m venv venv
source venv/bin/activate
pip install -e .
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

### Why Two-Stage Installation?

The installation process requires two steps because:

1. **Shell Configuration**: The tool needs to add a source line to your shell configuration file (`.bashrc` or `.zshrc`)
2. **User Consent**: Automatically modifying configuration files during `pip install` can be surprising and problematic
3. **Environment Safety**: Keeps the installation safe for CI, containers, and automated environments
4. **Clean Uninstall**: Provides a clean way to remove the integration with `ros-is-ros2 uninstall`
5. **Dependency Installation**: Automatically installs `bash-completion` if needed (bash users only)

The `ros-is-ros2 install` command:
- Detects your system and installs `bash-completion` if needed (Ubuntu/Debian, CentOS/RHEL/Fedora, macOS supported)
- Safely adds this block to your shell config:
```bash
# >>> ros-is-ros2 >>>
# ROS 1-style shims for ROS 2
source "/path/to/ros-is-ros2/shims/bash.sh"
# <<< ros-is-ros2 <<<
```

## Supported Commands

All commands support full tab completion:

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

## Usage

After installation, use ROS1 commands as you normally would:

```bash
rostopic list<TAB>        # Tab completion for subcommands
rostopic echo /chatter<TAB>  # Tab completion for topics
rosnode list
rosrun demo_nodes_cpp talker<TAB>  # Tab completion for packages/executables
```

Each command is a simple alias that forwards to the appropriate ROS2 equivalent with full completion support.

## Tab Completion Examples

```bash
# Topic completion
rostopic echo <TAB>
rostopic hz <TAB>

# Node completion
rosnode info <TAB>

# Package/executable completion
rosrun <TAB>
rosrun demo_nodes_cpp <TAB>
```

## Uninstallation

```bash
ros-is-ros2 uninstall
source ~/.bashrc    # or ~/.zshrc
```

## Requirements

- ROS2 (Humble, Iron, Jazzy, or Rolling)
- Python 3.8+
- Bash or Zsh shell

**Note**: For bash users, `bash-completion` will be automatically installed during setup for optimal tab completion experience.

## How It Works

1. **Simple Aliases**: Uses shell aliases instead of Python wrappers
2. **Complete-Alias**: Leverages the `cykerway/complete-alias` tool for proper completion
3. **Shell Integration**: Adds a single line to your shell RC file
4. **Cross-Shell**: Supports both Bash and Zsh

## Architecture

```
ros-is-ros2/
├── cli.py                 # Installation CLI
├── shims/
│   ├── bash.sh           # Bash aliases + completion setup
│   └── zsh.sh            # Zsh functions + completion setup
└── third_party/
    └── complete_alias.bash  # Vendored completion helper
```

## Development Status

This is a complete rewrite using shell aliases instead of Python wrappers. The approach is much simpler and more reliable than the previous implementation.

## Publishing to PyPI

This section is for maintainers who want to publish new versions of the package to PyPI.

### Prerequisites

1. **PyPI Account**: Create accounts on both [Test PyPI](https://test.pypi.org/account/register/) and [PyPI](https://pypi.org/account/register/)
2. **API Tokens**: Generate API tokens for both repositories:
   - [Test PyPI tokens](https://test.pypi.org/manage/account/#api-tokens)
   - [PyPI tokens](https://pypi.org/manage/account/#api-tokens)
3. **Build Tools**: Install required build tools:
   ```bash
   pip install --upgrade build twine
   ```

### Publishing Process

#### 1. Update Version
Edit the version in `pyproject.toml`:
```toml
[project]
version = "0.3.0"  # Update to your new version
```

#### 2. Build the Package
```bash
# Clean any previous builds
rm -rf dist/ build/ *.egg-info

# Build the package
python -m build
```

This creates both `.tar.gz` (source) and `.whl` (wheel) files in the `dist/` directory.

#### 3. Test on Test PyPI (Recommended)
```bash
# Upload to Test PyPI first
python -m twine upload --repository testpypi dist/*

# Test installation from Test PyPI
pip install --index-url https://test.pypi.org/simple/ --extra-index-url https://pypi.org/simple/ ros-is-ros2
```

#### 4. Publish to PyPI
Once tested successfully:
```bash
# Upload to PyPI
python -m twine upload dist/*
```

#### 5. Create Git Tag
```bash
git tag v0.3.0  # Match the version number
git push origin v0.3.0
```

### Authentication Setup

Configure your credentials using one of these methods:

**Option 1: Using API Tokens (Recommended)**
```bash
# Create ~/.pypirc file
cat > ~/.pypirc << EOF
[distutils]
index-servers = pypi testpypi

[pypi]
username = __token__
password = pypi-YOUR_API_TOKEN_HERE

[testpypi]
repository = https://test.pypi.org/legacy/
username = __token__
password = pypi-YOUR_TEST_API_TOKEN_HERE
EOF
```

**Option 2: Environment Variables**
```bash
export TWINE_USERNAME=__token__
export TWINE_PASSWORD=pypi-YOUR_API_TOKEN_HERE
```

### Automated Publishing with GitHub Actions

For automated releases, you can use GitHub Actions. Create `.github/workflows/publish.yml`:

```yaml
name: Publish to PyPI

on:
  release:
    types: [published]

jobs:
  publish:
    runs-on: ubuntu-latest
    steps:
    - uses: actions/checkout@v4
    - name: Set up Python
      uses: actions/setup-python@v4
      with:
        python-version: '3.11'
    - name: Install build dependencies
      run: |
        python -m pip install --upgrade pip
        pip install build twine
    - name: Build package
      run: python -m build
    - name: Publish to PyPI
      env:
        TWINE_USERNAME: __token__
        TWINE_PASSWORD: ${{ secrets.PYPI_API_TOKEN }}
      run: twine upload dist/*
```

Add your PyPI API token as `PYPI_API_TOKEN` in your repository secrets.

### Version Management

Follow [Semantic Versioning](https://semver.org/):
- **MAJOR** version when you make incompatible API changes
- **MINOR** version when you add functionality in a backwards compatible manner  
- **PATCH** version when you make backwards compatible bug fixes

### Verification

After publishing to PyPI, verify the installation works:
```bash
pip install ros-is-ros2==0.3.0  # Use your new version
ros-is-ros2 --help
```

For local/source installations, see the "Verifying Installation" section above.

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