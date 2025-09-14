# Installation & Setup Guide

## 🚀 Super Simple Install (Perfect Tab Completion + Help Delegation!)

1. **Install the package**:
   ```bash
   cd /path/to/ros-is-ros2
   pip install -e .
   ```

2. **Run any command** (completion is set up automatically):
   ```bash
   rostopic list
   ```
   
3. **Restart your shell** and enjoy tab completion:
   ```bash
   # Start a new shell or run:
   source ~/.bashrc
   
   # Then test completion:
   rostopic li<TAB>  # Should complete to "rostopic list"
   rostopic e<TAB>   # Should complete to "rostopic echo"
   ```

## 📋 Available Commands

After installation, these ROS1-style commands will be available:

- `rostopic` → `ros2 topic`
- `rosnode` → `ros2 node` 
- `rosservice` → `ros2 service`
- `rosparam` → `ros2 param`
- `rosmsg` → `ros2 interface` (messages only)
- `rossrv` → `ros2 interface` (services only)
- `rosbag` → `ros2 bag`
- `rosrun` → `ros2 run`
- `roslaunch` → `ros2 launch`
- `roscore` → Prints helpful migration message

## ✨ Features

- **Dynamic Discovery**: Commands and subcommands are discovered from your ROS2 installation at runtime
- **Complete Tab Completion**: Works for subcommands, arguments, flags, topic names, node names, and more
- **Proper Help Delegation**: `rostopic list --help` shows the actual `ros2 topic list` help
- **Automatic Mapping**: All `ros<X>` commands automatically map to `ros2 <X>`
- **Special Cases**: `rosmsg`/`rossrv` properly map to `ros2 interface` with appropriate flags
- **Robust Fallbacks**: Completion works even when ROS2 introspection fails
- **Future-Proof**: Automatically supports new ROS2 commands without code changes

## 🧪 Testing

```bash
# Test basic functionality
rostopic list
rosnode list
rosmsg list
rossrv list

# Test help delegation
rostopic --help         # Shows general rostopic help
rostopic list --help    # Shows specific ros2 topic list help ✅

# Test autocompletion (after sourcing ~/.bashrc)  
rostopic <TAB><TAB>     # Shows all subcommands ✅
rostopic li<TAB>        # Completes to "list" ✅

# Test flag completion
rostopic list --<TAB>   # Shows topic-specific flags: --verbose, --show-types, etc. ✅
rostopic list --v<TAB>  # Completes to "--verbose" ✅

# Test argument completion
rostopic echo <TAB>     # Shows topics and applicable flags ✅
rostopic echo /p<TAB>   # Completes to "/parameter_events" ✅
rosnode info <TAB>      # Shows available nodes ✅

# Test actual functionality
rostopic list --verbose    # Works with proper ros2 delegation ✅
rostopic list --show-types # Works with proper ros2 delegation ✅
```

## 🔧 Troubleshooting

**Commands not found**:
- Make sure `~/.local/bin` is in your PATH
- Try `pip install -e . --force-reinstall`

**No autocompletion**:
- Make sure you ran a ros command first to trigger auto-setup
- Run `source ~/.bashrc` or restart your shell
- Check if completion is registered: `complete -p | grep rostopic`

**ROS2 not detected**:
- Source your ROS2 environment: `source /opt/ros/<distro>/setup.bash`
- Verify `ros2` command is available: `which ros2`

## 🔄 Uninstall

```bash
pip uninstall ros-is-ros2
# Optionally remove completion lines from ~/.bashrc:
sed -i '/# ros-is-ros2 auto completion/,/roslaunch/d' ~/.bashrc
```
