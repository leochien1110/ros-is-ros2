# --- BEGIN ros-is-ros2 (bash) ---
# This file is sourced by your shell RC. Do not execute directly.

# 1) Load bash-completion (needed for programmable completion)
# Try multiple common locations for bash-completion
if [ -f /etc/bash_completion ]; then
  . /etc/bash_completion 2>/dev/null
elif [ -f /usr/share/bash-completion/bash_completion ]; then
  . /usr/share/bash-completion/bash_completion 2>/dev/null
elif [ -f /etc/bash/bash_completion ]; then
  . /etc/bash/bash_completion 2>/dev/null
elif [ -f /usr/local/share/bash-completion/bash_completion ]; then
  . /usr/local/share/bash-completion/bash_completion 2>/dev/null
else
  # If no bash-completion found, try to enable basic completion
  if [ -n "$BASH_VERSION" ] && shopt -q progcomp 2>/dev/null; then
    # Basic completion is available, continue
    true
  fi
fi

# 2) Source ROS 2 and register argcomplete for `ros2`
# Try to find and source ROS 2 setup
if [ -n "$ROS_DISTRO" ] && [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
  . "/opt/ros/$ROS_DISTRO/setup.bash"
elif [ -f /opt/ros/humble/setup.bash ]; then
  . /opt/ros/humble/setup.bash
elif [ -f /opt/ros/jazzy/setup.bash ]; then
  . /opt/ros/jazzy/setup.bash
elif [ -f /opt/ros/iron/setup.bash ]; then
  . /opt/ros/iron/setup.bash
elif [ -f /opt/ros/rolling/setup.bash ]; then
  . /opt/ros/rolling/setup.bash
fi

# If register-python-argcomplete exists, enable ros2 completion
if command -v register-python-argcomplete >/dev/null 2>&1; then
  eval "$(register-python-argcomplete ros2)"
fi

# 3) Load complete_alias (vendored)
_ros_is_ros2_complete_alias_path="${BASH_SOURCE[0]%/*}/../third_party/complete_alias.bash"
if [ -f "$_ros_is_ros2_complete_alias_path" ]; then
  . "$_ros_is_ros2_complete_alias_path"
fi

# 4) Define ROS1-like aliases to ROS2
alias rostopic='ros2 topic'
alias rosnode='ros2 node'
alias rosservice='ros2 service'
alias rosparam='ros2 param'
alias rosbag='ros2 bag'
alias rosrun='ros2 run'
alias roslaunch='ros2 launch'
alias rospack='ros2 pkg'
# Optionals:
alias rossrv='ros2 interface'
alias rosmsg='ros2 interface'

# 5) Delegate completion to the underlying command
if declare -F _complete_alias >/dev/null 2>&1; then
  complete -F _complete_alias rostopic
  complete -F _complete_alias rosnode
  complete -F _complete_alias rosservice
  complete -F _complete_alias rosparam
  complete -F _complete_alias rosbag
  complete -F _complete_alias rosrun
  complete -F _complete_alias roslaunch
  complete -F _complete_alias rospack
  complete -F _complete_alias rossrv
  complete -F _complete_alias rosmsg
fi
# --- END ros-is-ros2 (bash) ---
