#!/bin/zsh
# --- BEGIN ros-is-ros2 (zsh) ---
# This file is sourced by your shell RC. Do not execute directly.

# 1) Source ROS 2 and enable completion
# Try to find and source ROS 2 setup
if [ -n "$ROS_DISTRO" ] && [ -f "/opt/ros/$ROS_DISTRO/setup.zsh" ]; then
  source "/opt/ros/$ROS_DISTRO/setup.zsh"
elif [ -f /opt/ros/humble/setup.zsh ]; then
  source /opt/ros/humble/setup.zsh
elif [ -f /opt/ros/jazzy/setup.zsh ]; then
  source /opt/ros/jazzy/setup.zsh
elif [ -f /opt/ros/iron/setup.zsh ]; then
  source /opt/ros/iron/setup.zsh
elif [ -f /opt/ros/rolling/setup.zsh ]; then
  source /opt/ros/rolling/setup.zsh
fi

autoload -U +X compinit && compinit
# register ros2's argcomplete for zsh
if command -v register-python-argcomplete >/dev/null 2>&1; then
  eval "$(register-python-argcomplete --shell zsh ros2)"
fi

# 2) Use wrapper *functions* (not aliases) and hook ros2 completer
rostopic()   { ros2 topic   "$@"; }
rosnode()    { ros2 node    "$@"; }
rosservice() { ros2 service "$@"; }
rosparam()   { ros2 param   "$@"; }
rosbag()     { ros2 bag     "$@"; }
rosrun()     { ros2 run     "$@"; }
roslaunch()  { ros2 launch  "$@"; }
rospack()    { ros2 pkg     "$@"; }
rossrv()     { ros2 interface "$@"; }
rosmsg()     { ros2 interface "$@"; }

# Map zsh completion to ros2's argcomplete adapter
# In zsh, argcomplete registers a function named `_python_argcomplete+ros2`
if typeset -f _python_argcomplete+ros2 >/dev/null; then
  compdef _python_argcomplete+ros2 rostopic
  compdef _python_argcomplete+ros2 rosnode
  compdef _python_argcomplete+ros2 rosservice
  compdef _python_argcomplete+ros2 rosparam
  compdef _python_argcomplete+ros2 rosbag
  compdef _python_argcomplete+ros2 rosrun
  compdef _python_argcomplete+ros2 roslaunch
  compdef _python_argcomplete+ros2 rospack
  compdef _python_argcomplete+ros2 rossrv
  compdef _python_argcomplete+ros2 rosmsg
fi

# 3) Define rosdomainid function for managing ROS_DOMAIN_ID
rosdomainid() {
    if [ $# -eq 0 ]; then
        echo "${ROS_DOMAIN_ID:-unset}"
    else
        if [[ $1 =~ '^[0-9]+$' ]] && (( $1 >= 0 && $1 <= 101 )); then
            export ROS_DOMAIN_ID="$1"
            echo "ROS_DOMAIN_ID set to $1"
        else
            echo "Error: Argument must be a number between 0 and 101" >&2
            return 1
        fi
    fi
}

# Basic completion for rosdomainid (suggests numbers 0-101)
compdef '_values -s , "domain ids" {0..101}' rosdomainid

# 4) Define rosdepinstall function for easy rosdep installation
rosdepinstall() {
    rosdep update && rosdep install --from-paths src --ignore-src -r -y
}

# --- END ros-is-ros2 (zsh) ---
