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

# Custom completion function for roslaunch with argument completion
_roslaunch_args_complete() {
    local state
    local -a arguments

    # Get the words array
    local -a words
    words=("${(@)words}")

    # Determine position: roslaunch <package> <launch_file> [args...]
    local package=""
    local launch_file=""
    local args_start_idx=0

    # Handle both "roslaunch pkg file.py" and "ros2 launch pkg file.py"
    if [[ "${words[1]}" == "roslaunch" ]]; then
        # roslaunch <package> <launch_file> [args...]
        if (( CURRENT >= 4 )); then
            package="${words[2]}"
            launch_file="${words[3]}"
            args_start_idx=4
        fi
    elif [[ "${words[1]}" == "ros2" ]] && [[ "${words[2]}" == "launch" ]]; then
        # ros2 launch <package> <launch_file> [args...]
        if (( CURRENT >= 5 )); then
            package="${words[3]}"
            launch_file="${words[4]}"
            args_start_idx=5
        fi
    fi

    # If we have package and launch_file and we're in the args section, provide argument completion
    if [[ -n "$package" ]] && [[ -n "$launch_file" ]] && (( CURRENT >= args_start_idx )); then
        # Don't complete if current word looks like an option (starts with -)
        if [[ "${words[CURRENT]}" == -* ]]; then
            # Delegate to argcomplete for options
            if typeset -f _python_argcomplete+ros2 >/dev/null; then
                _python_argcomplete+ros2
            fi
            return
        fi

        # Get argument names from the Python helper
        local helper_script="${(%):-%x}"
        helper_script="${helper_script:h}/../launch_completion_helper.py"

        if [[ -f "$helper_script" ]]; then
            local -a arg_names
            # Run the helper and capture output
            arg_names=(${(f)"$(python3 "$helper_script" "$package" "$launch_file" 2>/dev/null)"})

            if (( ${#arg_names[@]} > 0 )); then
                # Filter arguments that have already been provided
                local -a provided_args
                local word
                for ((i=args_start_idx; i<CURRENT; i++)); do
                    word="${words[i]}"
                    # Extract argument name from "arg:=value" format
                    if [[ "$word" =~ '^([^:]+):=' ]]; then
                        provided_args+=("${match[1]}")
                    fi
                done

                # Build completion list
                local -a completions
                local arg_name
                for arg_name in "${arg_names[@]}"; do
                    # Skip if already provided
                    local skip=0
                    local provided
                    for provided in "${provided_args[@]}"; do
                        if [[ "$arg_name" == "$provided" ]]; then
                            skip=1
                            break
                        fi
                    done

                    if (( skip == 0 )); then
                        # Add := suffix for convenience
                        completions+=("${arg_name}:=")
                    fi
                done

                # Provide completions
                if (( ${#completions[@]} > 0 )); then
                    _describe 'launch arguments' completions
                    return
                fi
            fi
        fi

        # Don't fallback to argcomplete when we're in the arguments section
        # Return with whatever completions we found (even if empty)
        return
    fi

    # Fallback to argcomplete for package/file completion and other cases
    if typeset -f _python_argcomplete+ros2 >/dev/null; then
        _python_argcomplete+ros2
    fi
}

# Map zsh completion to ros2's argcomplete adapter
# In zsh, argcomplete registers a function named `_python_argcomplete+ros2`
if typeset -f _python_argcomplete+ros2 >/dev/null; then
  compdef _python_argcomplete+ros2 rostopic
  compdef _python_argcomplete+ros2 rosnode
  compdef _python_argcomplete+ros2 rosservice
  compdef _python_argcomplete+ros2 rosparam
  compdef _python_argcomplete+ros2 rosbag
  compdef _python_argcomplete+ros2 rosrun
  compdef _roslaunch_args_complete roslaunch
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
