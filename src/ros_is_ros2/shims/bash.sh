#!/bin/bash
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
  # shellcheck disable=SC1090
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
  # shellcheck disable=SC1090
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

# 5) Custom completion function for roslaunch with argument completion
_roslaunch_args_complete() {
    local cur="${COMP_WORDS[COMP_CWORD]}"

    # Determine if we're completing launch arguments
    # roslaunch (or ros2 launch) <package> <launch_file> [args...]
    # We need at least 3 words: command, package, launch_file

    local package=""
    local launch_file=""
    local args_start_idx=0

    # Detect the position of package and launch file
    # Handle both "roslaunch pkg file.py" and "ros2 launch pkg file.py"
    if [[ "${COMP_WORDS[0]}" == "roslaunch" ]]; then
        # roslaunch <package> <launch_file> [args...]
        if [[ ${COMP_CWORD} -ge 3 ]]; then
            package="${COMP_WORDS[1]}"
            launch_file="${COMP_WORDS[2]}"
            args_start_idx=3
        fi
    elif [[ "${COMP_WORDS[0]}" == "ros2" ]] && [[ "${COMP_WORDS[1]}" == "launch" ]]; then
        # ros2 launch <package> <launch_file> [args...]
        if [[ ${COMP_CWORD} -ge 4 ]]; then
            package="${COMP_WORDS[2]}"
            launch_file="${COMP_WORDS[3]}"
            args_start_idx=4
        fi
    fi

    # If we have package and launch_file and we're in the args section, provide argument completion
    if [[ -n "$package" ]] && [[ -n "$launch_file" ]] && [[ ${COMP_CWORD} -ge $args_start_idx ]]; then
        # If current word looks like an option (starts with -), provide ros2 launch flags
        if [[ "$cur" == -* ]]; then
            # Common ros2 launch flags
            local flags=(
                "-h" "--help"
                "-a" "--show-all-subprocesses-output"
                "-d" "--debug"
                "-s" "--show-args" "--show-arguments"
                "-p" "--print" "--print-description"
                "-n" "--noninteractive"
                "--launch-prefix"
                "--launch-prefix-filter"
            )
            for flag in "${flags[@]}"; do
                if [[ "$flag" == "$cur"* ]]; then
                    COMPREPLY+=("$flag")
                fi
            done
            return
        fi

        # Get argument names from the Python helper
        local helper_script="${BASH_SOURCE[0]%/*}/../launch_completion_helper.py"
        if [[ -f "$helper_script" ]]; then
            local arguments
            # Run the helper and capture output
            arguments=$(python3 "$helper_script" "$package" "$launch_file" 2>/dev/null)

            if [[ -n "$arguments" ]]; then
                # Filter arguments that have already been provided
                local provided_args=()
                local i
                for ((i=args_start_idx; i<COMP_CWORD; i++)); do
                    local word="${COMP_WORDS[i]}"
                    # Extract argument name from "arg:=value" format
                    if [[ "$word" =~ ^([^:]+):= ]]; then
                        provided_args+=("${BASH_REMATCH[1]}")
                    fi
                done

                # Generate completions
                local arg
                while IFS= read -r arg; do
                    # Skip if already provided
                    local skip=0
                    local provided
                    for provided in "${provided_args[@]}"; do
                        if [[ "$arg" == "$provided" ]]; then
                            skip=1
                            break
                        fi
                    done

                    if [[ $skip -eq 0 ]]; then
                        # Add := suffix for convenience
                        local suggestion="${arg}:="
                        # Only suggest if it matches the current prefix
                        if [[ "$suggestion" == "$cur"* ]]; then
                            COMPREPLY+=("$suggestion")
                        fi
                    fi
                done <<< "$arguments"
            fi
        fi

        # Don't fallback when we're in the arguments section
        # Return with whatever completions we found (even if empty)
        return
    fi

    # For package/file completion, manually get completions from ros2 command
    # This avoids the "cannot unmask alias" error from _complete_alias

    # Position 1: completing package name
    if [[ ${COMP_CWORD} -eq 1 ]]; then
        # Get list of packages with launch files
        local packages
        packages=$(ros2 pkg list 2>/dev/null)
        if [[ -n "$packages" ]]; then
            while IFS= read -r pkg; do
                if [[ "$pkg" == "$cur"* ]]; then
                    COMPREPLY+=("$pkg")
                fi
            done <<< "$packages"
        fi
        return
    fi

    # Position 2: completing launch file name
    if [[ ${COMP_CWORD} -eq 2 ]]; then
        local pkg="${COMP_WORDS[1]}"
        # Get launch file directory
        local pkg_share
        pkg_share=$(ros2 pkg prefix "$pkg" 2>/dev/null)
        if [[ -n "$pkg_share" ]]; then
            # Only search in the launch directory to avoid duplicates
            local launch_dir="$pkg_share/share/$pkg/launch"
            if [[ -d "$launch_dir" ]]; then
                # Find launch files (including symlinks with -L)
                # Match both naming styles: *.launch.py (workspace) and *_launch.py (system)
                local files
                files=$(find -L "$launch_dir" -maxdepth 2 -type f \( -name "*launch.py" -o -name "*launch.xml" -o -name "*launch.yaml" \) 2>/dev/null)
                if [[ -n "$files" ]]; then
                    # Use associative array to deduplicate
                    declare -A seen
                    while IFS= read -r file; do
                        local filename
                        filename=$(basename "$file")
                        if [[ "$filename" == "$cur"* ]] && [[ -z "${seen[$filename]}" ]]; then
                            COMPREPLY+=("$filename")
                            seen[$filename]=1
                        fi
                    done <<< "$files"
                fi
            fi
        fi
        return
    fi
}

# 6) Register custom completion for roslaunch
complete -F _roslaunch_args_complete roslaunch

# 7) Delegate completion to other aliases
if declare -F _complete_alias >/dev/null 2>&1; then
  complete -F _complete_alias rostopic
  complete -F _complete_alias rosnode
  complete -F _complete_alias rosservice
  complete -F _complete_alias rosparam
  complete -F _complete_alias rosbag
  complete -F _complete_alias rosrun
  complete -F _complete_alias rospack
  complete -F _complete_alias rossrv
  complete -F _complete_alias rosmsg
fi

# 8) Define rosdomainid function
rosdomainid() {
    if [ $# -eq 0 ]; then
        echo "${ROS_DOMAIN_ID:-unset}"
    else
        if [[ $1 =~ ^[0-9]+$ ]] && (( $1 >= 0 && $1 <= 101 )); then
            export ROS_DOMAIN_ID="$1"
            echo "ROS_DOMAIN_ID set to $1"
        else
            echo "Error: Argument must be a number between 0 and 101" >&2
            return 1
        fi
    fi
}

_rosdomainid_complete() {
    mapfile -t COMPREPLY < <(compgen -W "$(seq 0 101)" -- "$2")
}
complete -F _rosdomainid_complete rosdomainid

# 9) Define rosdepinstall function
rosdepinstall() {
    rosdep update && rosdep install --from-paths src --ignore-src -r -y
}

# --- END ros-is-ros2 (bash) ---
