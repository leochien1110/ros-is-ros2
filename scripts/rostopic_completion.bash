#!/bin/bash

# Bash completion for rostopic that delegates to ros2 topic completion

_rostopic_complete() {
    # Initialize completion variables manually (compatible with any bash)
    local cur prev words cword
    cur="${COMP_WORDS[COMP_CWORD]}"
    prev="${COMP_WORDS[COMP_CWORD-1]}"
    words=("${COMP_WORDS[@]}")
    cword=${COMP_CWORD}

    # Store original values
    local original_comp_words=("${COMP_WORDS[@]}")
    
    # Check if we're completing subcommands (rostopic <TAB> or rostopic l<TAB>)
    if [[ ${#original_comp_words[@]} -le 2 ]]; then
        # Completing subcommand: rostopic <TAB> or rostopic l<TAB>
        local subcommands="list echo hz info type find pub bw"
        COMPREPLY=($(compgen -W "$subcommands" -- "$cur"))
    else
        # We have a subcommand, delegate to ros2 completion
        
        # Get the current command line
        local rostopic_line="${COMP_LINE}"
        
        # Translate rostopic command to ros2 topic equivalent
        local ros2_line="${rostopic_line/rostopic/ros2 topic}"
        
        # Set up completion environment for ros2
        local original_comp_line="$COMP_LINE"
        local original_comp_point="$COMP_POINT"
        local original_comp_cword="$COMP_CWORD"
        
        # Update completion variables for ros2
        COMP_LINE="$ros2_line"
        COMP_POINT=$((${#ros2_line}))
        
        # Split the ros2 command line into words
        local ros2_words
        read -ra ros2_words <<< "$ros2_line"
        COMP_WORDS=("${ros2_words[@]}")
        
        # Adjust COMP_CWORD to point to the current word being completed
        COMP_CWORD=$((${#ros2_words[@]} - 1))
        
        # If the line ends with a space, we're completing a new word
        if [[ "$ros2_line" =~ [[:space:]]$ ]]; then
            COMP_CWORD=$((COMP_CWORD + 1))
            COMP_WORDS+=("")
        fi
        
        # Clear previous replies
        COMPREPLY=()
        
        # Try to use ros2's completion function if available
        if declare -F _ros2_complete >/dev/null 2>&1; then
            _ros2_complete
        elif command -v ros2 >/dev/null 2>&1; then
            # Fallback: try to get completions from ros2 directly
            _rostopic_fallback_complete
        else
            # Final fallback: basic completions
            _rostopic_basic_complete
        fi
        
        # Restore original completion environment
        COMP_LINE="$original_comp_line"
        COMP_POINT="$original_comp_point"
        COMP_WORDS=("${original_comp_words[@]}")
        COMP_CWORD="$original_comp_cword"
    fi
}

_rostopic_fallback_complete() {
    local cur="${COMP_WORDS[COMP_CWORD]}"
    
    # Try to get completions using ros2's help output
    if [[ ${#COMP_WORDS[@]} -le 2 ]]; then
        # Completing subcommand: rostopic <TAB> or rostopic l<TAB>
        COMPREPLY=($(compgen -W "list echo hz info type find pub bw" -- "$cur"))
    elif [[ ${#COMP_WORDS[@]} -ge 3 ]]; then
        local subcommand="${COMP_WORDS[2]}"
        
        # Get flag completions from ros2 help
        local flags
        flags=$(ros2 topic "$subcommand" --help 2>/dev/null | grep -E '^\s*-' | sed 's/,.*//g' | awk '{print $1}' | grep '^-')
        
        # Get entity completions (topics, nodes, services)
        local entities=""
        case "$subcommand" in
            echo|hz|info|type|pub)
                entities=$(ros2 topic list 2>/dev/null)
                ;;
        esac
        
        # Combine flags and entities
        local completions="$flags $entities"
        COMPREPLY=($(compgen -W "$completions" -- "$cur"))
    fi
}

_rostopic_basic_complete() {
    local cur="${COMP_WORDS[COMP_CWORD]}"
    
    if [[ ${#COMP_WORDS[@]} -le 2 ]]; then
        # Completing subcommand: rostopic <TAB> or rostopic l<TAB>
        COMPREPLY=($(compgen -W "list echo hz info type find pub bw" -- "$cur"))
    else
        # Basic flag completions
        COMPREPLY=($(compgen -W "-h --help" -- "$cur"))
    fi
}

# Register the completion function
complete -F _rostopic_complete rostopic

# Also try to set up ros2 completion if not already done
if command -v ros2 >/dev/null 2>&1; then
    # Try to source ros2 completion
    if [[ -f /usr/share/bash-completion/completions/ros2 ]]; then
        source /usr/share/bash-completion/completions/ros2 2>/dev/null || true
    elif [[ -f /opt/ros/*/share/ros2cli/environment/ros2_completion.bash ]]; then
        source /opt/ros/*/share/ros2cli/environment/ros2_completion.bash 2>/dev/null || true
    fi
    
    # Try argcomplete approach as well
    if command -v register-python-argcomplete >/dev/null 2>&1; then
        eval "$(register-python-argcomplete ros2)" 2>/dev/null || true
    fi
fi
