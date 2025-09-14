#!/bin/bash

# Debug completion script

echo "🔍 Debugging rostopic completion..."

# Source the completion script
source scripts/rostopic_completion.bash

# Enable debug mode by uncommenting debug lines
sed -i 's/# echo "DEBUG:/echo "DEBUG:/g' scripts/rostopic_completion.bash

# Test function to simulate completion with debug info
test_debug_completion() {
    local cmd="$1"
    echo "💻 Debugging: '$cmd'"
    
    # Set up completion environment
    COMP_LINE="$cmd"
    COMP_POINT=${#COMP_LINE}
    COMP_WORDS=($cmd)
    COMP_CWORD=$((${#COMP_WORDS[@]} - 1))
    
    # If line ends with space, we're completing a new word
    if [[ "$cmd" =~ [[:space:]]$ ]]; then
        COMP_CWORD=$((COMP_CWORD + 1))
        COMP_WORDS+=("")
    fi
    
    echo "   COMP_LINE='$COMP_LINE'"
    echo "   COMP_WORDS=(${COMP_WORDS[*]})"
    echo "   COMP_CWORD=$COMP_CWORD"
    echo "   #COMP_WORDS=${#COMP_WORDS[@]}"
    
    # Clear replies and call completion
    COMPREPLY=()
    _rostopic_complete 2>&1
    
    # Show results
    echo "   Completions: ${COMPREPLY[*]}"
    echo
}

# Test problematic cases
test_debug_completion "rostopic "
test_debug_completion "rostopic l"

# Restore the completion script
sed -i 's/echo "DEBUG:/# echo "DEBUG:/g' scripts/rostopic_completion.bash

echo "🔍 Debug complete!"
