#!/bin/bash

# Final test script for completion functionality

echo "🧪 Testing rostopic completion functionality..."

# Source the completion script
source scripts/rostopic_completion.bash

# Test function to simulate completion
test_completion() {
    local cmd="$1"
    echo "💻 Testing: '$cmd'"
    
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
    
    # Clear replies and call completion
    COMPREPLY=()
    _rostopic_complete
    
    # Show results
    if [[ ${#COMPREPLY[@]} -eq 0 ]]; then
        echo "   ❌ No completions found"
    else
        echo "   ✅ Completions (${#COMPREPLY[@]}): ${COMPREPLY[*]}"
    fi
    echo
}

echo "=== Basic Subcommand Completion ==="
test_completion "rostopic "
test_completion "rostopic l"

echo "=== Flag Completion ==="
test_completion "rostopic list "
test_completion "rostopic list -"

echo "=== Topic Completion ==="
test_completion "rostopic echo "
test_completion "rostopic echo /"

echo "=== Advanced Completion ==="
test_completion "rostopic hz "
test_completion "rostopic info "

echo "🎉 Completion test complete!"
