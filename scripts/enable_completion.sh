#!/bin/bash

# Script to enable bash completion for ros-is-ros2 commands

echo "Setting up bash completion for ros-is-ros2 commands..."

# Check if argcomplete is installed
if ! python3 -c "import argcomplete" 2>/dev/null; then
    echo "Error: argcomplete is not installed. Please install it with:"
    echo "  pip install argcomplete"
    exit 1
fi

# Check if ros-is-ros2 is installed
if ! python3 -c "import ros_is_ros2" 2>/dev/null; then
    echo "Error: ros-is-ros2 is not installed. Please install it first."
    exit 1
fi

# Get the script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
COMPLETION_SCRIPT="$SCRIPT_DIR/rostopic_completion.bash"

# Check if completion script exists
if [[ ! -f "$COMPLETION_SCRIPT" ]]; then
    echo "Error: Completion script not found at $COMPLETION_SCRIPT"
    exit 1
fi

# Enable completion for current session
echo "Enabling completion for current session..."
source "$COMPLETION_SCRIPT"

# Also enable argcomplete as fallback
eval "$(register-python-argcomplete rostopic)" 2>/dev/null || true

# Add to .bashrc for permanent installation
BASHRC="$HOME/.bashrc"
BASH_COMPLETION_LINE="source '$COMPLETION_SCRIPT'"
ARGCOMPLETE_LINE='eval "$(register-python-argcomplete rostopic)"'

if ! grep -q "rostopic_completion.bash" "$BASHRC" 2>/dev/null; then
    echo "Adding completion to $BASHRC..."
    echo "" >> "$BASHRC"
    echo "# ros-is-ros2 completion" >> "$BASHRC"
    echo "$BASH_COMPLETION_LINE" >> "$BASHRC"
    echo "# Fallback argcomplete completion" >> "$BASHRC"
    echo "command -v register-python-argcomplete >/dev/null 2>&1 && $ARGCOMPLETE_LINE" >> "$BASHRC"
    echo "Completion added to $BASHRC"
else
    echo "Completion already configured in $BASHRC"
fi

echo ""
echo "✅ Completion setup complete!"
echo ""
echo "To test completion:"
echo "  - Type: rostopic <TAB><TAB>"
echo "  - Type: rostopic l<TAB> (should complete to 'list')"
echo "  - Type: rostopic list <TAB><TAB> (should show --count-topics, -s, -t, etc.)"
echo "  - Type: rostopic echo <TAB><TAB> (should show available topics)"
echo ""
echo "If completion doesn't work immediately, restart your terminal or run:"
echo "  source ~/.bashrc"
