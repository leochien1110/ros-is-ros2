#!/usr/bin/env python3
"""
ROS 1-style CLI shims for ROS 2
Copyright (C) 2025 ros-is-ros2 contributors

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
"""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
from pathlib import Path

BLOCK_BEGIN = "# >>> ros-is-ros2 >>>"
BLOCK_END = "# <<< ros-is-ros2 <<<"


def rc_file(shell: str) -> Path:
    home = Path.home()
    if shell.endswith("zsh"):
        return home / ".zshrc"
    return home / ".bashrc"


def shim_path(shell: str) -> Path:
    here = Path(__file__).resolve().parent
    if shell.endswith("zsh"):
        return here / "shims" / "zsh.sh"
    return here / "shims" / "bash.sh"


def detect_shell() -> str:
    # Prefer $SHELL; fallback to argv[0]
    shell = os.environ.get("SHELL", "")
    if shell:
        return shell
    return shutil.which("bash") or "bash"


def add_block(rc: Path, shell: str, shim: Path):
    rc.parent.mkdir(parents=True, exist_ok=True)
    content = rc.read_text() if rc.exists() else ""
    # Remove previous block (idempotent)
    if BLOCK_BEGIN in content and BLOCK_END in content:
        pre = content.split(BLOCK_BEGIN)[0]
        post = content.split(BLOCK_END)[-1]
        content = pre + post
    line = f'. "{shim}"'
    block = f"{BLOCK_BEGIN}\n# ROS 1-style shims for ROS 2\n{line}\n{BLOCK_END}\n"
    # Append newline if file doesn't end with one
    if content and not content.endswith("\n"):
        content += "\n"
    rc.write_text(content + block)


def remove_block(rc: Path):
    if not rc.exists():
        return
    content = rc.read_text()
    if BLOCK_BEGIN in content and BLOCK_END in content:
        pre = content.split(BLOCK_BEGIN)[0]
        post = content.split(BLOCK_END)[-1]
        rc.write_text(pre + post)


def is_bash_completion_installed() -> bool:
    """Check if bash-completion is already installed and functional."""
    # First check if bash-completion package is installed (Linux)
    try:
        result = subprocess.run(
            ["dpkg", "-l", "bash-completion"], capture_output=True, text=True, timeout=5
        )
        if result.returncode == 0 and "ii" in result.stdout:
            return True
    except (subprocess.SubprocessError, FileNotFoundError):
        pass

    # Check for the main bash-completion file (most reliable indicator)
    main_completion_file = "/usr/share/bash-completion/bash_completion"
    if Path(main_completion_file).exists():
        return True

    # Check for other platform-specific locations
    other_paths = [
        "/usr/local/etc/bash_completion",  # macOS homebrew
        "/opt/homebrew/etc/bash_completion",  # macOS Apple Silicon homebrew
        "/usr/local/share/bash-completion/bash_completion",
    ]

    if any(Path(path).exists() for path in other_paths):
        return True

    # /etc/bash_completion often just sources the main file, so check if it's functional
    etc_completion = Path("/etc/bash_completion")
    if etc_completion.exists():
        try:
            content = etc_completion.read_text().strip()
            # If it just sources the main file, check if that file exists
            if content.startswith(". /usr/share/bash-completion/bash_completion"):
                return Path("/usr/share/bash-completion/bash_completion").exists()
            # If it has actual content, consider it installed
            elif len(content) > 100:  # Arbitrary threshold for "real" content
                return True
        except (OSError, UnicodeDecodeError):
            pass

    return False


def detect_package_manager() -> str | None:
    """Detect the available package manager."""
    managers = {
        "apt": ["apt", "apt-get"],
        "yum": ["yum"],
        "dnf": ["dnf"],
        "brew": ["brew"],
        "pacman": ["pacman"],
        "zypper": ["zypper"],
    }

    for manager, commands in managers.items():
        for cmd in commands:
            if shutil.which(cmd):
                return manager
    return None


def install_bash_completion() -> bool:
    """Install bash-completion using the appropriate package manager."""
    if is_bash_completion_installed():
        print("✓ bash-completion is already installed")
        return True

    print("Installing bash-completion for optimal tab completion...")

    package_manager = detect_package_manager()
    if not package_manager:
        print("⚠ Could not detect package manager.")
        print("Please install bash-completion manually:")
        print("  Ubuntu/Debian: sudo apt install bash-completion")
        print("  CentOS/RHEL:   sudo yum install bash-completion")
        print("  Fedora:        sudo dnf install bash-completion")
        print("  macOS:         brew install bash-completion")
        return False

    # Define installation commands
    install_commands = {
        "apt": ["sudo", "apt", "install", "-y", "bash-completion"],
        "yum": ["sudo", "yum", "install", "-y", "bash-completion"],
        "dnf": ["sudo", "dnf", "install", "-y", "bash-completion"],
        "brew": ["brew", "install", "bash-completion"],
        "pacman": ["sudo", "pacman", "-S", "--noconfirm", "bash-completion"],
        "zypper": ["sudo", "zypper", "install", "-y", "bash-completion"],
    }

    if package_manager not in install_commands:
        print(f"⚠ Unsupported package manager: {package_manager}")
        return False

    try:
        cmd = install_commands[package_manager]
        print(f"Running: {' '.join(cmd)}")

        # For brew, don't use sudo and capture output
        if package_manager == "brew":
            subprocess.run(cmd, check=True, capture_output=True, text=True)
        else:
            # Check if we can run sudo
            if os.geteuid() == 0:  # Already root
                cmd = [c for c in cmd if c != "sudo"]  # Remove sudo if already root
                subprocess.run(cmd, check=True, capture_output=True, text=True)
            else:
                # For sudo commands, don't capture output so password prompt works
                # Use stdin=None, stdout=None, stderr=None to inherit terminal
                subprocess.run(cmd, check=True, stdin=None, stdout=None, stderr=None)

        print("✓ bash-completion installed successfully")
        return True

    except subprocess.CalledProcessError as e:
        print(f"⚠ Failed to install bash-completion: {e}")
        if package_manager != "brew" and os.geteuid() != 0:
            print("This might be due to:")
            print("  - Incorrect sudo password")
            print("  - Network connectivity issues")
            print("  - Package repository problems")
        print("You can install it manually using:")
        print(f"  {' '.join(cmd)}")
        return False
    except Exception as e:
        print(f"⚠ Error installing bash-completion: {e}")
        return False


def main():
    p = argparse.ArgumentParser(
        prog="ros-is-ros2",
        description="Install ROS 1-style CLI shims (with autocompletion) for ROS 2.",
    )
    sub = p.add_subparsers(dest="cmd", required=True)
    sub.add_parser("install", help="Append sourcing line to your shell RC.")
    sub.add_parser("uninstall", help="Remove lines from your shell RC.")
    sub.add_parser(
        "print-path", help="Print the shim file path for your current shell."
    )
    args = p.parse_args()

    shell = detect_shell()
    rc = rc_file(shell)
    shim = shim_path(shell)

    if args.cmd == "install":
        # Install bash-completion if using bash
        if shell.endswith("bash"):
            install_bash_completion()

        add_block(rc, shell, shim)
        print(f"Added ros-is-ros2 block to {rc}")
        print("Reload your shell:  source", rc)
    elif args.cmd == "uninstall":
        remove_block(rc)
        print(f"Removed ros-is-ros2 block from {rc}")
    elif args.cmd == "print-path":
        print(shim)
    else:
        p.print_help()
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
