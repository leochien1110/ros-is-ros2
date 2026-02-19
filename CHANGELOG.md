# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/), and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.4.1] - 2026-02-18

### Added
- `status` subcommand: `ros-is-ros2 status` reports whether shell integration is active.
- First-run hint: running `ros-is-ros2` with no subcommand now shows a friendly setup prompt (`⚡ Shell integration not set up yet!`) when the shim block is not yet present in the user's RC file, instead of a cryptic argparse error.
- `is_installed()` helper in `cli.py` to detect whether the RC block is present.
- `SETUP_HINT` constant for consistent messaging across `status` and the no-args path.

### Changed
- `install` success message now uses `✓` prefix for consistency with `status` output.
- README installation sections simplified to single copy-paste one-liners (`pip install ... && ros-is-ros2 install && source ~/.bashrc`).
- Removed the `python3 -m ros_is_ros2 install` fallback note (superseded by the first-run hint).

### Fixed
- Running `ros-is-ros2` with no arguments no longer exits with a non-zero code and argparse error; it exits cleanly with code 0 and actionable output.

## [0.4.0] - 2025-10-25

### Added
- Launch argument tab completion (ROS1-style)
  - Tab-complete launch file arguments for both system and workspace packages
  - Partial matching and filtering of already-provided arguments
  - File-based caching (1 hour TTL, configurable)
  - Supports Python, XML, and YAML launch files
- `launch_completion_helper.py`: Helper for parsing launch arguments
- Cache cleanup on uninstall
- Support for ros2 launch flags completion (`-s`, `-d`, `--debug`, etc.)

### Changed
- Custom completion for `roslaunch` with argument support (bash and zsh)
- Added `clean_cache()` to cli.py for reversible uninstallation
- Improved launch file discovery (supports symlinks in workspace packages)

## [0.3.0] - 2025-09-27

### Added
- `rosdomainid` command: Easily check or set `ROS_DOMAIN_ID` (e.g., `rosdomainid` shows current value; `rosdomainid 42` sets and exports it to 42).
  - Validates input as a number between 0 and 101.
  - Added dynamic tab completion suggesting 0-101.
- `rosdepinstall` command: Shortcut for `rosdep update && rosdep install --from-paths src --ignore-src -r -y`.
- Updated README.md with instructions for updating the package and new macros section.
- Improved shell shims for cleaner range generation in completion (using `seq` in Bash and brace expansion in Zsh).

### Fixed
- Shellcheck warnings in bash.sh completion function (SC2207) by using `mapfile` for safe array assignment.

## [0.2.0] - 2025-09-20

### Added
- Integration with `complete-alias` for full tab completion on aliased ROS1 commands.
- Pre-commit hooks for automated code quality checks (formatting, linting, testing).
- Refined shell shims for Bash and Zsh, including automatic bash-completion installation.
- Multiple updates to aliases and completion setup.

### Changed
- Updated project URL and tests.

## [0.1.0] - 2025-09-15

### Added
- Initial project structure with basic ROS1-style aliases (e.g., `rostopic` → `ros2 topic`).
- Autocomplete fixes for subcommands and arguments.
- .gitignore for Python cache files.
- Basic implementation focusing on topic commands, with plans for expansion.
