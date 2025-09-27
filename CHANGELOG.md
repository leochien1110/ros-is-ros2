# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/), and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

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
