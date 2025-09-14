# Project Plan: ros-is-ros2

The goal of **ros-is-ros2** is to provide ROS1-style commands (`rostopic`, `rosnode`, etc.) as thin shims over ROS2 (`ros2 topic`, `ros2 node`, etc.) to improve developer ergonomics.

> For each milestone, write down the note in the note folder.

## To Do List

- [ ] Initialize repo with `pyproject.toml` (using hatchling or poetry).
- [ ] Set up base package structure: `src/ros_is_ros2/`.
- [ ] Add linting (ruff, black), type checking (mypy), and testing (pytest).
- [ ] Configure GitHub Actions CI with lint + unit test.
- [ ] Add console script `rostopic`.
- [ ] Map to `ros2 topic <subcommand>`.
- [ ] Support subcommands: `list`, `echo`, `hz`, `info`.
- [ ] Add integration test in ROS2 Docker container (Humble).
- [ ] Add console script `rosnode`.
- [ ] Map to `ros2 node <subcommand>`.
- [ ] Support subcommands: `list`, `info`.
- [ ] Add console script `rosservice`.
- [ ] Map to `ros2 service <subcommand>`.
- [ ] Support subcommands: `list`, `type`, `call`.
- [ ] Add console script `rosparam`.
- [ ] Map to `ros2 param <subcommand>`.
- [ ] Support subcommands: `list`, `get`, `set`, `dump`, `load`.
- [ ] Add console script `rosrun`.
- [ ] Map to `ros2 run <pkg> <executable>`.
- [ ] Ensure package/executable arguments pass through cleanly.
- [ ] Add console script `rosbag`.
- [ ] Map to `ros2 bag <subcommand>`.
- [ ] Support `record`, `play`, `info`.
- [ ] Add console script `rosmsg`.
- [ ] Map to `ros2 interface <subcommand>`.
- [ ] Support `list`, `show`.
- [ ] Add console script `rossrv`.
- [ ] Map to `ros2 interface <subcommand>`.
- [ ] Support `list`, `show`.
- [ ] Add console script `roslaunch`.
- [ ] Map to `ros2 launch`.
- [ ] Handle `.launch.py` files and warn for deprecated `.launch` files.
- [ ] Print helpful message: “ROS2 does not use roscore. Try `ros2 daemon start`.”
- [ ] Publish to TestPyPI then PyPI.
- [ ] Update README with parity table.
- [ ] Record asciinema demo for `rostopic`, `rosnode`, `rosrun`.
- [ ] Announce early preview to ROS Discourse.
- [ ] Add Homebrew formula (`brew install ros-is-ros2`).
- [ ] Package for Arch Linux AUR.
- [ ] Write CONTRIBUTING.md and tag `good first issue`.
- [ ] Ensure all major ROS1 commands implemented.
- [ ] Document limitations (e.g., no `roscore`).
- [ ] Publish GitHub Release.
- [ ] Post official announcement with demo gifs.
---

## Milestone 0 – Project Setup
**Tasks:**
- Initialize repo with `pyproject.toml` (using hatchling or poetry).
- Set up base package structure: `src/ros_is_ros2/`.
- Add linting (ruff, black), type checking (mypy), and testing (pytest).
- Configure GitHub Actions CI with lint + unit test.

**Done criteria:**
- Repo builds successfully.
- CI passes lint + test on Python 3.10–3.12.
- Empty `main.py` with placeholder dispatcher runs via `python -m ros_is_ros2`.

---

## Milestone 1 – Implement `rostopic`
**Tasks:**
- Add console script `rostopic`.
- Map to `ros2 topic <subcommand>`.
- Support subcommands: `list`, `echo`, `hz`, `info`.
- Add integration test in ROS2 Docker container (Humble).

**Done criteria:**
- Running `rostopic list` inside container produces same output as `ros2 topic list`.
- Tests pass locally and in CI.

---

## Milestone 2 – Implement `rosnode`
**Tasks:**
- Add console script `rosnode`.
- Map to `ros2 node <subcommand>`.
- Support subcommands: `list`, `info`.

**Done criteria:**
- `rosnode list` and `rosnode info <node>` behave as expected.
- Integration tests pass in ROS2 container.

---

## Milestone 3 – Implement `rosservice`
**Tasks:**
- Add console script `rosservice`.
- Map to `ros2 service <subcommand>`.
- Support subcommands: `list`, `type`, `call`.

**Done criteria:**
- Example services (`/add_two_ints`) callable with `rosservice call`.
- Integration tests confirm parity with `ros2 service`.

---

## Milestone 4 – Implement `rosparam`
**Tasks:**
- Add console script `rosparam`.
- Map to `ros2 param <subcommand>`.
- Support subcommands: `list`, `get`, `set`, `dump`, `load`.

**Done criteria:**
- Running `rosparam list` shows active parameters.
- `rosparam get/set` works on a demo node.
- Tests validate expected behavior.

---

## Milestone 5 – Implement `rosrun`
**Tasks:**
- Add console script `rosrun`.
- Map to `ros2 run <pkg> <executable>`.
- Ensure package/executable arguments pass through cleanly.

**Done criteria:**
- `rosrun demo_nodes_cpp talker` runs the same as `ros2 run demo_nodes_cpp talker`.
- Test confirms exit code and stdout parity.

---

## Milestone 6 – Implement `rosbag`
**Tasks:**
- Add console script `rosbag`.
- Map to `ros2 bag <subcommand>`.
- Support `record`, `play`, `info`.

**Done criteria:**
- `rosbag record` creates a ROS2 bag file.
- `rosbag info` shows metadata.
- Integration tests validate basic recording/playback.

---

## Milestone 7 – Implement `rosmsg` and `rossrv`
**Tasks:**
- Add console scripts `rosmsg` and `rossrv`.
- Map to `ros2 interface <subcommand>`.
- Support `list`, `show`.

**Done criteria:**
- `rosmsg list` shows available message types.
- `rossrv show std_srvs/srv/Empty` works correctly.
- Tests confirm parity.

---

## Milestone 8 – Implement `roslaunch`
**Tasks:**
- Add console script `roslaunch`.
- Map to `ros2 launch`.
- Handle `.launch.py` files and warn for deprecated `.launch` files.

**Done criteria:**
- `roslaunch demo_nodes_cpp talker.launch.py` works.
- Unsupported `.launch` triggers a friendly message.
- Integration test demonstrates working launch.

---

## Milestone 9 – Handle `roscore`
**Tasks:**
- Add console script `roscore`.
- Print helpful message: “ROS2 does not use roscore. Try `ros2 daemon start`.”

**Done criteria:**
- Running `roscore` exits with non-zero and displays guidance.
- Test verifies message output.

---

## Milestone 10 – Release v0.1.0
**Tasks:**
- Publish to TestPyPI then PyPI.
- Update README with parity table.
- Record asciinema demo for `rostopic`, `rosnode`, `rosrun`.
- Announce early preview to ROS Discourse.

**Done criteria:**
- Users can install with `pipx install ros-is-ros2`.
- Core commands (`rostopic`, `rosnode`, `rosrun`) functional.

---

## Milestone 11 – Extended Command Parity
**Tasks:**
- Improve flag mappings (e.g., `rosbag record -O` → `ros2 bag record -o`).
- Add shell completion stubs.
- Broaden integration tests to ROS2 Jazzy.

**Done criteria:**
- Common flags mapped.
- Tab completion partially works.
- CI matrix green for both Humble & Jazzy.

---

## Milestone 12 – Community Distribution
**Tasks:**
- Add Homebrew formula (`brew install ros-is-ros2`).
- Package for Arch Linux AUR.
- Write CONTRIBUTING.md and tag `good first issue`.

**Done criteria:**
- Users on macOS/Linux can install outside pipx.
- Community contributors file issues/PRs.

---

## Milestone 13 – Release v1.0.0
**Tasks:**
- Ensure all major ROS1 commands implemented.
- Document limitations (e.g., no `roscore`).
- Publish GitHub Release.
- Post official announcement with demo gifs.

**Done criteria:**
- `pipx install ros-is-ros2` provides full set of shims.
- README parity table complete.
- CI/CD fully automated for release.

---

## Architecture & file structure (just for reference)
```bash
ros-is-ros2/
  pyproject.toml          # build system (hatchling/poetry) + deps
  src/ros_is_ros2/
    __init__.py
    main.py               # common dispatch helpers
    mappings.py           # verb + flag mappings and per-tool adapters
    exec.py               # thin wrapper for subprocess exec & error codes
    detectors.py          # environment detection (ROS_VERSION, which('ros2'), etc.)
    tools/
      rostopic.py
      rosnode.py
      rosservice.py
      rosparam.py
      roslaunch.py
      rosbag.py
      rosmsg.py
      rossrv.py
      rosrun.py
      roscore.py          # prints guidance
  tests/
    unit/
    integration/
  scripts/
    dev_setup.sh
    demo_recordings/      # asciinema scripts for README gifs
  .github/workflows/
    ci.yml
    release.yml
  README.md
  CHANGELOG.md
  LICENSE (Apache-2.0)
  CODE_OF_CONDUCT.md
  CONTRIBUTING.md
```