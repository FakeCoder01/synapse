# Project Synapse — src/README

This directory contains the default, in-tree ROS 2 packages for Project Synapse and guidance for adding external dependencies via a workspace-level .repos file and colcon defaults.

Use this document to:
- Understand the default packages shipped in `src/`
- Add external repositories with a `.repos` file (vcs YAML)
- Configure workspace build defaults via `colcon-defaults.yaml`
- Ignore/enable packages during builds

---

## Default packages in src/

The following packages are included by default in this workspace:

- synapse_interfaces
  - Custom messages/services for persons, emotions, and memory APIs.
- synapse_bringup
  - Launch files to start sensors, core nodes, and system compositions.
- synapse_base
  - Diff-drive base driver that subscribes to `/cmd_vel` and (optionally) publishes odom/TF.
- synapse_perception
  - Camera ingestion; face detection/recognition; publishes `/detected_persons`.
- synapse_memory
  - Cognitive core services (SQLite person DB + vector DB for conversational memory).
- synapse_interaction
  - Conversation manager: STT → Memory → LLM → TTS pipeline with emotion context.
- synapse_emotion
  - Emotion engine (finite-state machine) publishing `/emotion/state`.
- synapse_display
  - Face/eyes display renderer; consumes emotion state.
- synapse_behaviors
  - High-level autonomy: Patrol, Seek-and-Greet, Person Following.
- synapse_navigation
  - Nav2 and SLAM bringup/configuration utilities.
- synapse_teleop
  - Keyboard teleoperation node publishing `/cmd_vel`.

You can temporarily exclude any package from builds by adding a `COLCON_IGNORE` file inside that package directory.

Example:
  touch src/some_package/COLCON_IGNORE

---

## Workspace .repos placeholder (vcs YAML)

Place a `.repos` file at the workspace root (e.g., `synapse_ws/synapse.repos`) to fetch external dependencies that are not installed via your package manager.

Example `synapse.repos` template (edit as needed):

  repositories:
    slam_toolbox:
      type: git
      url: https://github.com/SteveMacenski/slam_toolbox.git
      version: humble
    realsense2_camera:
      type: git
      url: https://github.com/IntelRealSense/realsense-ros.git
      version: ros2
    rplidar_ros:
      type: git
      url: https://github.com/Slamtec/rplidar_ros.git
      version: ros2

Usage:

- From the workspace root (the parent of this src/ directory):

    # Import external repositories into src/
    vcs import src < synapse.repos

    # Install dependencies (after the repositories are present)
    rosdep install --from-paths src --ignore-src -r -y

    # Build
    colcon build --symlink-install

Notes:
- The `.repos` file format is standard vcs YAML. You can add/remove repositories as your hardware and features evolve.
- Prefer installing drivers via your OS package manager when available (e.g., ROS 2 binary packages), and only use `.repos` for sources you need to build from source.

---

## Colcon defaults (workspace-level)

You can add `colcon-defaults.yaml` at the workspace root to establish consistent build/test/launch defaults across developers and CI.

Example `colcon-defaults.yaml`:

  build:
    symlink-install: true
    # Example: build only a subset (uncomment to use)
    # packages-select:
    #   - synapse_interfaces
    #   - synapse_base
    cmake-args: []
    event-handlers:
      # Show summarized build status
      - console_cohesion+
  test:
    # Increase test timeout if your environment is resource constrained
    pytest-args: []
  # Optional: defaults for 'colcon test-result', 'colcon list', etc. can be added here

Usage examples:

  # Build with defaults
  colcon build

  # Override a default just for this invocation
  colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

---

## Ignoring/including packages

- Ignore a package: create an empty `COLCON_IGNORE` file inside that package’s folder.
- Re-enable: delete the `COLCON_IGNORE` file.
- Select packages to build:
    colcon build --packages-select synapse_base synapse_interfaces

---

## Recommended root layout

Your workspace is expected to look like:

  synapse_ws/
  ├─ synapse.repos                 # (optional) external repos list (vcs YAML)
  ├─ colcon-defaults.yaml          # (optional) colcon default arguments
  ├─ src/                          # (this directory)
  │  ├─ synapse_interfaces/
  │  ├─ synapse_bringup/
  │  ├─ synapse_base/
  │  ├─ synapse_perception/
  │  ├─ synapse_memory/
  │  ├─ synapse_interaction/
  │  ├─ synapse_emotion/
  │  ├─ synapse_display/
  │  ├─ synapse_behaviors/
  │  ├─ synapse_navigation/
  │  └─ synapse_teleop/
  └─ install/ build/ log/          # created by colcon

---

## Quick start

From the workspace root:

  # (1) Optionally import external repos
  vcs import src < synapse.repos

  # (2) Resolve dependencies
  rosdep install --from-paths src --ignore-src -r -y

  # (3) Build
  colcon build --symlink-install

  # (4) Source the overlay
  source install/setup.bash

Tip: Add the source line to your shell startup for convenience.

---

## Notes

- If you use multiple hardware configurations, consider keeping variant `.repos` files (e.g., `synapse.jetson.repos`, `synapse.pi.repos`) and import the one matching the target platform.
- Keep `.repos` small and focused; prefer binary packages when stable releases exist for your ROS 2 distribution.
- For CI, commit `colcon-defaults.yaml` with stable, reproducible settings.
