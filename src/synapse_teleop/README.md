# synapse_teleop

Keyboard/controller teleoperation node for Project Synapse. Publishes `geometry_msgs/msg/Twist` on `/cmd_vel` to manually drive the robot for bring-up and testing.

This package is an alternative to `teleop_twist_keyboard` with:
- Smooth and configurable publish loop
- Deadman timeout (stop on key release)
- Arrow-key support (optional)
- On-the-fly speed scaling

---

## Features

- Linear and angular velocity control via WASD and Arrow keys
- Configurable speed scales and publish rate
- Deadman-stop behavior when keys are released (optional)
- Cross-platform key capture (Linux/macOS/Windows)
- ROS 2 parameters for topic and behavior tuning

---

## Dependencies

- ROS 2 (Humble/Iron)
- Python 3.8+
- `rclpy`, `geometry_msgs`, `std_msgs` (installed via ROS 2)

These are typically available after installing ROS 2. No extra pip requirements are needed for this package.

---

## Build and Install (within a ROS 2 workspace)

1) Place this package under your workspace `src`:
    - Robot/synapse_ws/src/synapse_teleop

2) From the workspace root:

    source /opt/ros/humble/setup.bash
    rosdep install --from-paths src --ignore-src -r -y
    colcon build --symlink-install
    source install/setup.bash

---

## Usage

- Start the node:

    ros2 run synapse_teleop keyboard_teleop

- Default behavior:
  - Publishes to `/cmd_vel` at 20 Hz
  - Linear scale: 0.30 m/s
  - Angular scale: 1.20 rad/s
  - Stops if no key pressed for 0.5 s (deadman timeout)

- Remap topic:

    ros2 run synapse_teleop keyboard_teleop --ros-args -p cmd_vel_topic:=/my_robot/cmd_vel

- Tune parameters:

    ros2 run synapse_teleop keyboard_teleop --ros-args \
      -p linear_scale:=0.4 \
      -p angular_scale:=1.5 \
      -p repeat_rate:=30.0 \
      -p stop_on_key_release:=true \
      -p deadman_timeout:=0.6 \
      -p enable_arrow_keys:=true

---

## Controls

Movement:
- w or Up Arrow: forward
- s or Down Arrow: backward
- a or Left Arrow: rotate left (CCW)
- d or Right Arrow: rotate right (CW)
- SPACE or x: stop (zero velocities)

Speed scaling:
- + or =: increase linear and angular scales by 10%
- - or _: decrease linear and angular scales by 10%
- ]: increase angular scale by 10%
- [: decrease angular scale by 10%

Other:
- h or ?: print help
- q: quit

Notes:
- Uppercase movement keys (W/A/S/D) apply a temporary boost multiplier for snappier maneuvers.

---

## Parameters

All parameters can be set via `--ros-args -p name:=value` or YAML.

- cmd_vel_topic (string, default: "/cmd_vel")
  - Topic to publish `Twist` messages to.

- linear_scale (double, default: 0.30)
  - Maximum forward/backward linear speed in m/s.

- angular_scale (double, default: 1.20)
  - Maximum angular speed (yaw) in rad/s.

- repeat_rate (double, default: 20.0)
  - Publish rate in Hz.

- stop_on_key_release (bool, default: true)
  - If true, publish zero velocity when no key has been pressed for `deadman_timeout` seconds.

- deadman_timeout (double, default: 0.5)
  - Seconds since last keypress to trigger zero-velocity publish when `stop_on_key_release` is true.

- enable_arrow_keys (bool, default: true)
  - Enable arrow key handling on supported terminals.

Example YAML snippet:

    synapse_teleop:
      ros__parameters:
        cmd_vel_topic: "/cmd_vel"
        linear_scale: 0.35
        angular_scale: 1.4
        repeat_rate: 25.0
        stop_on_key_release: true
        deadman_timeout: 0.5
        enable_arrow_keys: true

Launch with parameters from YAML:

    ros2 run synapse_teleop keyboard_teleop --ros-args --params-file /path/to/teleop.yaml

---

## Best Practices and Safety

- Always test teleop in a safe, open area.
- Start with lower speed scales and increase gradually.
- Ensure your base controller respects limits (max velocity/acceleration) and has an e-stop.
- Use `stop_on_key_release` in tight/indoor spaces for additional safety.

---

## Troubleshooting

- No motion:
  - Confirm base driver is running and subscribed to `/cmd_vel`.
  - Check the topic name: `ros2 topic list` and `ros2 topic echo /cmd_vel`.
  - Confirm your terminal focus is on the teleop window.

- Arrow keys not working:
  - Some terminals may not forward arrow sequences correctly. Try another terminal emulator.
  - Set `enable_arrow_keys:=false` and use WASD as a fallback.

- High CPU usage:
  - Reduce `repeat_rate` (e.g., 10–15 Hz).
  - Avoid running multiple teleop nodes simultaneously.

- Windows support:
  - Arrow keys rely on platform support; WASD is recommended universally.

---

## Development

Entry point:
- `synapse_teleop/keyboard_teleop.py` provides the main node implementation.

Console script:
- `keyboard_teleop` (configured in `setup.cfg`)

Contributions:
- Add unit tests (pytest) for parameter handling and command mapping.
- Consider adding controller/gamepad support in a sibling node.

---

## License

Apache-2.0

---

## Changelog

- 0.1.0
  - Initial release with WASD/Arrow control, deadman stop, and speed scaling.
