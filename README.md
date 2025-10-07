# Project Synapse — An Autonomous AI Robot Companion

Project Synapse is a modular, ROS 2–based, socially intelligent mobile robot that navigates a home, recognizes people, remembers conversations, exhibits emotions, and proactively engages with inhabitants. It is inspired by Anki Vector but designed to be far more capable, extensible, and open.

This repository contains a ROS 2 workspace scaffold and documentation to build, run, and extend the system.

- Modularity: Perception, navigation, cognition, interaction, display, and behaviors are isolated packages.
- Scalability: Add sensors, nodes, and behaviors without breaking core systems.
- Data-Centric: Long-term semantic memory and person profiles power personalization.

Status: Initial workspace scaffolding, launch and package structure. Subsequent commits will add complete node implementations per the project plan.

---

## Workspace Layout

This folder is a ROS 2 workspace (`synapse_ws`). Key packages:

- `synapse_interfaces`: Custom ROS 2 messages/services/actions for persons, memories, and behaviors.
- `synapse_bringup`: Launch files to bring up sensors and baseline system.
- `synapse_base`: Base control and motor interface; subscribes to `/cmd_vel` to drive the robot.
- `synapse_perception`: Camera processing, face detection/recognition, person tracking; publishes `/detected_persons`.
- `synapse_memory`: Cognitive core; SQLite for person profiles and vector DB for conversational memories.
- `synapse_interaction`: Conversation manager, STT/LLM/TTS pipeline, prompt construction, logging.
- `synapse_emotion`: Emotion engine (finite-state machine) with events and decay; publishes `/emotion/state`.
- `synapse_display`: Face/eyes and UI overlay rendering on an LCD/OLED; subscribes to emotion and status.
- `synapse_behaviors`: High-level behaviors: Patrol, Seek-and-Greet, Person Following.
- `synapse_description`: Robot description (URDF/Xacro), meshes, and description launch (robot_state_publisher).
- `synapse_navigation`: SLAM (slam_toolbox), Nav2 launch/config, mapping and navigation utilities.
- `synapse_teleop`: Keyboard/controller teleoperation for bring-up and testing.

Directory tree (abbreviated):

```
synapse_ws/
├─ src/
│  ├─ synapse_interfaces/        # msg/srv/action definitions
│  ├─ synapse_bringup/           # launch/
│  ├─ synapse_base/              # src/
│  ├─ synapse_perception/        # src/
│  ├─ synapse_memory/            # src/
│  ├─ synapse_interaction/       # src/
│  ├─ synapse_emotion/           # src/
│  ├─ synapse_display/           # src/
│  ├─ synapse_behaviors/         # src/
│  ├─ synapse_navigation/        # launch/, config/, src/
│  └─ synapse_teleop/            # src/
```

---

## Recommended Hardware (Summary)

- Compute Unit (SBC)
  - NVIDIA Jetson Orin Nano 8GB: Best for multiple NNs (vision + ASR) with CUDA acceleration.
  - NVIDIA Jetson Xavier NX: Proven balance of performance and power.
  - Raspberry Pi 5 (8GB) + NPU (Intel NCS2/Coral USB): Cost-effective if GPU not required.

- Chassis & Drivetrain
  - 4WD differential base (e.g., Rover/JetBot-style) with quadrature encoders.
  - Motor driver: Roboclaw 2x7A or Cytron MDD10A with encoder interface board.
  - IMU (optional but recommended): Bosch BNO055 or MPU-9250/MPU-6050.

- Primary Sensor (Spatial)
  - 360° LiDAR: RPLIDAR A2M8 (budget), YDLIDAR G4 (mid), SLAMTEC S2 (better indoor range).
  - Alternatively: Hokuyo URG-04LX-UG01 (reliable, narrow FOV).

- Secondary Sensor (Vision/Depth)
  - Intel RealSense D435/D455 (widely supported, ROS 2 driver).
  - Orbbec Astra/Astra+ (ROS drivers available).
  - Luxonis OAK-D (onboard NN, depth AI).

- Audio System
  - 4/6-mic array: ReSpeaker USB Mic Array v2.0 or 4-Mic Pi HAT; MiniDSP UMA-16 for advanced beamforming.

- Interaction/Display
  - 2–3.5" SPI/I2C TFT/OLED (Waveshare 2.8"/3.5" IPS SPI) or a small HDMI screen.

- Power System
  - 3S/4S LiPo or 2S–3S Li-ion pack with BMS.
  - 5V/12V regulators and power distribution board (PD-BEC).
  - Smart charging dock (optional) + contact pads for auto-docking.

Detailed hardware integration and wiring will be documented alongside each driver node.

---

## Prerequisites

- OS: Ubuntu 22.04 LTS
- ROS 2: Humble (preferred) or Iron
- Compiler/Build: colcon, CMake, Python 3.10
- Graphics/Compute: Jetson CUDA/cuDNN (if on Jetson), OpenCV
- Device Drivers: LiDAR, Depth Camera, Microphone array
- Optional Accelerators: Intel NCS2, Coral Edge TPU

Install ROS 2 Humble (Desktop + Dev Tools):

```bash
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo apt update && sudo apt install -y ros-humble-desktop ros-humble-ros-base \
  ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox \
  ros-humble-teleop-twist-keyboard ros-humble-xacro ros-humble-rqt* \
  python3-colcon-common-extensions python3-rosdep python3-vcstool git
sudo rosdep init || true
rosdep update
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

Sensor Drivers (examples; pick those matching your hardware):

```bash
# Intel RealSense
sudo apt install -y ros-humble-realsense2-camera ros-humble-realsense2-description

# RPLIDAR
sudo apt install -y ros-humble-rplidar-ros

# Hokuyo
sudo apt install -y ros-humble-urg-node

# Common vision stack
sudo apt install -y ros-humble-image-common ros-humble-image-transport-plugins
```

Python ecosystem (create a venv, optional but recommended):

```bash
sudo apt install -y python3-venv python3-pip
python3 -m venv ~/synapse_venv
source ~/synapse_venv/bin/activate
pip install --upgrade pip

# Core ML/AI and utilities (to be refined per package):
pip install opencv-python numpy scipy scikit-learn
pip install face-recognition dlib==19.24.2  # Requires build tools; see notes below
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu
pip install sentence-transformers chromadb
pip install pydantic fastapi uvicorn  # internal APIs if needed
pip install sounddevice soundfile vosk  # offline STT option
pip install TTS==0.22.0 or gTTS  # pick one TTS engine
pip install python-dotenv
```

Notes:

- `dlib` may require `sudo apt install build-essential cmake libopenblas-dev liblapack-dev libx11-dev libgtk-3-dev`.
- On Jetson, use JetPack’s OpenCV/CUDA and platform-specific wheels for PyTorch.

---

## Build the Workspace

From the workspace root (this folder):

```bash
# If using a venv:
source ~/synapse_venv/bin/activate

# Source ROS 2
source /opt/ros/humble/setup.bash

# Install dependencies (when package manifests are complete):
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --symlink-install

# Source the overlay
source install/setup.bash
```

Add to ~/.bashrc for convenience:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source $(pwd)/install/setup.bash" >> ~/.bashrc
```

---

## Configuration and Calibration

- TF Frames: `map` -> `odom` -> `base_link` -> `laser` / `camera_link` / `imu_link`
- Base kinematics: Set wheel radius, track width, max velocity/acceleration, and encoder ticks per revolution in `synapse_base`.
- Sensor poses: Configure static transforms in `synapse_bringup` (e.g., `static_transform_publisher`).
- Nav2 configuration: Costmaps, inflation, planner/controller parameters in `synapse_navigation/config`.
- Camera/LiDAR: Verify device IDs, frame IDs, resolution, and FPS.

---

## Running the System (By Stages)

Below are standard bring-up flows aligned with the development steps.

Robot Description (URDF/Xacro):

- Visualize/publish the robot model and TF tree:
  - ros2 launch synapse_description description.launch.py
- In the all-in-one bringup this runs by default; you can toggle with:
  - ros2 launch synapse_bringup robot_bringup.launch.py start_description:=true

### Step 1: Foundation & Basic Mobility

Bring up the base and drive with keyboard to verify motion.

```bash
# Terminal 1
source install/setup.bash
ros2 run synapse_base base_driver_node

# Terminal 2 (teleop)
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard  # or:
ros2 run synapse_teleop keyboard_teleop
```

Expected:

- `synapse_base` subscribes `/cmd_vel` and drives motors.
- Odometry published (`/odom`) if available.

### Step 2: Perception & Sensing

Start sensors (LiDAR + Depth Camera):

```bash
# Example: RPLIDAR
ros2 launch rplidar_ros rplidar.launch.py serial_baudrate:=115200 serial_port:=/dev/ttyUSB0 frame_id:=laser

# Example: Intel RealSense
ros2 launch realsense2_camera rs_launch.py align_depth:=true pointcloud.enable:=true

# Or use unified bringup once provided:
ros2 launch synapse_bringup sensors.launch.py
```

Verify topics:

- `/scan` (sensor_msgs/LaserScan)
- `/camera/color/image_raw`, `/camera/depth/image_rect_raw`, `/camera/depth/color/points` (or `/points2`)

### Step 3: Autonomous Navigation (SLAM + Nav2)

Mapping:

```bash
# Start SLAM with slam_toolbox
ros2 launch slam_toolbox online_async_launch.py

# Visualize in RViz2
rviz2
# Add map, TF, LaserScan, Odometry; drive around with teleop to build map.

# Save map
ros2 run nav2_map_server map_saver_cli -f ~/maps/home_map
```

Navigation:

```bash
# Bring up Nav2 with your saved map
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=false map:=~/maps/home_map.yaml

# Or use a curated launch:
ros2 launch synapse_navigation nav2_bringup.launch.py map:=~/maps/home_map.yaml
```

Send goals from RViz2 via “2D Nav Goal” or via action:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: { header: {frame_id: 'map'}, pose: { position: {x: 1.0, y: 0.5, z: 0.0}, orientation: {z: 0.0, w: 1.0}}}}"
```

### Step 4: Cognitive Core (Memory & Person Profiles)

Start the memory service:

```bash
ros2 run synapse_memory memory_server
```

APIs (ROS 2 services; details in `synapse_interfaces`):

- `add_person(name, face_encoding, first_seen_date, notes)`
- `get_person_by_face(face_encoding)`
- `add_conversation_memory(person_id, text, embedding, timestamp)`
- `retrieve_relevant_memories(person_id, query_text)`

Configuration:

- SQLite database path: `~/.synapse/memory.db`
- Vector DB (ChromaDB) path: `~/.synapse/chroma/`

### Step 5: Vision Intelligence

Run perception node:

```bash
ros2 run synapse_perception face_perception_node
```

Expected:

- Subscribes: camera image stream (e.g., `/camera/color/image_raw`)
- Publishes: `/detected_persons` with bounding boxes, IDs (`UNKNOWN` if not matched), and scores.
- Calls memory server to resolve identities by face encodings.

### Step 6: Interaction & Personality

Environment:

```bash
# Choose your provider and set keys before launching interaction stack
export LLM_PROVIDER=openai   # or: google
export OPENAI_API_KEY=...
export GOOGLE_API_KEY=...
export SYNAPSE_TTS_ENGINE=coqui  # or: gtts
export SYNAPSE_STT_ENGINE=vosk   # or: whisper, external API
```

Launch:

```bash
ros2 run synapse_emotion emotion_engine
ros2 run synapse_display face_display_node
ros2 run synapse_interaction conversation_manager
```

Pipeline:

- Perception detects a known person nearby.
- Conversation Manager activates mic, performs STT, queries memory for relevant context, crafts LLM prompt with emotion state, calls LLM, does TTS, and appends memory.

### Step 7: Proactive & Advanced Behaviors

Patrol / Seek-and-Greet / Person Following:

```bash
ros2 run synapse_behaviors behavior_manager
```

- Patrol: cycles waypoints (“living_room”, “kitchen”, etc.).
- Seek-and-Greet: interrupts patrol if a known person appears; approaches at safe distance; initiates greeting.
- Person Following: sets moving goal in Nav2 from tracked person position.

---

## Key Topics and Interfaces (Overview)

- Base/Control:
  - `/cmd_vel` (geometry_msgs/Twist)
  - `/odom` (nav_msgs/Odometry)
  - `/tf`, `/tf_static` (tf2_msgs/TFMessage)
- Sensors:
  - `/scan` (sensor_msgs/LaserScan)
  - `/camera/*` (sensor_msgs/Image, sensor_msgs/PointCloud2)
- Perception:
  - `/detected_persons` (synapse_interfaces/msg/DetectedPersons)
- Memory:
  - Services: `AddPerson`, `GetPersonByFace`, `AddConversationMemory`, `RetrieveRelevantMemories`
- Interaction:
  - `/conversation/transcript`, `/conversation/response` (std_msgs/String)
  - `/emotion/state` (synapse_interfaces/msg/EmotionState)
- Navigation:
  - Nav2 actions: `/navigate_to_pose` (nav2_msgs/NavigateToPose)
  - `/amcl_pose` or SLAM pose topic
- Behaviors:
  - `/behavior/state`, `/behavior/event`

Message/service definitions live in `synapse_interfaces`.

---

## Development Notes

- Coding Standards: Python (PEP 8), C++ (C++17), ROS 2 best practices (composable nodes where possible).
- Logging: Use `rclcpp`/`rclpy` logging with structured context; avoid excessive verbosity on SBCs.
- Parameters: Expose node params via YAML and declare parameters in code; use lifecycle nodes for critical stacks.
- Testing: Include unit tests (pytest/rostest) and bag short datasets for regression.
- Performance: Use image transport for camera; throttle expensive computations; leverage GPU/NPU where available.

---

## Troubleshooting

- Permissions: Add your user to `dialout` for serial LiDAR/motor drivers.
  ```bash
  sudo usermod -a -G dialout $USER
  ```
- Audio:
  - Verify ALSA devices: `arecord -l`, `aplay -l`
  - Set default mic/speaker via `pavucontrol` or `.asoundrc`
- RealSense:
  - If frames drop, reduce FPS and resolution; turn off IR if not needed.
- Nav2 fails to plan:
  - Check TF (no jumps), footprint size, costmap inflation, and map frame consistency.
- Face recognition poor:
  - Ensure good lighting, increase input resolution, collect multiple embeddings per person.

---

## Security & Privacy

- Store API keys in environment variables or `.env` not committed to VCS.
- Encrypt or restrict access to `~/.synapse` if storing personal data.
- Provide opt-in, data export, and wipe commands for all memories.

---

## Roadmap

- Full node implementations and launch compositions per package.
- Robust person re-identification (multi-view embeddings).
- On-device STT/TTS optimizations for offline operation.
- Auto-docking and autonomous charging.
- Multi-room map management with semantic labels.

---

## License

To be determined. For now, assume all code is provided under an OSI-approved license compatible with ROS 2 packages.

---

## Acknowledgments

Built on top of the ROS 2 ecosystem, Nav2, slam_toolbox, OpenCV, and the open-source AI community.
