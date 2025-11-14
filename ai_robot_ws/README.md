# AI Pet Robot - Complete ROS 2 Documentation

Welcome to the complete documentation for the AI Pet Robot, a comprehensive ROS 2 Humble project. This document covers every aspect of the system, from high-level architecture to detailed, step-by-step guides for simulation and real-world hardware deployment.

![RViz View](https://i.imgur.com/8i9hZkL.png)

## Table of Contents
1.  [**Project Philosophy**](#project-philosophy)
2.  [**System Architecture**](#system-architecture)
    *   [Package Structure](#package-structure)
    *   [High-Level Diagram](#high-level-diagram)
    *   [Node-by-Node Breakdown](#node-by-node-breakdown)
3.  [**Core Workflows & Data Flow**](#core-workflows--data-flow)
    *   [Vision & Perception Pipeline](#vision--perception-pipeline)
    *   [Navigation & Control Flow](#navigation--control-flow)
    *   [Interactive Conversation Loop](#interactive-conversation-loop)
4.  [**File Deep Dive**](#file-deep-dive)
    *   [Launch Files Explained](#launch-files-explained)
    *   [Configuration Files Explained](#configuration-files-explained)
    *   [URDF Structure Explained](#urdf-structure-explained)
    *   [Firmware Explained](#firmware-explained)
5.  [**Part 1: Simulation Guide**](#part-1-simulation-guide)
    *   [Prerequisites](#prerequisites)
    *   [Build Instructions](#build-instructions)
    *   [Step-by-Step: Running the Simulation](#step-by-step-running-the-simulation)
6.  [**Part 2: Real Hardware Guide**](#part-2-real-hardware-guide)
    *   [Bill of Materials](#bill-of-materials)
    *   [Hardware Assembly & Wiring](#hardware-assembly--wiring)
    *   [Host Computer & Network Setup](#host-computer--network-setup)
    *   [Firmware Flashing](#firmware-flashing)
    *   [Step-by-Step: Running on the Real Robot](#step-by-step-running-on-the-real-robot)
7.  [**Part 3: Customization**](#part-3-customization)

---

## Project Philosophy

This project was created to be a fully self-contained, functional, and understandable example of a modern robotics system using ROS 2. Key principles are:

*   **Sim-to-Real Parity**: The system is designed to work almost identically in a realistic Gazebo simulation and on physical hardware, which is a critical practice in modern robotics development.
*   **No Black Boxes**: The AI is implemented from scratch with clear, readable Python. The face recognition uses vector comparison and the conversation logic is rule-based, requiring no external APIs, cloud services, or pre-trained models that you can't inspect.
*   **Clear Separation of Concerns**: The workspace is divided into a `core` logic package and a `hardware_drivers` package, demonstrating how to keep your main application logic separate from hardware-specific code.

---

## System Architecture

### Package Structure

The workspace is organized into two key packages:

*   `ai_robot_core`: This is the "brain and soul" of the robot. It is hardware-agnostic and contains:
    *   The robot's physical definition (URDF).
    *   All AI and logic nodes (perception, memory, interaction).
    *   The simulation world and launch files.
    *   The core Nav2 configuration.
*   `ai_robot_hardware_drivers`: This package is the bridge to the physical world. It contains:
    *   ROS 2 nodes that drive specific hardware (Kinect camera, motor controller).
    *   The MQTT bridge node that communicates with the ESP8266 microcontroller.
    *   The launch file needed to run the robot on real hardware.

### High-Level Diagram

```
+---------------------------------------------------------------------------------+
|                                  HOST COMPUTER                                  |
|                               (Laptop or Raspberry Pi)                          |
|---------------------------------------------------------------------------------|
|                                                                                 |
|    +--------------------------+         +---------------------------------+     |
|    |      AI STACK            |         |      NAVIGATION STACK (Nav2)    |     |
|    | (ai_robot_core)          |         |      (ai_robot_core)            |     |
|    | - Interaction Manager    |         | - Planner, Controller, AMCL     |     |
|    | - Emotion Engine         |         | - Map Server                    |     |
|    | - Memory Manager         |         +---------------------------------+     |
|    | - Face Recognition       |                                                 |
|    +--------------------------+                                                 |
|                                                                                 |
|         ^               |                ^                            |         |
|         |               v                |                            v         |
|  (Audio/Vision)  (Speech/Control)   (Map/Odom)                  (/cmd_vel)     |
|         |               |                |                            |         |
|                                                                                 |
|    +-----------------------------------------------------------------------+    |
|    |                     HARDWARE ABSTRACTION LAYER                        |    |
|    |                  (ai_robot_hardware_drivers)                          |    |
|    |-----------------------------------------------------------------------|    |
|    | +-----------------+  +-----------------+  +-------------------------+ |    |
|    | | Kinect Driver   |  | Audio Interface |  | ESP8266 MQTT Bridge     | |    |
|    | +-----------------+  +-----------------+  +-------------------------+ |    |
|    +-----------------------------------------------------------------------+    |
|                                                                                 |
+--------------------------------|----------------|----------------|---------------+
                                 |                |                |
                        (USB)    |        (USB)   |        (USB)   |        (WiFi)
                                 v                v                v
+-----------------+   +-----------------+   +-----------------+   +-----------------+
| Kinect Camera   |   |   Microphone    |   |    Speakers     |   | ESP8266 & L298N |
+-----------------+   +-----------------+   +-----------------+   +-----------------+
```

### Node-by-Node Breakdown

#### `ai_robot_core` Nodes:
*   **`face_recognition_node`**:
    *   **Purpose**: The robot's "eyes". It processes raw video frames to find and identify people.
    *   **Subscribes**: `/camera/image_raw` (Raw video from Gazebo or Kinect).
    *   **Publishes**: `/annotated_image` (The video feed with boxes/names drawn on it), `/detected_people` (A structured message containing data for each person found).
    *   **Clients**: Uses `/lookup_person` and `/register_person` services from the `memory_manager_node`.
*   **`memory_manager_node`**:
    *   **Purpose**: The robot's "long-term memory". It manages the SQLite database.
    *   **Services Provided**:
        *   `/lookup_person`: Compares a face "embedding" to all known faces in the DB to find a match.
        *   `/register_person`: Adds a new, unknown person to the database.
        *   `/get_conversation`, `/store_conversation`: Manages conversation history.
*   **`emotion_engine_node`**:
    *   **Purpose**: The robot's "heart". It determines the robot's emotional state.
    *   **Subscribes**: `/detected_people`.
    *   **Publishes**: `/robot_emotion` (A string: "neutral", "happy", or "curious").
*   **`interaction_manager_node`**:
    *   **Purpose**: The robot's "consciousness". It orchestrates greetings and conversations.
    *   **Subscribes**: `/detected_people`, `/robot/audio_transcription` (user's speech).
    *   **Publishes**: `/robot_speech_output` (what the robot should say).
    *   **Function Calls**: Contains the `call_llm_api` function, a rule-based chatbot that generates responses.
*   **`person_follower_node`**:
    *   **Purpose**: Enables the "follow me" behavior.
    *   **Subscribes**: `/detected_people`, `/follow_person_start`, `/follow_person_stop`.
    *   **Publishes**: `/cmd_vel` (Overrides Nav2 to control the robot's movement).

#### `ai_robot_hardware_drivers` Nodes:
*   **`esp8266_bridge_node`**:
    *   **Purpose**: The critical link to the robot's motors and low-level sensors.
    *   **Subscribes**: `/cmd_vel` (from Nav2 or teleop).
    *   **Publishes**: `/odom` and the `odom`->`base_footprint` TF transform (by estimating from velocity commands), `/sensor/ir_left`.
    *   **MQTT**: Publishes motor commands to the ESP8266 and receives sensor data.
*   **`kinect_interface_node`**:
    *   **Purpose**: Driver for the physical Kinect camera.
    *   **Publishes**: `/camera/camera/image_raw` and `/camera/depth/image_raw`.
*   **`audio_interface_node`**:
    *   **Purpose**: Manages Text-to-Speech (TTS) and Speech-to-Text (STT) using the host computer's audio devices.
    *   **Subscribes**: `/robot/tts` (text to be spoken).
    *   **Publishes**: `/robot/audio_transcription` (text transcribed from user speech).

---

## Core Workflows & Data Flow

### Vision & Perception Pipeline
This is how the robot "sees" and "recognizes" a person.

1.  **Image Capture**: The **Gazebo Camera Plugin** (in sim) or the **`kinect_interface_node`** (on hardware) publishes a raw video frame to `/camera/image_raw`.
2.  **Face Detection**: The **`face_recognition_node`** receives the image. It uses an OpenCV Haar Cascade to find the bounding box of a face.
3.  **Recognition**:
    *   For the detected face, it generates a random vector (a fake "embedding").
    *   It calls the `/lookup_person` service on the **`memory_manager_node`**, sending this embedding.
4.  **Memory Lookup**:
    *   The **`memory_manager_node`** receives the request. It compares the embedding to all embeddings stored in its `robot_memory.db` database.
    *   If a close match is found (within a distance threshold), it returns the known person's ID and name.
    *   If no match is found, it returns an "unknown" person status.
5.  **Registration**: If the `face_recognition_node` receives an "unknown" status, it immediately calls the `/register_person` service to save the new face's embedding to the database, so it will be recognized next time.
6.  **Broadcast**: The `face_recognition_node` publishes the final data (ID, name, bounding box) to the `/detected_people` topic for other AI nodes to use.

### Navigation & Control Flow
This is how the robot moves from point A to point B.

1.  **Goal Setting**: A user sets a goal in RViz using the "Nav2 Goal" tool.
2.  **Global Plan**: The **Nav2 Planner** creates a high-level path from the robot's current location to the goal, avoiding known obstacles from the map. This is published to `/plan`.
3.  **Local Control**: The **Nav2 Controller** receives this path. Its job is to generate immediate, short-term velocity commands to follow the path while avoiding immediate, new obstacles (detected by the laser scan).
4.  **Velocity Command**: The Controller publishes a `geometry_msgs/Twist` message to the `/cmd_vel` topic. This message says "move forward at X m/s and rotate at Y rad/s".
5.  **Hardware Abstraction**:
    *   **In Simulation**: The **Gazebo Diff Drive Plugin** reads `/cmd_vel` and directly moves the simulated wheels. It also publishes the ground-truth odometry to `/odom`.
    *   **On Hardware**: The **`esp8266_bridge_node`** reads `/cmd_vel`. It converts the Twist message into PWM values for the left and right motors (e.g., `{'fl': 150, 'fr': 150, ...}`).
6.  **Motor Command**: The bridge node publishes this command as a JSON string to the `robot/motor_cmd` MQTT topic.
7.  **Execution**: The **ESP8266** receives the MQTT message and sets the L298N motor driver pins to the specified PWM values, causing the robot to move.

### Interactive Conversation Loop
This is how the robot has a two-way conversation.

1.  **User Speaks**: The user speaks into the USB microphone connected to the host computer.
2.  **Speech-to-Text (STT)**: The **`audio_interface_node`** is constantly listening. It captures the audio and uses the `speech_recognition` library to transcribe it into text.
3.  **Publish Transcription**: The node publishes the transcribed text as a string to the `/robot/audio_transcription` topic.
4.  **Brain Receives Input**: The **`interaction_manager_node`** receives this text.
5.  **Generate Response**: It passes the text to its internal `call_llm_api` function. This function checks for keywords (e.g., "hello", "your name") and selects a pre-programmed response.
6.  **Publish Response**: The `interaction_manager_node` publishes the chosen response string to the `/robot_speech_output` topic.
7.  **Remapping**: A `<remap>` rule in the launch file redirects the `/robot_speech_output` topic to `/robot/tts`, which the `audio_interface_node` is listening to.
8.  **Text-to-Speech (TTS)**: The **`audio_interface_node`** receives the text on `/robot/tts`. It uses the `pyttsx3` library to convert this text into audible speech.
9.  **Robot Speaks**: The generated audio is played through the USB speakers connected to the host computer.

---

## File Deep Dive

This section clarifies the purpose of key files, especially those that seem similar.

### Launch Files Explained

The launch files are organized to maximize reuse and separate simulation from reality.

*   **`ai_robot_core/launch/mapping.launch.py`**
    *   **Purpose**: To create a map of an environment using SLAM.
    *   **What it Does**: Launches Gazebo (sim), teleop (keyboard driving), and `slam_toolbox`. It's a focused tool for a specific task.

*   **`ai_robot_core/launch/navigation.launch.py`**
    *   **Purpose**: To run autonomous navigation *in the simulation*.
    *   **What it Does**: Launches Gazebo (sim) and includes the reusable `nav2_bringup.launch.py`.

*   **`ai_robot_core/launch/full_ai_app.launch.py`**
    *   **Purpose**: The main entry point for the *simulation*.
    *   **What it Does**: Launches `navigation.launch.py` (which brings up the sim and Nav2) and then adds all the AI brain nodes on top.

*   **`ai_robot_hardware_drivers/launch/hardware_bringup.launch.py`**
    *   **Purpose**: The main entry point for the *real robot*. This is the hardware equivalent of `full_ai_app.launch.py`.
    *   **Key Difference**: It does **NOT** launch Gazebo. Instead, it launches the hardware driver nodes (`esp8266_bridge`, `kinect_interface`, etc.). It then includes the same reusable `nav2_bringup.launch.py` but tells it `use_sim_time:=false`.

*   **`ai_robot_core/launch/nav2_bringup.launch.py` (The Reusable Component)**
    *   **Purpose**: This file was created specifically to avoid duplicating the Nav2 launch logic.
    *   **What it Does**: It contains only the logic for starting the Nav2 stack and RViz. Both the simulation (`navigation.launch.py`) and hardware (`hardware_bringup.launch.py`) launch files *include* this file, passing it the correct parameters (`map`, `use_sim_time`). This is a best practice.

### Configuration Files Explained

*   `ai_robot_core/config/nav2_params.yaml`: Contains dozens of parameters for every part of the Nav2 stack. This is the primary configuration for robot navigation behavior (speed, tolerances, etc.).
*   `ai_robot_hardware_drivers/config/hardware_params.yaml`: A separate, smaller file for hardware-specific settings. Currently, it only holds the MQTT broker IP address, but you could add other hardware settings here to keep them separate from the core navigation logic.

### URDF Structure Explained

*   `ai_robot.urdf.xacro`: Defines the robot's physical properties: links (chassis, wheels, camera body) and joints. This file is used for both simulation and hardware (to display the model in RViz).
*   `gazebo_plugins.xacro`: Contains `<plugin>` blocks that are **only understood by Gazebo**. This includes the differential drive controller and the simulated 3D camera. This file is included by the main URDF but is ignored by everything except Gazebo, which is why the simulation works.

### Firmware Explained

*   `firmware/esp_firmware_8266.ino`: This is the complete code for the motor controller. Its only job is to connect to WiFi/MQTT, listen for motor power commands (e.g., `{'fl': 200, 'fr': -200, ...}`), and set the motor PWMs accordingly. It has no complex logic.
*   `firmware/esp_firmware_32_audio.ino`: This is a **template file only**. It shows the structure for a potential future project where audio is processed on a dedicated ESP32. In our current, functional project, audio is handled on the host computer by the `audio_interface_node`.

---

## Part 1: Simulation Guide
*(This section is a more detailed version of the previous README)*

### Prerequisites
*   Ubuntu 22.04 with ROS 2 Humble **Desktop-Full** installed.
*   `colcon`, `git`, and other standard ROS 2 development tools.
*   Install the Nav2 and SLAM packages:
    ```bash
    sudo apt-get update
    sudo apt-get install ros-humble-slam-toolbox ros-humble-nav2-bringup
    ```

### Build Instructions
1.  **Clone**: `mkdir -p ai_robot_ws/src && cd ai_robot_ws/src && git clone <repo_url>`
2.  **Install Dependencies**: `cd .. && rosdep install -i --from-path src -y --rosdistro humble`
3.  **Build**: `colcon build --symlink-install`

### Step-by-Step: Running the Simulation

Always source your workspace first: `cd ~/ai_robot_ws && source install/setup.bash`

#### Step 1. Create and Save the Map
1.  **Launch Mapping**:
    ```bash
    ros2 launch ai_robot_core mapping.launch.py
    ```
2.  **Launch Keyboard Control**: In a **new terminal**:
    ```bash
    ros2 launch ai_robot_core teleop.launch.py
    ```
3.  **Drive**: Focus the teleop terminal and use the keys to drive the robot around the Gazebo world. Watch the map build in RViz.
4.  **Save**: When the map is complete, save it from your workspace directory (`~/ai_robot_ws`):
    ```bash
    mkdir -p maps
    ros2 run nav2_map_server map_saver_cli -f maps/my_map
    ```

#### Step 2. Run the Full AI Simulation
This single command uses your saved map to launch the robot in a fully autonomous, interactive state.
```bash
ros2 launch ai_robot_core full_ai_app.launch.py map:=$(pwd)/maps/my_map.yaml
```

---

## Part 2: Real Hardware Guide
*(This section is a more detailed version of the previous README)*

### Bill of Materials
*(Same as previous README)*

### Hardware Assembly & Wiring
*(Same as previous README, with diagram)*

### Host Computer & Network Setup
1.  **Install OS**: Install Ubuntu 22.04 Server on your Raspberry Pi.
2.  **Install ROS 2**: Install ROS 2 Humble (the `ros-base` variant is sufficient).
3.  **Connect Peripherals**: Connect the Kinect, USB microphone, and speakers to the Pi.
4.  **Install & Enable MQTT Broker**:
    ```bash
    sudo apt-get update
    sudo apt-get install mosquitto mosquitto-clients
    sudo systemctl enable mosquitto
    sudo systemctl start mosquitto
    ```
5.  **Network**: Connect the Pi to WiFi and get its IP address with `ip a`.

### Firmware Flashing
*(Same as previous README)*

### Step-by-Step: Running on the Real Robot
1.  **Build Workspace**: On the Pi, follow the same build instructions as the simulation.
2.  **Install Hardware Dependencies**:
    ```bash
    sudo apt-get install python3-paho-mqtt python3-pyttsx3 python3-pyaudio
    # WARNING: pykinect2 is difficult to install on Linux. This is a known
    # challenge. You may need to research how to build it from source or
    # use an alternative like the 'ros-humble-libfreenect-camera' package.
    ```
3.  **Create a Map**: Follow the same SLAM process as the simulation, but use the hardware launch file: `ros2 launch ai_robot_hardware_drivers hardware_bringup.launch.py ...` (with a mapping config).
4.  **Launch the Full Hardware Stack**:
    ```bash
    # Source your workspace
    source install/setup.bash
    
    # Launch the hardware bringup with your saved map
    ros2 launch ai_robot_hardware_drivers hardware_bringup.launch.py map:=/path/to/your/maps/my_map.yaml
    ```

---

## Part 3: Customization
*   **Conversation Logic**: To change how the robot talks, edit the `call_llm_api` function in `ai_robot_core/nodes/interaction_manager_node.py`.
*   **Face Recognition Tuning**: To make face matching more or less strict, change the `min_distance` threshold in the `lookup_person_callback` function in `ai_robot_core/nodes/memory_manager_node.py`. A lower value (e.g., 0.5) is stricter.
*   **MQTT Broker**: To change the MQTT broker IP address for the hardware, edit the file `ai_robot_hardware_drivers/config/hardware_params.yaml`.
