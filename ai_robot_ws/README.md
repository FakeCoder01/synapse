# AI Pet Robot - The Definitive Guide (ROS 2 Humble)

Welcome to the definitive guide for the AI Pet Robot, a comprehensive ROS 2 Humble project. This document is structured as a complete user manual, covering everything from the high-level architecture and data flows to granular, step-by-step instructions for simulation and real-world hardware deployment.

![RViz View](https://i.imgur.com/8i9hZkL.png)

## Table of Contents
1.  [**Project Philosophy & Goals**](#project-philosophy--goals)
2.  [**Part 1: System Architecture Deep Dive**](#part-1-system-architecture-deep-dive)
    *   [Architectural Principles](#architectural-principles)
    *   [Package Structure](#package-structure)
    *   [Node-by-Node Breakdown](#node-by-node-breakdown)
    *   [Core Workflow 1: The Vision Pipeline](#core-workflow-1-the-vision-pipeline)
    *   [Core Workflow 2: The Conversation Loop](#core-workflow-2-the-conversation-loop)
3.  [**Part 2: The Simulation Environment**](#part-2-the-simulation-environment)
    *   [Prerequisites](#prerequisites)
    *   [Build Instructions](#build-instructions)
    *   [Step-by-Step: Running the Simulation](#step-by-step-running-the-simulation)
4.  [**Part 3: Building and Deploying the Physical Robot**](#part-3-building-and-deploying-the-physical-robot)
    *   [Hardware Bill of Materials](#hardware-bill-of-materials)
    *   [Assembly & Wiring Guide](#assembly--wiring-guide)
    *   [Host Computer & Network Setup](#host-computer--network-setup)
    *   [Firmware Flashing Guide](#firmware-flashing-guide)
    *   [Critical First Run: Mapping a New Environment](#critical-first-run-mapping-a-new-environment)
    *   [Running the Full AI Stack on Hardware](#running-the-full-ai-stack-on-hardware)
5.  [**Part 4: Developer's Guide & Customization**](#part-4-developers-guide--customization)
    *   [File-by-File Code Explanation](#file-by-file-code-explanation)
    *   [How to Customize the AI's Personality](#how-to-customize-the-ais-personality)

---

## Project Philosophy & Goals

This project is an educational tool designed to be a fully transparent, functional, and understandable example of a modern robotics system.

*   **Goal**: To create an autonomous mobile robot that can map its environment, navigate intelligently, and engage in basic, voice-driven social interaction with people it detects and recognizes.
*   **Sim-to-Real**: The codebase is structured to run in a realistic Gazebo simulation and on physical hardware with minimal changes, demonstrating a critical workflow in modern robotics.
*   **Self-Contained AI**: The "brain" is built from scratch. The face recognition uses vector math, and the conversation logic is a rule-based state machine. This avoids dependencies on external, paid cloud services and allows every line of code to be inspected and understood.

---

## Part 1: System Architecture Deep Dive

### Architectural Principles

*   **Separation of Concerns**: The code is split into two packages. `ai_robot_core` is the hardware-agnostic "brain," containing all the application logic. `ai_robot_hardware_drivers` contains the specific code needed to "talk" to the physical motors, camera, etc. This makes the core logic portable to different robot bases.
*   **Modularity**: Nodes are designed to perform a single, clear task (e.g., `memory_manager` only handles the database). They communicate using well-defined ROS 2 topics and services, making the system easy to debug and extend.
*   **Reusability**: Launch files are structured to be reusable. A core `nav2_bringup.launch.py` file handles the complex task of starting the Nav2 stack, and is *included* by both the simulation and hardware launch files, preventing code duplication.

### Package Structure
*   `ai_robot_core`: Contains all primary logic, AI nodes, robot description (URDF), and simulation files.
*   `ai_robot_hardware_drivers`: Contains all code for interfacing with the physical world: the MQTT bridge, camera/audio drivers, and the final hardware launch file.
*   `firmware`: Contains the Arduino code for the ESP8266 microcontroller.

### Node-by-Node Breakdown

#### Core Logic (`ai_robot_core`)
*   **`face_recognition_node`**: The robot's "eyes." It processes raw video, finds faces, generates a unique (but deterministic) "embedding" for each face, and asks the `memory_manager` to identify it.
*   **`memory_manager_node`**: The robot's "hippocampus." It manages the `robot_memory.db` SQLite file. It provides services to look up a face embedding against its database of known people and to store new people.
*   **`emotion_engine_node`**: The robot's "amygdala." It listens to the output of the vision pipeline and sets a simple emotional state: "happy" if it sees a known person, "curious" if it sees a stranger, and "neutral" if it sees no one.
*   **`interaction_manager_node`**: The "frontal cortex." It orchestrates social interactions. It decides when to greet someone, what to say (using its internal rule-based chatbot), and listens for spoken replies from the user.
*   **`person_follower_node`**: A specialized behavior node. When activated, it takes over motor control to visually servo on a person's position in the camera frame.

#### Hardware Drivers (`ai_robot_hardware_drivers`)
*   **`esp8266_bridge_node`**: The "spinal cord." It translates high-level ROS 2 velocity commands (`/cmd_vel`) into low-level PWM motor commands for the ESP8266. It also performs the critical task of **dead-reckoning**—estimating the robot's movement to publish `/odom` and the required TF transform, which is essential for navigation to work on hardware.
*   **`kinect_interface_node`**: A driver for the physical Kinect camera.
*   **`audio_interface_node`**: Manages the host computer's microphone and speakers for STT and TTS.

### Core Workflow 1: The Vision Pipeline
`Camera -> Face Detection -> Memory Lookup -> Emotion & Interaction`
1.  A video frame is published by the **Gazebo Camera** (in sim) or **`kinect_interface_node`** (on hardware).
2.  **`face_recognition_node`** grabs the frame. It finds a face's bounding box.
3.  It generates a unique vector for the face and calls the `/lookup_person` service on the **`memory_manager_node`**.
4.  **`memory_manager_node`** compares this vector to all vectors in its database. If a close match is found, it returns the person's name. If not, it returns "Unknown".
5.  The final result (e.g., "Jane, is_known=true") is published to `/detected_people`.
6.  Both **`emotion_engine_node`** and **`interaction_manager_node`** receive this message and react accordingly (e.g., change emotion to "happy" and formulate a greeting).

### Core Workflow 2: The Conversation Loop
`User Speaks -> STT -> AI Brain -> TTS -> Robot Speaks`
1.  The **`audio_interface_node`** captures audio from the USB microphone and transcribes it to text.
2.  This text is published to `/robot/audio_transcription`.
3.  The **`interaction_manager_node`** receives the text.
4.  It passes the text to its internal `call_llm_api` function, which uses `if/elif/else` logic to select a response.
5.  The chosen response string is published to `/robot_speech_output`.
6.  A launch file remaps this topic to `/robot/tts`, which the **`audio_interface_node`** is listening to.
7.  The `audio_interface_node` receives the text and uses the `pyttsx3` library to generate speech, which is played through the USB speakers.

---

## Part 2: The Simulation Environment

### Prerequisites
*   Ubuntu 22.04 with ROS 2 Humble **Desktop-Full**.
*   `colcon`, `git`.
*   `sudo apt-get update && sudo apt-get install ros-humble-slam-toolbox ros-humble-nav2-bringup`

### Build Instructions
1.  **Clone**: `mkdir -p ai_robot_ws/src && cd ai_robot_ws/src && git clone <repo_url>`
2.  **Install Dependencies**: `cd .. && rosdep install -i --from-path src -y --rosdistro humble`
3.  **Build**: `colcon build --symlink-install`
4.  **Source**: In every new terminal, `cd ~/ai_robot_ws && source install/setup.bash`

### Step-by-Step: Running the Simulation

Always source your workspace first: `cd ~/ai_robot_ws && source install/setup.bash`

#### Step 1. Create a Map (SLAM)
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

## Part 3: Building and Deploying the Physical Robot

### Hardware Bill of Materials
*   **Chassis**: A 4-wheel drive robot chassis with TT motors.
*   **Microcontroller**: 1x ESP8266 (NodeMCU or a similar board).
*   **Motor Driver**: 1x L298N Dual H-Bridge Motor Driver.
*   **Host Computer**: 1x Raspberry Pi 4 (4GB+ recommended) or a similar single-board computer.
*   **Camera**: 1x Microsoft Xbox 360 Kinect (Model 1414 or 1473) and the special USB/Power adapter it requires.
*   **Audio**: 1x standard USB Microphone and 1x standard USB-powered Speaker.
*   **Power**:
    *   A 7.4V-12V battery pack (e.g., a 2S LiPo or a holder for 6x AA batteries) to power the L298N motor driver.
    *   A 5V USB power bank to provide clean power to the Raspberry Pi.
    *   The ESP8266 will be powered directly from the L298N's onboard 5V regulator.

### Assembly & Wiring Guide

This wiring is crucial for the firmware to work correctly. Double-check all connections.

```
+-----------------+      +----------------+      +-----------------+
|   ESP8266       |      | L298N Driver   |      |   Motors        |
| (NodeMCU)       |      |                |      |                 |
|                 |      |                |      |                 |
|      D1 (GPIO5) |----->| IN1            |      |                 |
|      D2 (GPIO4) |----->| IN2            |      |                 |
|      D3 (GPIO0) |----->| ENA (Enable A) |      |                 |
|                 |      |                |      |                 |
|      D5 (GPIO14)|----->| IN3            |      |                 |
|      D6 (GPIO12)|----->| IN4            |      |                 |
|      D7 (GPIO13)|----->| ENB (Enable B) |      |                 |
|                 |      |                |      |                 |
|             GND |<---->| GND            |<---->| Battery (-)     |
|             VIN |<-----| 5V Output      |      |                 |
+-----------------+      |                |      |                 |
                         | 12V Input      |<---->| Battery (+)     |
                         |                |      | (7.4V - 12V)    |
                         |                |      |                 |
                         | OUT1 & OUT2    |----->| Left Motors     |
                         | OUT3 & OUT4    |----->| Right Motors    |
                         +----------------+      +-----------------+
```
**Assembly Notes**:
*   **Motor Wiring**: Wire the two left-side motors in parallel to the `OUT1` and `OUT2` terminals of the L298N. Wire the two right-side motors in parallel to `OUT3` and `OUT4`.
*   **L298N Jumper**: Ensure the 5V regulator jumper on the L298N is **in place**. This enables the `5V Output` terminal that will power the ESP8266.
*   **Common Ground**: The most common point of failure is a missing common ground. The negative terminal of your motor battery, the `GND` on the L298N, and the `GND` on the ESP8266 must all be connected.

### Host Computer & Network Setup
1.  **Install OS**: Install a fresh copy of Ubuntu 22.04 Server on your Raspberry Pi's microSD card.
2.  **Install ROS 2**: Follow the official ROS 2 documentation to install the `ros-humble-ros-base` package.
3.  **Connect Peripherals**:
    *   Connect the Kinect camera's USB adapter to one of the Pi's blue USB 3.0 ports.
    *   Connect the USB microphone and USB speakers to the remaining USB ports.
4.  **Install & Enable MQTT Broker**: The broker is the message hub between the Pi and the ESP8266.
    ```bash
    sudo apt-get update
    sudo apt-get install mosquitto mosquitto-clients
    sudo systemctl enable mosquitto
    sudo systemctl start mosquitto
    ```
5.  **Network Configuration**:
    *   Connect the Raspberry Pi to your WiFi network.
    *   Find the Pi's IP address using `ip a`. Note this down, as you will need it for the firmware.

### Firmware Flashing Guide
1.  **Setup Arduino IDE**:
    *   Download and install the Arduino IDE on your main computer.
    *   Go to `File > Preferences`. In "Additional Boards Manager URLs," add: `http://arduino.esp8266.com/stable/package_esp8266com_index.json`
    *   Go to `Tools > Board > Boards Manager...`, search for `esp8266`, and install the package by ESP8266 Community.
    *   Select your board from the `Tools > Board` menu (e.g., "NodeMCU 1.0 (ESP-12E Module)").
2.  **Install Libraries**:
    *   Go to `Sketch > Include Library > Manage Libraries...`.
    *   Search for and install `PubSubClient` by Nick O'Leary.
    *   Search for and install `ArduinoJson` by Benoit Blanchon.
3.  **Configure and Flash**:
    *   Open the `firmware/esp_firmware_8266.ino` file from this project in the Arduino IDE.
    *   **Crucially, you must edit these three lines** with your own network details:
        ```cpp
        const char *SSID = "YOUR_WIFI_SSID";
        const char *PASSWORD = "YOUR_WIFI_PASSWORD";
        const char *MQTT_SERVER = "RASPBERRY_PI_IP_ADDRESS"; 
        ```
    *   Connect the ESP8266 to your computer via USB.
    *   Select the correct port under `Tools > Port`.
    *   Click the "Upload" button (the right-arrow icon).

### Critical First Run: Mapping a New Environment
You cannot navigate in a new place without first creating a map. This process is called SLAM.

1.  **Place the Robot**: Put your fully assembled, powered-on robot in the starting position of the area you want to map.
2.  **Launch the Mapping Stack**: On your host computer (e.g., Raspberry Pi), run the following command. This special launch file starts the hardware drivers but runs `slam_toolbox` instead of the full navigation stack.
    ```bash
    # Make sure you have sourced your workspace!
    ros2 launch ai_robot_core mapping.launch.py use_sim_time:=false
    ```
    *Notice `use_sim_time:=false`*. This is critical for running on real hardware.

3.  **Launch Keyboard Control**: In a **second terminal** on the host computer:
    ```bash
    # Source your workspace again
    source install/setup.bash
    ros2 launch ai_robot_core teleop.launch.py
    ```
4.  **Build the Map**:
    *   RViz will open on your host computer (or you can run it on a separate desktop connected to the same ROS network).
    *   Focus the `teleop` terminal and use the keys to **slowly and carefully** drive the robot around your entire space.
    *   Watch the `Map` display in RViz. You will see black lines (obstacles), grey areas (explored free space), and white areas (unexplored).
    *   Drive until all the areas the robot needs to access are grey and the walls form a complete, closed loop.

5.  **Save the Map**: Once the map is complete, **do not close the mapping launch**. Open a **third terminal**, navigate to your workspace root, and run the map saver:
    ```bash
    cd ~/ai_robot_ws
    mkdir -p maps
    ros2 run nav2_map_server map_saver_cli -f maps/my_map
    ```
    This saves `maps/my_map.yaml` and `maps/my_map.pgm`. You have now successfully mapped your environment. You can now `Ctrl+C` out of the mapping and teleop launch files.

### Running the Full AI Stack on Hardware

Now that you have a map, you can run the robot in its fully autonomous navigation and interaction mode.

1.  **Verify Dependencies**: Ensure you have installed the hardware-specific Python packages on your host computer:
    ```bash
    sudo apt-get install python3-paho-mqtt python3-pyttsx3 python3-pyaudio
    # WARNING: pykinect2 is difficult to install on Linux. This is a known
    # challenge. You may need to research how to build it from source or
    # use an alternative like the 'ros-humble-libfreenect-camera' package.
    ```
2.  **Launch the Main Hardware Stack**: This is the primary command for running your physical robot. It starts all hardware drivers, the AI stack, and the Nav2 stack in navigation (not mapping) mode.

    **You must provide the full, absolute path to your map file.**

    ```bash
    # Source your workspace
    source install/setup.bash
    
    # Example:
    ros2 launch ai_robot_hardware_drivers hardware_bringup.launch.py map:=/home/sex/ai_robot_ws/maps/my_map.yaml
    ```
    *   **Dissecting the command**:
        *   `ros2 launch ai_robot_hardware_drivers hardware_bringup.launch.py`: This tells ROS to run the main launch file from the hardware driver package.
        *   `map:=/path/to/your/map.yaml`: This is a **launch argument**. It passes the location of your map file into the launch process, where it is then passed to the `map_server` node within the Nav2 stack. Without this, Nav2 will fail to start.

Your robot is now fully operational. It will localize itself in the map and is ready to accept navigation goals from RViz or interact with people it sees.

---

## Part 4: Developer's Guide & Customization

### File-by-File Code Explanation
*   **Launch Files**: The launch directory contains scripts for orchestrating the startup of multiple nodes. The files are split to separate simulation (`full_ai_app.launch.py`) from hardware (`hardware_bringup.launch.py`) and to make core components like navigation (`nav2_bringup.launch.py`) reusable.
*   **Configuration Files**: The `config` directory holds `.yaml` files. These files allow you to change node parameters (like robot speed, MQTT IP addresses, or sensor settings) without editing the Python code itself.
*   **URDF Files**: The `.urdf.xacro` files define the robot's physical shape, joints, and mass. The `.xacro` format allows us to use variables and include files, making the definitions cleaner. The `gazebo_plugins.xacro` file is included to add simulation-only properties like the camera and motor plugins.
*   **Firmware**: The `.ino` files are self-contained Arduino sketches. `esp_firmware_8266.ino` is the complete, functional code for the motor controller.

### How to Customize the AI's Personality
*   **Conversation Logic**: The robot's conversational ability is defined entirely within the `call_llm_api` function in `ai_robot_core/nodes/interaction_manager_node.py`. You can easily add more `elif` conditions to check for different keywords and provide custom responses.
*   **Face Recognition Tuning**: The strictness of face matching is controlled by the `min_distance` variable in `ai_robot_core/nodes/memory_manager_node.py`. Lower this value (e.g., to `0.5`) to make the robot less likely to incorrectly match a new face to a known one.
*   **MQTT Broker**: To change the MQTT broker IP address for the hardware, edit the file `ai_robot_hardware_drivers/config/hardware_params.yaml`.
