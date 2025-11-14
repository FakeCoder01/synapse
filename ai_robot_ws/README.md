# The AI Pet Robot: The Definitive Manual

**Version 1.0**

Welcome to the definitive guide and manual for the AI Pet Robot, a comprehensive ROS 2 Humble project. This document is structured as a complete, multi-chapter book, covering everything from foundational robotics concepts and high-level architecture to a line-by-line explanation of the code, and provides granular, step-by-step tutorials for running the robot in simulation and deploying it on real-world hardware.

![RViz View](https://i.imgur.com/8i9hZkL.png)

## Table of Contents

### **Chapter 1: Introduction & Core Concepts**

- [1.1. Project Vision & Philosophy](#11-project-vision--philosophy)
- [1.2. Who is this Project For?](#12-who-is-this-project-for)
- [1.3. Core Robotics Concepts Covered](#13-core-robotics-concepts-covered)
- [1.4. High-Level System Diagram](#14-high-level-system-diagram)

### **Chapter 2: The Codebase: A Guided Tour**

- [2.1. Workspace Philosophy: Core vs. Drivers](#21-workspace-philosophy-core-vs-drivers)
- [2.2. Package Deep Dive: `ai_robot_core`](#22-package-deep-dive-ai_robot_core)
- [2.3. Package Deep Dive: `ai_robot_hardware_drivers`](#23-package-deep-dive-ai_robot_hardware_drivers)
- [2.4. Message Package: `ai_robot_msgs`](#24-message-package-ai_robot_msgs)
- [2.5. Firmware Deep Dive: `esp_firmware_8266.ino`](#25-firmware-deep-dive-esp_firmware_8266ino)
- [2.6. Core Workflow 1: The Vision Pipeline](#26-core-workflow-1-the-vision-pipeline)
- [2.7. Core Workflow 2: The Conversation Loop](#27-core-workflow-2-the-conversation-loop)

### **Chapter 3: The Simulation: A Virtual Proving Ground**

- [3.1. Why Simulate? The Importance of a Digital Twin](#31-why-simulate-the-importance-of-a-digital-twin)
- [3.2. Prerequisites for Simulation](#32-prerequisites-for-simulation)
- [3.3. Build Instructions](#33-build-instructions)
- [3.4. Tutorial: Creating a Map with SLAM](#34-tutorial-creating-a-map-with-slam)
- [3.5. Tutorial: Autonomous Navigation in Simulation](#35-tutorial-autonomous-navigation-in-simulation)
- [3.6. Tutorial: Full AI Interaction in Simulation](#36-tutorial-full-ai-interaction-in-simulation)

### **Chapter 4: From Virtual to Reality: Building the Robot**

- [4.1. Hardware Bill of Materials](#41-hardware-bill-of-materials)
- [4.2. Step-by-Step Assembly & Wiring Guide](#42-step-by-step-assembly--wiring-guide)
- [4.3. Host Computer (Raspberry Pi) Setup](#43-host-computer-raspberry-pi-setup)
- [4.4. Firmware Flashing Guide](#44-firmware-flashing-guide)

### **Chapter 5: Bringing the Physical Robot to Life**

- [5.1. Pre-Launch Checklist](#51-pre-launch-checklist)
- [5.2. Tutorial: Mapping a Real-World Room](#52-tutorial-mapping-a-real-world-room)
- [5.3. Tutorial: Autonomous Operation on Hardware](#53-tutorial-autonomous-operation-on-hardware)

### **Chapter 6: Developer's Guide & Customization**

- [6.1. How to Customize the AI's Personality](#61-how-to-customize-the-ais-personality)
- [6.2. How to Tune Face Recognition](#62-how-to-tune-face-recognition)
- [6.3. Common Issues & Troubleshooting](#63-common-issues--troubleshooting)

---

# Chapter 1: Introduction & Core Concepts

## 1.1. Project Vision & Philosophy

This project was created to be a fully self-contained, functional, and understandable example of a modern robotics system using ROS 2. It is designed to bridge the gap between simple "hello world" examples and complex, research-grade systems that are often difficult to dissect.

- **Goal**: To create an autonomous mobile robot that can map its environment, navigate intelligently, and engage in basic, voice-driven social interaction with people it detects and recognizes.
- **Sim-to-Real Parity**: The codebase is structured to run almost identically in a realistic Gazebo simulation and on physical hardware. This is a critical practice in modern robotics, allowing for rapid, safe development in simulation before deploying to the real world.
- **Self-Contained AI**: The "brain" is built from scratch with clear, readable Python. The face recognition uses vector math, and the conversation logic is a rule-based state machine. This avoids dependencies on external, paid cloud services and allows every line of code to be inspected and understood.

## 1.2. Who is this Project For?

- **Students**: A practical, hands-on project that covers a wide array of robotics topics, from low-level firmware to high-level AI.
- **Hobbyists**: A complete recipe for building a fun, interactive robot at home with relatively accessible components.
- **Developers New to ROS**: A real-world example that shows how to structure a complex ROS 2 application with multiple packages, nodes, and launch files.

## 1.3. Core Robotics Concepts Covered

This project isn't just a set of files; it's a practical demonstration of key robotics concepts:

- **URDF (Unified Robot Description Format)**: The `ai_robot.urdf.xacro` file is a standard XML-based format used to describe the robot's physical structure. It defines all the `links` (the solid parts like the chassis and wheels) and `joints` (which connect the links and define how they can move). This description allows ROS to understand the robot's shape for visualization and collision checking.
- **TF2 (Transform Library)**: In a robot, nothing is static. The wheels move relative to the chassis, the camera is mounted on the chassis, and the robot itself moves through the world. TF2 is the ROS 2 library that keeps track of all these dynamic relationships between different coordinate frames. For example, it knows where the `camera_link` is relative to the `base_link`, and where the `base_link` is relative to the `odom` (odometry) frame. This is absolutely essential for taking a sensor reading in one frame (like the camera) and using it to navigate in another frame (like the map).
- **SLAM (Simultaneous Localization and Mapping)**: This is the classic "chicken-and-egg" problem in robotics. To navigate a map, you need to know where you are. But to know where you are, you need a map. SLAM solves this by doing both at the same time. As the robot moves through an _unknown_ environment, it uses its sensors (in our case, a laser scan created from the 3D camera) to build a 2D occupancy grid map while simultaneously estimating its own position within that map as it's being built. We use the powerful `slam_toolbox` package for this.
- **AMCL (Adaptive Monte Carlo Localization)**: Once a map has been created and saved, we no longer need to run SLAM. Now, the task is just localization. AMCL is a probabilistic algorithm that uses a particle filter to figure out the robot's most likely position and orientation on a _known_ map. It works by comparing the robot's current laser scans to the map and "voting" for positions where the scan matches the map's obstacles.
- **Navigation Stack (Nav2)**: This is the complete software suite that enables autonomous movement. It's a collection of nodes that work together:
  - **Planner Server**: Creates a high-level, long-range path from the robot's start to its goal, avoiding obstacles on the static map.
  - **Controller Server**: Creates low-level, short-term motor commands to follow the global path while avoiding immediate, new obstacles (like a person walking in front of the robot).
  - **Costmaps**: Both a global and local costmap are used. These are special versions of the map where areas near obstacles are "inflated" to create a buffer zone, ensuring the robot doesn't get too close to walls.
- **Behavioral AI**: This project demonstrates a simple but effective "sense-think-act" AI loop.
  - **Sense**: The robot uses its camera to detect faces.
  - **Think**: The AI nodes process this information. Is the face known? What is my current emotion? Based on this, what should I say?
  - **Act**: The robot performs an action, such as speaking a greeting or sending a command to its motors.

## 1.4. High-Level System Diagram

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

---

# Chapter 2: The Codebase: A Guided Tour

## 2.1. Workspace Philosophy: Core vs. Drivers

The workspace is split into two packages to enforce a clean separation of concerns, a best practice in robotics software engineering.

- **`ai_robot_core`**: This package represents the robot's "mind." It contains all the high-level, application-specific logic. It is designed to be **hardware-agnostic**. The AI nodes in this package don't know what kind of camera or motors the robot has; they just subscribe to generic ROS topics like `/camera/image_raw` and publish to `/cmd_vel`. This means you could swap out the robot base for a completely different one, and as long as the new base's driver provides these same topics, the entire AI stack would work without modification.

- **`ai_robot_hardware_drivers`**: This package is the "body" and its connection to the nervous system. It contains the specific, low-level code required to interface with the chosen physical hardware. If you were to switch from a Kinect to an Intel RealSense camera, you would only need to change the code within this package (specifically, swapping `kinect_interface_node` for a RealSense driver). The `ai_robot_core` package would remain untouched.

This separation makes the project more robust, maintainable, and easily adaptable to new hardware platforms.

## 2.2. Package Deep Dive: `ai_robot_core`

This package contains the robot's intelligence and its abstract definition.

#### Directory: `nodes/`

This is where the AI "brain" lives. Each file is a standalone ROS 2 node.

- **`face_recognition_node.py`**
  - **Purpose**: To act as the primary vision processing unit.
  - **Logic**:
    1.  In `__init__`, it loads an OpenCV Haar Cascade file for face detection and creates publishers and subscribers.
    2.  The `image_callback` is triggered for every frame from the camera.
    3.  It converts the ROS image message to an OpenCV image using `cv_bridge`.
    4.  It runs the face detector on the image to get bounding boxes `(x, y, w, h)`.
    5.  For each face, it generates a deterministic "embedding" (a feature vector). In this project, this is a placeholder, but the logic stands.
    6.  It calls the `/lookup_person` service on the `memory_manager_node` to check if this embedding is already in the database.
    7.  If the person is unknown, it calls `/register_person` to add them.
    8.  It draws the person's name and a bounding box on the image.
    9.  Finally, it publishes the annotated image to `/annotated_image` and a structured `PersonArray` message to `/detected_people`.

- **`memory_manager_node.py`**
  - **Purpose**: To provide a persistent memory for the robot using a simple file-based database.
  - **Logic**:
    1.  In `__init__`, it connects to or creates a `robot_memory.db` SQLite database file. It defines the schema for a `people` table and a `conversations` table.
    2.  The `lookup_person_callback` is the core of the recognition. It receives a face embedding, fetches all embeddings from the database, and calculates the Euclidean distance between the new embedding and each known one. If a distance is found below a certain threshold (`min_distance`), it returns that person's data. Otherwise, it returns an "unknown" person.
    3.  The `register_person_callback` inserts a new row into the `people` table with a new unique ID, the provided name hint, and the face embedding.

- **`emotion_engine_node.py`**
  - **Purpose**: To provide a simplified emotional state based on visual input.
  - **Logic**:
    1.  It maintains an internal state variable, `self.current_emotion`.
    2.  The `detected_faces_callback` checks the incoming `PersonArray`.
    3.  If the array contains anyone with `is_known: true`, it sets the emotion to "happy".
    4.  If the array only contains people with `is_known: false`, it sets the emotion to "curious".
    5.  A timer, `check_for_neutral`, runs periodically. If no faces have been seen for 10 seconds, it resets the emotion to "neutral".
    6.  It only publishes the new emotion to `/robot_emotion` if the state actually changes.

- **`interaction_manager_node.py`**
  - **Purpose**: To be the master controller for social behavior.
  - **Logic**:
    1.  It subscribes to `/detected_people` to know who is present and `/robot/audio_transcription` to hear what is said.
    2.  The `detected_people_callback` identifies the largest person in the frame and checks if they have been greeted recently.
    3.  If a _known_ person appears who hasn't been greeted in the last 5 minutes, it calls the `/get_conversation` service to retrieve the last thing they talked about and formulates a personalized greeting (e.g., "Hello, Jane! Last time we talked about...").
    4.  If an _unknown_ person appears, it formulates a generic greeting ("Hello! I don't believe we've met.").
    5.  The `audio_transcription_callback` is triggered when the user speaks. It takes the transcribed text and passes it to the `call_llm_api` function.
    6.  The `call_llm_api` function is a **rule-based chatbot**. It uses simple `if/elif` statements to check for keywords in the user's speech and returns a hard-coded, logical response. This provides a complete, interactive experience without any external services.
    7.  All spoken output is published to the `/robot_speech_output` topic.

#### Directory: `launch/`

This directory contains scripts to start the robot's software components.

- **`mapping.launch.py` vs. `navigation.launch.py` vs. `full_ai_app.launch.py`**
  - These files seem similar but serve distinct purposes in a typical robotics workflow.
  - `mapping.launch.py`: The **first step**. Its only job is to run SLAM to create a map. It launches the robot simulation and the `slam_toolbox` node.
  - `navigation.launch.py`: The **second step**. Its job is to test autonomous navigation. It launches the robot simulation and the Nav2 stack, using the map created in the previous step.
  - `full_ai_app.launch.py`: The **final step for simulation**. It does everything `navigation.launch.py` does, but also adds all the AI brain nodes, bringing the robot to its full "sentient" state.

- **`nav2_bringup.launch.py`**
  - **Purpose**: This is a reusable, generic launch file for Nav2. It was created to avoid duplicating the complex Nav2 launch configuration in both the simulation and hardware launch files. Both `navigation.launch.py` and `hardware_bringup.launch.py` _include_ this file, which is a ROS best practice.

#### Directory: `config/` & `urdf/`

- `nav2_params.yaml`: A large file containing all the parameters for the Nav2 stack. Tuning these values can change the robot's navigation behavior (e.g., how close it gets to walls, how fast it accelerates).
- `ai_robot.urdf.xacro`: The robot's "blueprint," defining its physical shape for visualization and simulation.
- `gazebo_plugins.xacro`: A supplementary file that adds simulation-only features, like the differential drive motor controller and the simulated 3D camera sensor. It is ignored when the URDF is used outside of Gazebo.

## 2.3. Package Deep Dive: `ai_robot_hardware_drivers`

This package contains the "glue" between the robot's brain and the physical world.

- **`esp8266_bridge_node.py`**:
  - **Purpose**: To communicate with the low-level motor controller.
  - **Logic**: It connects to an MQTT broker. When it receives a `/cmd_vel` message from Nav2, it does the math to convert the desired linear and angular velocities into PWM power levels for the left and right wheels. It then publishes these values as a JSON message to an MQTT topic. It also performs **dead-reckoning**: based on the commands it sends, it integrates the robot's position over time and publishes this as an `/odom` topic and a TF2 transform, which is essential for AMCL to work.
- **`kinect_interface_node.py`**:
  - **Purpose**: A driver for the physical Kinect camera.
  - **Logic**: It uses the `pykinect2` library to connect to the physical camera, grab color and depth frames, convert them into ROS `Image` messages, and publish them to the appropriate topics for the `face_recognition_node` to use.
- **`audio_interface_node.py`**:
  - **Purpose**: To handle all audio I/O on the host computer.
  - **Logic**: It uses the `pyttsx3` library for text-to-speech and the `speech_recognition` library for speech-to-text. It runs in the background, listening for spoken words from a USB microphone and for text to speak on a ROS topic.

## 2.4. Message Package: `ai_robot_msgs`

This package doesn't contain any code. Its sole purpose is to define the custom "language" or data structures that the nodes use to communicate. Creating a dedicated package for messages is a standard ROS practice that helps prevent circular dependencies.

## 2.5. Firmware Deep Dive: `esp_firmware_8266.ino`

This is the code that runs on the ESP8266 microcontroller. It is the lowest level of control in the system.

- **`setup()`**: This function runs once on boot. It initializes the GPIO pins for the motor driver, connects to WiFi, and connects to the MQTT broker.
- **`loop()`**: This is the main loop. Its only jobs are to ensure the WiFi/MQTT connection is alive and to process incoming MQTT messages via `mqttClient.loop()`.
- **`mqttCallback()`**: This function is triggered when a message arrives on a subscribed MQTT topic. It checks if the topic is `robot/motor_cmd`, parses the JSON payload (e.g., `{'fl': 200, ...}`), and calls `setLeftMotors` and `setRightMotors` to apply the requested power to the L298N driver.
- **No Kinematics**: The firmware has no "smarts." It does not know what `/cmd_vel` is. It only knows how to set motor power, making it a simple and reliable hardware endpoint.

## 2.6. Core Workflow 1: The Vision Pipeline

`Camera -> Face Detection -> Memory Lookup -> Emotion & Interaction`

1.  A video frame is published by the **Gazebo Camera** (in sim) or **`kinect_interface_node`** (on hardware).
2.  **`face_recognition_node`** grabs the frame. It finds a face's bounding box.
3.  It generates a unique vector for the face and calls the `/lookup_person` service on the **`memory_manager_node`**.
4.  **`memory_manager_node`** compares this vector to all vectors in its database. If a close match is found, it returns the person's name. If not, it returns "Unknown".
5.  The final result (e.g., "Jane, is_known=true") is published to `/detected_people`.
6.  Both **`emotion_engine_node`** and **`interaction_manager_node`** receive this message and react accordingly (e.g., change emotion to "happy" and formulate a greeting).

## 2.7. Core Workflow 2: The Conversation Loop

`User Speaks -> STT -> AI Brain -> TTS -> Robot Speaks`

1.  The **`audio_interface_node`** captures audio from the USB microphone and transcribes it to text.
2.  This text is published to `/robot/audio_transcription`.
3.  The **`interaction_manager_node`** receives the text.
4.  It passes the text to its internal `call_llm_api` function, which uses `if/elif/else` logic to select a response.
5.  The chosen response string is published to `/robot_speech_output`.
6.  A launch file remaps this topic to `/robot/tts`, which the **`audio_interface_node`** is listening to.
7.  The `audio_interface_node` receives the text and uses the `pyttsx3` library to generate speech, which is played through the USB speakers.

---

# Chapter 3: The Simulation: A Virtual Proving Ground

## 3.1. Why Simulate? The Importance of a Digital Twin

Simulation allows us to develop and test nearly the entire software stack—from navigation to AI—without risking damage to a physical robot and without the slow pace of real-world testing. By developing in a "digital twin" of the real robot, we can be confident that the logic will work correctly when deployed. It is an indispensable tool for safe, rapid, and cost-effective robotics development.

## 3.2. Prerequisites for Simulation

- **Ubuntu 22.04**: This project is targeted for this specific LTS version of Ubuntu.
- **ROS 2 Humble Hawksbill (Desktop-Full Install)**: It is critical to install the `Desktop-Full` version, as it includes Gazebo, RViz, and other essential tools.
- **Git**: For cloning the repository.
- **Colcon**: The standard build tool for ROS 2.
- **Required ROS Packages**:
  ```bash
  sudo apt-get update
  sudo apt-get install ros-humble-slam-toolbox ros-humble-nav2-bringup
  ```

## 3.3. Build Instructions

1.  **Create a Workspace**: First, create a new directory for your ROS 2 workspace.
    ```bash
    mkdir -p ~/ai_robot_ws/src
    ```
2.  **Clone the Repository**: Navigate into the `src` directory and clone this project.
    ```bash
    cd ~/ai_robot_ws/src
    git clone https://github.com/your-repo/ai_robot.git
    ```
    _(Replace the URL with the actual repository URL)_
3.  **Install Dependencies**: Navigate to the workspace root (`ai_robot_ws`) and let `rosdep` automatically install any missing system dependencies.
    ```bash
    cd ~/ai_robot_ws
    rosdep install -i --from-path src -y --rosdistro humble
    ```
4.  **Build the Workspace**: Use `colcon` to build all the packages.
    ```bash
    colcon build --symlink-install
    ```
    The `--symlink-install` flag is very useful during development, as it allows you to change Python files and have the changes take effect immediately without needing to rebuild.

## 3.4. Tutorial: Creating a Map with SLAM

Before the robot can navigate, it needs a map. This tutorial will guide you through creating one using the simulation.

1.  **Source Your Workspace**: Open a new terminal and source your workspace to make the ROS 2 packages available.
    ```bash
    cd ~/ai_robot_ws
    source install/setup.bash
    ```
2.  **Launch the Mapping Stack**:

    ```bash
    ros2 launch ai_robot_core mapping.launch.py
    ```

    - **What Happens Now?** Several windows and processes will start.
      - **Gazebo**: A 3D simulation window will appear, showing the robot in a world with a few obstacles. This is the "reality" the robot perceives.
      - **RViz**: A 2D/3D visualization tool will open. This is your "god's-eye view" into the robot's mind.
      - **Terminal Output**: You will see logs from all the nodes being launched.

3.  **Launch Keyboard Control**: To drive the robot, open a **second terminal**, and source the workspace again.

    ```bash
    cd ~/ai_robot_ws
    source install/setup.bash
    ros2 launch ai_robot_core teleop.launch.py
    ```

    This will open a new `xterm` window specifically for keyboard input.

4.  **Explore and Build the Map**:
    - Click on the `xterm` window to give it focus.
    - Use the keys printed in the terminal to drive the robot: `i` (forward), `,` (backward), `j` (turn left), `l` (turn right).
    - As you drive the robot in Gazebo, look at the RViz window. You will see the map being built in real-time.
      - **Black/Grey Pixels**: Obstacles detected by the robot's laser scan.
      - **Light Grey Areas**: Free space the robot has explored.
      - **White/Uncolored Areas**: Unexplored territory.
    - Your goal is to "paint" all the accessible areas light grey by driving through them. Ensure you create a closed loop by driving around obstacles and returning to a previously visited area. This helps the SLAM algorithm correct for errors and create a more accurate map.

5.  **Save the Map**: Once the map in RViz looks complete and accurate, **do not close the mapping or teleop terminals**. Open a **third terminal**, source the workspace, and run the `map_saver_cli` tool.

    ```bash
    cd ~/ai_robot_ws
    mkdir -p maps
    ros2 run nav2_map_server map_saver_cli -f maps/my_map
    ```

    - **What this does**: This command connects to the `/map` topic being published by `slam_toolbox`, saves the current map data as an image (`maps/my_map.pgm`), and creates a configuration file (`maps/my_map.yaml`) that describes the map's resolution and origin.

    You have now successfully created a map. You can close all the terminals (`Ctrl+C`).

## 3.5. Tutorial: Autonomous Navigation in Simulation

With a saved map, you can now run the navigation stack.

1.  **Source Your Workspace**: Open a new terminal and source it.
    ```bash
    cd ~/ai_robot_ws
    source install/setup.bash
    ```
2.  **Launch the Navigation Stack**:

    ```bash
    ros2 launch ai_robot_core navigation.launch.py map:=$(pwd)/maps/my_map.yaml
    ```

    - **What's Happening**: This launches Gazebo and the robot, but instead of `slam_toolbox`, it launches the full Nav2 stack, including AMCL. AMCL will load your map and try to figure out the robot's position. You may see a cloud of small red arrows in RViz around the robot's starting position—these are the "particles" from the particle filter, representing possible locations for the robot. As the robot moves, these particles will converge on the single, most likely location.

3.  **Set an Initial Pose**: AMCL is good, but sometimes it needs a hint. If the robot looks lost (particles are all over the map), you need to give it an initial pose.
    - In the RViz toolbar, click the "2D Pose Estimate" button.
    - Click on the map where the robot is located in the Gazebo world, and drag in the direction it is facing.
    - This will re-initialize the AMCL particles in the correct area, and they should converge quickly.

4.  **Set a Navigation Goal**:
    - In the RViz toolbar, click the "Nav2 Goal" button.
    - Click on a destination on the map and drag to set the desired final orientation.
    - You will see a green line appear—this is the global path from the **Planner**. The robot will now start moving, following this path while the **Controller** makes small adjustments to avoid obstacles.

## 3.6. Tutorial: Full AI Interaction in Simulation

This is the final, "everything" launch file for the simulation.

1.  **Source Your Workspace**: Open a new terminal and source it.
2.  **Launch the Full Application**:

    ```bash
    ros2 launch ai_robot_core full_ai_app.launch.py map:=$(pwd)/maps/my_map.yaml
    ```

    - **What's Happening**: This does everything the navigation launch file did, but also starts all the AI nodes (`face_recognition`, `memory_manager`, `emotion_engine`, `interaction_manager`) and the RQT UI.

3.  **Observe the AI**:
    - An RQT window titled "AI Robot Face" will appear. It will show the robot's camera feed and its current emotion emoji.
    - In Gazebo, go to the "Insert" tab on the left, find a human model (e.g., "Actor"), and place it in front of the robot.
    - You should see the robot turn to face the person. In the RQT window, a green box will be drawn around the person's face with the name "Unknown".
    - Check the terminal where you ran the launch file. The `interaction_manager` should log a message like: `[interaction_manager_node-11] [INFO] Saying: Hello! I don't believe we've met.`
    - If you remove the human model and add it again, the robot should now recognize it and say something like, "Hello, Jane! Nice to see you again."

The robot is now fully alive in the simulation, ready to navigate and interact.

---

# Chapter 4: From Virtual to Reality: Building the Robot

## 4.1. Hardware Bill of Materials

- **Chassis**: A 4-wheel drive robot chassis with TT motors. These are very common and inexpensive.
- **Microcontroller**: 1x ESP8266 (NodeMCU or a similar board). This will serve as our low-level motor controller, receiving commands over WiFi.
- **Motor Driver**: 1x L298N Dual H-Bridge Motor Driver. A robust and inexpensive driver capable of supplying enough current for four standard TT motors.
- **Host Computer**: 1x Raspberry Pi 4 (4GB+ recommended) or a similar single-board computer. This will be the robot's main brain, running the ROS 2 stack.
- **Camera**: 1x Microsoft Xbox 360 Kinect (Model 1414 or 1473). This is an older but very capable 3D camera that provides both a color image and a depth image, which is crucial for navigation and perception. It requires a special USB/Power adapter.
- **Audio**: 1x standard USB Microphone and 1x standard USB-powered Speaker. These will connect to the Raspberry Pi for voice interaction.
- **Power**:
  - A 7.4V-12V battery pack (e.g., a 2S LiPo or a holder for 6-8x AA batteries) to power the L298N motor driver.
  - A 5V USB power bank (at least 3A output recommended) to provide clean, stable power to the Raspberry Pi.
  - The ESP8266 will be powered directly from the L298N's onboard 5V regulator.

## 4.2. Step-by-Step Assembly & Wiring Guide

This guide provides the exact wiring needed for the provided firmware to work correctly. Double-check all connections.

**Wiring Diagram:**

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

**Assembly Instructions:**

1.  **Mount Components**: Securely mount the Raspberry Pi, L298N driver, and ESP8266 onto your robot chassis. Also mount the battery holders/packs.
2.  **Wire Motors to Driver**:
    - Take the two wires from your **front-left** motor and connect them to the `OUT1` and `OUT2` screw terminals on the L298N.
    - Take the two wires from your **back-left** motor and also connect them to `OUT1` and `OUT2`. The motors on the same side are wired in parallel.
    - Repeat for the right side: connect the **front-right** and **back-right** motor wires to `OUT3` and `OUT4`.
3.  **Wire Power to Driver**:
    - Connect the positive (red) wire from your 7.4V-12V battery pack to the `12V` screw terminal on the L298N.
    - Connect the negative (black) wire from the battery pack to the `GND` screw terminal on the L298N.
4.  **Wire ESP8266 to Driver (Control Signals)**: Use jumper wires for these connections.
    - `ESP8266 D1` -> `L298N IN1`
    - `ESP8266 D2` -> `L298N IN2`
    - `ESP8266 D5` -> `L298N IN3`
    - `ESP8266 D6` -> `L298N IN4`
    - `ESP8266 D3` -> `L298N ENA` (This controls the speed of the left motors)
    - `ESP8266 D7` -> `L298N ENB` (This controls the speed of the right motors)
5.  **Wire Power to ESP8266**:
    - **CRITICAL**: Ensure the "5V Regulator" jumper on the L298N is in place.
    - Connect a wire from the `GND` screw terminal on the L298N to any `GND` pin on the ESP8266. This creates a common ground, which is essential.
    - Connect a wire from the `5V` screw terminal on the L298N to the `VIN` pin on the ESP8266. This will power the microcontroller from the motor driver's regulated output.

## 4.3. Host Computer (Raspberry Pi) Setup

1.  **Install OS**: Using the Raspberry Pi Imager, flash a microSD card with **Ubuntu Server 22.04.1 LTS (64-bit)**.
2.  **Install ROS 2**: Boot the Pi, connect it to the internet, and follow the official ROS 2 documentation to install the `ros-humble-ros-base` package. This version is lighter than the desktop install and suitable for a robot.
3.  **Install Essential Tools**:
    ```bash
    sudo apt-get update
    sudo apt-get install git python3-pip
    ```
4.  **Install MQTT Broker**: The broker is the message hub between the Pi and the ESP8266.
    ```bash
    sudo apt-get install mosquitto mosquitto-clients
    sudo systemctl enable mosquitto # Make it start on boot
    sudo systemctl start mosquitto
    ```
5.  **Network Configuration**:
    - Connect the Raspberry Pi to your WiFi network.
    - Find the Pi's IP address using `ip a`. Note this down, as you will need it for the firmware.

## 4.4. Firmware Flashing Guide

1.  **Setup Arduino IDE**:
    - Download and install the Arduino IDE on your main computer (not the Pi).
    - Go to `File > Preferences`. In "Additional Boards Manager URLs," add: `http://arduino.esp8266.com/stable/package_esp8266com_index.json`
    - Go to `Tools > Board > Boards Manager...`, search for `esp8266`, and install the package by ESP8266 Community.
    - Select your board from the `Tools > Board` menu (e.g., "NodeMCU 1.0 (ESP-12E Module)").
2.  **Install Libraries**:
    - Go to `Sketch > Include Library > Manage Libraries...`.
    - Search for and install `PubSubClient` by Nick O'Leary.
    - Search for and install `ArduinoJson` by Benoit Blanchon.
3.  **Configure and Flash**:
    - Open the `firmware/esp_firmware_8266.ino` file from this project in the Arduino IDE.
    - **Crucially, you must edit these three lines** with your own network details:
      ```cpp
      const char *SSID = "YOUR_WIFI_SSID";
      const char *PASSWORD = "YOUR_WIFI_PASSWORD";
      const char *MQTT_SERVER = "RASPBERRY_PI_IP_ADDRESS";
      ```
    - Connect the ESP8266 to your computer via USB.
    - Select the correct port under `Tools > Port`.
    - Click the "Upload" button (the right-arrow icon). You can open the Serial Monitor (`Ctrl+Shift+M`) to see status messages as it boots and connects.

---

# Chapter 5: Bringing the Physical Robot to Life

## 5.1. Pre-Launch Checklist

Before running the main launch file, verify every one of these points:

1.  **Power**: Is the motor battery pack charged and connected to the L298N? Is the Raspberry Pi powered on via its USB power bank?
2.  **Network**: Are both the Raspberry Pi and the ESP8266 connected to the _same_ WiFi network?
3.  **Connectivity**: From a terminal on the Raspberry Pi, can you `ping` the ESP8266's IP address? (You can find the ESP's IP in the Arduino IDE's Serial Monitor on boot).
4.  **MQTT Broker**: Is the Mosquitto service running on the Pi? Check with `systemctl status mosquitto`.
5.  **Peripherals**: Are the Kinect, USB microphone, and USB speakers all plugged into the Raspberry Pi?

## 5.2. Tutorial: Mapping a Real-World Room

You cannot use navigation in a new space without first creating a map. This process uses SLAM to build the map from live sensor data.

1.  **Place the Robot**: Put your robot on the floor in a relatively open area of the room you want to map.
2.  **Launch the Mapping Stack**: On your Raspberry Pi, run the following command. This starts the hardware drivers and `slam_toolbox`.
    ```bash
    ros2 launch ai_robot_core mapping.launch.py use_sim_time:=false
    ```
3.  **Launch Keyboard Control**: In a **second SSH terminal** to your Raspberry Pi, launch teleop.
    ```bash
    cd ~/ai_robot_ws && source install/setup.bash
    ros2 launch ai_robot_core teleop.launch.py
    ```
4.  **Visualize and Build the Map**:
    - On a separate desktop computer on the same WiFi network, install ROS 2 Humble Desktop.
    - Launch RViz: `rviz2`.
    - In RViz, change the "Global Options" -> "Fixed Frame" to `odom`.
    - Add the `/map`, `/scan`, and `TF` displays.
    - Drive the robot using the teleop terminal. Watch the map build in RViz.
5.  **Save the Map**: Once complete, open a **third SSH terminal** to the Pi and run the map saver:
    ```bash
    cd ~/ai_robot_ws
    mkdir -p maps
    ros2 run nav2_map_server map_saver_cli -f maps/my_map
    ```
    You can now `Ctrl+C` the mapping and teleop terminals.

## 5.3. Tutorial: Autonomous Operation on Hardware

Now that you have a map, you can run the robot in its fully autonomous, interactive mode.

1.  **Install Hardware Dependencies**: On the Raspberry Pi:
    ```bash
    sudo apt-get install python3-paho-mqtt python3-pyttsx3 python3-pyaudio
    ```
2.  **Launch the Main Hardware Stack**: This command starts all hardware drivers, the AI stack, and the Nav2 stack in navigation mode. **You must provide the full, absolute path to your map file.**

    ```bash
    # Source your workspace
    source install/setup.bash

    # Example:
    ros2 launch ai_robot_hardware_drivers hardware_bringup.launch.py map:=/home/ubuntu/ai_robot_ws/maps/my_map.yaml
    ```

    - **Dissecting the command**:
      - `ros2 launch ai_robot_hardware_drivers hardware_bringup.launch.py`: Runs the main launch file from the hardware driver package.
      - `map:=/path/to/your/map.yaml`: This **launch argument** is critical. It passes the location of your map file to the Nav2 `map_server` node.

3.  **Localize and Interact**:
    - Launch RViz on your desktop computer.
    - The robot will attempt to localize itself. Give it an initial pose using the "2D Pose Estimate" tool in RViz.
    - Once localized, you can give it navigation goals.
    - Walk in front of the robot. It should detect your face, and you can begin interacting with it through voice commands.

---

# Chapter 6: Developer's Guide & Customization

## 6.1. How to Customize the AI's Personality

The robot's conversational ability is defined entirely within the `call_llm_api` function in `ai_robot_core/nodes/interaction_manager_node.py`. You can easily add more `elif` conditions to check for different keywords and provide custom responses.

**Example: Add a new response**

```python
# In interaction_manager_node.py, inside call_llm_api function:

elif "what time is it" in new_line:
    # Add an import for datetime at the top of the file
    import datetime
    return f"The current time is {datetime.datetime.now().strftime('%I:%M %p')}."
```

## 6.2. How to Tune Face Recognition

The strictness of face matching is controlled by the `min_distance` variable in the `lookup_person_callback` function in `ai_robot_core/nodes/memory_manager_node.py`. This value is the threshold for the Euclidean distance between face vectors.

- **Stricter Matching**: Lower this value (e.g., to `0.5` or `0.45`).
- **Looser Matching**: Increase this value (e.g., to `0.65`).

## 6.3. Common Issues & Troubleshooting

- **Problem**: The robot doesn't move, but I see `/cmd_vel` messages.
  - **Solution**: This is an MQTT issue. Check that the broker is running, the IP in the firmware is correct, and both devices are on the same WiFi.
- **Problem**: The robot's navigation is jerky or gets stuck.
  - **Solution**: This is a tuning issue in `nav2_params.yaml`. Start by reducing `max_vel_x` and the acceleration limits.
- **Problem**: `pykinect2` fails to install or run on the Raspberry Pi.
  - **Solution**: This is a known, difficult issue. The professional solution is to replace it with a different driver that uses `libfreenect`, the open-source Kinect library. Install `ros-humble-freenect-camera` and change the `hardware_bringup.launch.py` file to launch the `freenect_launch.py` file instead of the `kinect_interface_node`.
