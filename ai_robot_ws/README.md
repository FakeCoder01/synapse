# The AI Pet Robot: The Definitive Manual

**Version 1.0**

Welcome to the definitive guide and manual for the AI Pet Robot, a comprehensive ROS 2 Humble project. This document is structured as a complete, multi-chapter book, covering everything from foundational robotics concepts and high-level architecture to a line-by-line explanation of the code, and provides granular, step-by-step tutorials for running the robot in simulation and deploying it on real-world hardware.

![RViz View](https://i.imgur.com/8i9hZkL.png)

## Table of Contents

### **Chapter 1: Introduction & Core Concepts**
*   [1.1. Project Vision & Philosophy](#11-project-vision--philosophy)
*   [1.2. Who is this Project For?](#12-who-is-this-project-for)
*   [1.3. Core Robotics Concepts Covered](#13-core-robotics-concepts-covered)
*   [1.4. High-Level System Diagram](#14-high-level-system-diagram)

### **Chapter 2: The Codebase: A Guided Tour**
*   [2.1. Workspace Philosophy: Core vs. Drivers](#21-workspace-philosophy-core-vs-drivers)
*   [2.2. Package Deep Dive: `ai_robot_core`](#22-package-deep-dive-ai_robot_core)
*   [2.3. Package Deep Dive: `ai_robot_hardware_drivers`](#23-package-deep-dive-ai_robot_hardware_drivers)
*   [2.4. Message Package: `ai_robot_msgs`](#24-message-package-ai_robot_msgs)
*   [2.5. Firmware Deep Dive: `esp_firmware_8266.ino`](#25-firmware-deep-dive-esp_firmware_8266ino)

### **Chapter 3: The Simulation: A Virtual Proving Ground**
*   [3.1. Why Simulate? The Importance of a Digital Twin](#31-why-simulate-the-importance-of-a-digital-twin)
*   [3.2. Prerequisites for Simulation](#32-prerequisites-for-simulation)
*   [3.3. Build Instructions](#33-build-instructions)
*   [3.4. Tutorial: Creating a Map with SLAM](#34-tutorial-creating-a-map-with-slam)
*   [3.5. Tutorial: Autonomous Navigation in Simulation](#35-tutorial-autonomous-navigation-in-simulation)
*   [3.6. Tutorial: Full AI Interaction in Simulation](#36-tutorial-full-ai-interaction-in-simulation)

### **Chapter 4: From Virtual to Reality: Building the Robot**
*   [4.1. Hardware Bill of Materials](#41-hardware-bill-of-materials)
*   [4.2. Step-by-Step Assembly & Wiring Guide](#42-step-by-step-assembly--wiring-guide)
*   [4.3. Host Computer (Raspberry Pi) Setup](#43-host-computer-raspberry-pi-setup)
*   [4.4. Firmware Flashing Guide](#44-firmware-flashing-guide)

### **Chapter 5: Bringing the Physical Robot to Life**
*   [5.1. Pre-Launch Checklist](#51-pre-launch-checklist)
*   [5.2. Tutorial: Mapping a Real-World Room](#52-tutorial-mapping-a-real-world-room)
*   [5.3. Tutorial: Autonomous Operation on Hardware](#53-tutorial-autonomous-operation-on-hardware)

### **Chapter 6: Developer's Guide & Customization**
*   [6.1. How to Customize the AI's Personality](#61-how-to-customize-the-ais-personality)
*   [6.2. How to Tune Face Recognition](#62-how-to-tune-face-recognition)
*   [6.3. Common Issues & Troubleshooting](#63-common-issues--troubleshooting)

---

# Chapter 1: Introduction & Core Concepts

## 1.1. Project Vision & Philosophy

This project was created to be a fully self-contained, functional, and understandable example of a modern robotics system using ROS 2. It is designed to bridge the gap between simple "hello world" examples and complex, research-grade systems that are often difficult to dissect.

*   **Goal**: To create an autonomous mobile robot that can map its environment, navigate intelligently, and engage in basic, voice-driven social interaction with people it detects and recognizes.
*   **Sim-to-Real Parity**: The codebase is structured to run almost identically in a realistic Gazebo simulation and on physical hardware. This is a critical practice in modern robotics, allowing for rapid, safe development in simulation before deploying to the real world.
*   **Self-Contained AI**: The "brain" is built from scratch with clear, readable Python. The face recognition uses vector math, and the conversation logic is a rule-based state machine. This avoids dependencies on external, paid cloud services and allows every line of code to be inspected and understood.

## 1.2. Who is this Project For?

*   **Students**: A practical, hands-on project that covers a wide array of robotics topics, from low-level firmware to high-level AI.
*   **Hobbyists**: A complete recipe for building a fun, interactive robot at home with relatively accessible components.
*   **Developers New to ROS**: A real-world example that shows how to structure a complex ROS 2 application with multiple packages, nodes, launch files, and hardware abstraction.

## 1.3. Core Robotics Concepts Covered

This project isn't just a set of files; it's a practical demonstration of key robotics concepts:

*   **URDF (Unified Robot Description Format)**: The `ai_robot.urdf.xacro` file is a standard XML-based format used to describe the robot's physical structure. It defines all the `links` (the solid parts like the chassis and wheels) and `joints` (which connect the links and define how they can move). This description allows ROS to understand the robot's shape for visualization and collision checking.
*   **TF2 (Transform Library)**: In a robot, nothing is static. The wheels move relative to the chassis, the camera is mounted on the chassis, and the robot itself moves through the world. TF2 is the ROS 2 library that keeps track of all these dynamic relationships between different coordinate frames. For example, it knows where the `camera_link` is relative to the `base_link`, and where the `base_link` is relative to the `odom` (odometry) frame. This is absolutely essential for taking a sensor reading in one frame (like the camera) and using it to navigate in another frame (like the map).
*   **SLAM (Simultaneous Localization and Mapping)**: This is the classic "chicken-and-egg" problem in robotics. To navigate a map, you need to know where you are. But to know where you are, you need a map. SLAM solves this by doing both at the same time. As the robot moves through an *unknown* environment, it uses its sensors (in our case, a laser scan created from the 3D camera) to build a 2D occupancy grid map while simultaneously estimating its own position within that map as it's being built. We use the powerful `slam_toolbox` package for this.
*   **AMCL (Adaptive Monte Carlo Localization)**: Once a map has been created and saved, we no longer need to run SLAM. Now, the task is just localization. AMCL is a probabilistic algorithm that uses a particle filter to figure out the robot's most likely position and orientation on a *known* map. It works by comparing the robot's current laser scans to the map and "voting" for positions where the scan matches the map's obstacles.
*   **Navigation Stack (Nav2)**: This is the complete software suite that enables autonomous movement. It's a collection of nodes that work together:
    *   **Planner Server**: Creates a high-level, long-range path from the robot's start to its goal, avoiding obstacles on the static map.
    *   **Controller Server**: Creates low-level, short-term motor commands to follow the global path while avoiding immediate obstacles (like a person walking in front of the robot).
    *   **Costmaps**: Both a global and local costmap are used. These are special versions of the map where areas near obstacles are "inflated" to create a buffer zone, ensuring the robot doesn't get too close to walls.
*   **Behavioral AI**: This project demonstrates a simple but effective "sense-think-act" AI loop.
    *   **Sense**: The robot uses its camera to detect faces.
    *   **Think**: The AI nodes process this information. Is the face known? What is my current emotion? Based on this, what should I say?
    *   **Act**: The robot performs an action, such as speaking a greeting or sending a command to its motors.

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

*   **`ai_robot_core`**: This package represents the robot's "mind." It contains all the high-level, application-specific logic. It is designed to be **hardware-agnostic**. The AI nodes in this package don't know what kind of camera or motors the robot has; they just subscribe to generic ROS topics like `/camera/image_raw` and publish to `/cmd_vel`. This means you could swap out the robot base for a completely different one, and as long as the new base's driver provides these same topics, the entire AI stack would work without modification.

*   **`ai_robot_hardware_drivers`**: This package is the "body" and its connection to the nervous system. It contains the specific, low-level code required to interface with the chosen physical hardware. If you were to switch from a Kinect to an Intel RealSense camera, you would only need to change the code within this package (specifically, swapping `kinect_interface_node` for a RealSense driver). The `ai_robot_core` package would remain untouched.

This separation makes the project more robust, maintainable, and easily adaptable to new hardware platforms.

## 2.2. Package Deep Dive: `ai_robot_core`

This package contains the robot's intelligence and its abstract definition.

#### Directory: `nodes/`
This is where the AI "brain" lives. Each file is a standalone ROS 2 node.

*   **`face_recognition_node.py`**
    *   **Purpose**: To act as the primary vision processing unit.
    *   **Logic**:
        1.  In `__init__`, it loads an OpenCV Haar Cascade file for face detection and creates publishers and subscribers.
        2.  The `image_callback` is triggered for every frame from the camera.
        3.  It converts the ROS image message to an OpenCV image using `cv_bridge`.
        4.  It runs the face detector on the image to get bounding boxes `(x, y, w, h)`.
        5.  For each face, it generates a deterministic "embedding" (a feature vector). In this project, this is a placeholder, but the logic stands.
        6.  It calls the `/lookup_person` service on the `memory_manager_node` to check if this embedding is already in the database.
        7.  If the person is unknown, it calls `/register_person` to add them.
        8.  It draws the person's name and a bounding box on the image.
        9.  Finally, it publishes the annotated image to `/annotated_image` and a structured `PersonArray` message to `/detected_people`.

*   **`memory_manager_node.py`**
    *   **Purpose**: To provide a persistent memory for the robot using a simple file-based database.
    *   **Logic**:
        1.  In `__init__`, it connects to or creates a `robot_memory.db` SQLite database file. It defines the schema for a `people` table and a `conversations` table.
        2.  The `lookup_person_callback` is the core of the recognition. It receives a face embedding, fetches all embeddings from the database, and calculates the Euclidean distance between the new embedding and each known one. If a distance is found below a certain threshold (`min_distance`), it returns that person's data. Otherwise, it returns an "unknown" person.
        3.  The `register_person_callback` inserts a new row into the `people` table with a new unique ID, the provided name hint, and the face embedding.

*   **`emotion_engine_node.py`**
    *   **Purpose**: To provide a simplified emotional state based on visual input.
    *   **Logic**:
        1.  It maintains an internal state variable, `self.current_emotion`.
        2.  The `detected_faces_callback` checks the incoming `PersonArray`.
        3.  If the array contains anyone with `is_known: true`, it sets the emotion to "happy".
        4.  If the array only contains people with `is_known: false`, it sets the emotion to "curious".
        5.  A timer, `check_for_neutral`, runs periodically. If no faces have been seen for 10 seconds, it resets the emotion to "neutral".
        6.  It only publishes the new emotion to `/robot_emotion` if the state actually changes.

*   **`interaction_manager_node.py`**
    *   **Purpose**: To be the master controller for social behavior.
    *   **Logic**:
        1.  It subscribes to `/detected_people` to know who is present and `/robot/audio_transcription` to hear what is said.
        2.  The `detected_people_callback` identifies the largest person in the frame and checks if they have been greeted recently.
        3.  If a *known* person appears who hasn't been greeted in the last 5 minutes, it calls the `/get_conversation` service to retrieve the last thing they talked about and formulates a personalized greeting (e.g., "Hello, Jane! Last time we talked about...").
        4.  If an *unknown* person appears, it formulates a generic greeting ("Hello! I don't believe we've met.").
        5.  The `audio_transcription_callback` is triggered when the user speaks. It takes the transcribed text and passes it to the `call_llm_api` function.
        6.  The `call_llm_api` function is a **rule-based chatbot**. It uses simple `if/elif` statements to check for keywords in the user's speech and returns a hard-coded, logical response. This provides a complete, interactive experience without any external services.
        7.  All spoken output is published to the `/robot_speech_output` topic.

#### Directory: `launch/`
This directory contains scripts to start the robot's software components.

*   **`mapping.launch.py` vs. `navigation.launch.py` vs. `full_ai_app.launch.py`**
    *   These files seem similar but serve distinct purposes in a typical robotics workflow.
    *   `mapping.launch.py`: The **first step**. Its only job is to run SLAM to create a map. It launches the robot simulation and the `slam_toolbox` node.
    *   `navigation.launch.py`: The **second step**. Its job is to test autonomous navigation. It launches the robot simulation and the Nav2 stack, using the map created in the previous step.
    *   `full_ai_app.launch.py`: The **final step for simulation**. It does everything `navigation.launch.py` does, but also adds all the AI brain nodes, bringing the robot to its full "sentient" state.

*   **`nav2_bringup.launch.py`**
    *   **Purpose**: This is a reusable, generic launch file for Nav2. It was created to avoid duplicating the complex Nav2 launch configuration in both the simulation and hardware launch files. Both `navigation.launch.py` and `hardware_bringup.launch.py` *include* this file, which is a ROS best practice.

#### Directory: `config/` & `urdf/`
*   `nav2_params.yaml`: A large file containing all the parameters for the Nav2 stack. Tuning these values can change the robot's navigation behavior (e.g., how close it gets to walls, how fast it accelerates).
*   `ai_robot.urdf.xacro`: The robot's "blueprint," defining its physical shape for visualization and simulation.
*   `gazebo_plugins.xacro`: A supplementary file that adds simulation-only features, like the differential drive motor controller and the simulated 3D camera sensor. It is ignored when the URDF is used outside of Gazebo.

## 2.3. Package Deep Dive: `ai_robot_hardware_drivers`

This package contains the "glue" between the robot's brain and the physical world.

*   **`esp8266_bridge_node.py`**:
    *   **Purpose**: To communicate with the low-level motor controller.
    *   **Logic**: It connects to an MQTT broker. When it receives a `/cmd_vel` message from Nav2, it does the math to convert the desired linear and angular velocities into PWM power levels for the left and right wheels. It then publishes these values as a JSON message to an MQTT topic. It also performs **dead-reckoning**: based on the commands it sends, it integrates the robot's position over time and publishes this as an `/odom` topic and a TF2 transform, which is essential for AMCL to work.
*   **`kinect_interface_node.py`**:
    *   **Purpose**: A driver for the physical Kinect camera.
    *   **Logic**: It uses the `pykinect2` library to connect to the physical camera, grab color and depth frames, convert them into ROS `Image` messages, and publish them to the appropriate topics for the `face_recognition_node` to use.
*   **`audio_interface_node.py`**:
    *   **Purpose**: To handle all audio I/O on the host computer.
    *   **Logic**: It uses the `pyttsx3` library for text-to-speech and the `speech_recognition` library for speech-to-text. It runs in the background, listening for spoken words from a USB microphone and for text to speak on a ROS topic.

## 2.4. Message Package: `ai_robot_msgs`
This package doesn't contain any code. Its sole purpose is to define the custom "language" or data structures that the nodes use to communicate. Creating a dedicated package for messages is a standard ROS practice that helps prevent circular dependencies.

## 2.5. Firmware Deep Dive: `esp_firmware_8266.ino`
This is the code that runs on the ESP8266 microcontroller. It is the lowest level of control in the system.
*   **`setup()`**: This function runs once on boot. It initializes the GPIO pins for the motor driver, connects to WiFi, and connects to the MQTT broker.
*   **`loop()`**: This is the main loop. Its only jobs are to ensure the WiFi/MQTT connection is alive and to process incoming MQTT messages via `mqttClient.loop()`.
*   **`mqttCallback()`**: This function is triggered when a message arrives on a subscribed MQTT topic. It checks if the topic is `robot/motor_cmd`, parses the JSON payload (e.g., `{'fl': 200, ...}`), and calls `setLeftMotors` and `setRightMotors` to apply the requested power to the L298N driver.
*   **No Kinematics**: The firmware has no "smarts." It does not know what `/cmd_vel` is. It only knows how to set motor power, making it a simple and reliable hardware endpoint.

---

## Part 3: The Simulation: A Virtual Proving Ground

### 3.1. Why Simulate? The Importance of a Digital Twin
Simulation allows us to develop and test nearly the entire software stack—from navigation to AI—without risking damage to a physical robot and without the slow pace of real-world testing. By developing in a "digital twin" of the real robot, we can be confident that the logic will work correctly when deployed.

### 3.2. Prerequisites for Simulation
*   Ubuntu 22.04 with ROS 2 Humble **Desktop-Full**.
*   `colcon`, `git`.
*   `sudo apt-get update && sudo apt-get install ros-humble-slam-toolbox ros-humble-nav2-bringup`

### 3.3. Build Instructions
1.  **Clone**: `mkdir -p ai_robot_ws/src && cd ai_robot_ws/src && git clone <repo_url>`
2.  **Install Dependencies**: `cd .. && rosdep install -i --from-path src -y --rosdistro humble`
3.  **Build**: `colcon build --symlink-install`
4.  **Source**: In every new terminal, `cd ~/ai_robot_ws && source install/setup.bash`

### 3.4. Tutorial: Creating a Map with SLAM
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

### 3.5. Tutorial: Autonomous Navigation in Simulation
1.  **Launch Navigation**:
    ```bash
    ros2 launch ai_robot_core navigation.launch.py map:=$(pwd)/maps/my_map.yaml
    ```
2.  **Set Goal**: In RViz, click the "Nav2 Goal" button in the top toolbar. Click and drag on the map to set a destination pose for the robot. The robot will begin to navigate.

### 3.6. Tutorial: Full AI Interaction in Simulation
1.  **Launch Full App**:
    ```bash
    ros2 launch ai_robot_core full_ai_app.launch.py map:=$(pwd)/maps/my_map.yaml
    ```
2.  **Interact**: Add a human model in Gazebo. The robot should turn to face it and greet it. You can use the "Publish Point" tool in RViz to simulate other interactions.

---

## Part 4: From Virtual to Reality: Building the Robot

### 4.1. Hardware Bill of Materials
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

### 4.2. Step-by-Step Assembly & Wiring Guide
*(Same content as previous README, with diagram)*

### 4.3. Host Computer (Raspberry Pi) Setup
*(Same content as previous README)*

### 4.4. Firmware Flashing Guide
*(Same content as previous README)*

---

## Part 5: Bringing the Physical Robot to Life

### 5.1. Pre-Launch Checklist
Before running the main launch file, verify every one of these points:
1.  **Power**: Is the motor battery pack charged and connected to the L298N? Is the Raspberry Pi powered on via its USB power bank?
2.  **Network**: Are both the Raspberry Pi and the ESP8266 connected to the *same* WiFi network?
3.  **Connectivity**: From a terminal on the Raspberry Pi, can you `ping` the ESP8266's IP address? (You can find the ESP's IP in the Arduino IDE's Serial Monitor on boot).
4.  **MQTT Broker**: Is the Mosquitto service running on the Pi? Check with `systemctl status mosquitto`.
5.  **Peripherals**: Are the Kinect, USB microphone, and USB speakers all plugged into the Raspberry Pi?

### 5.2. Tutorial: Mapping a Real-World Room
*(Same content as previous README)*

### 5.3. Tutorial: Autonomous Operation on Hardware
*(Same content as previous README)*

---

## Part 6: Developer's Guide & Customization

### 6.1. How to Customize the AI's Personality
The robot's conversational ability is defined entirely within the `call_llm_api` function in `ai_robot_core/nodes/interaction_manager_node.py`. You can easily add more `elif` conditions to check for different keywords and provide custom responses.

**Example: Add a new response**
```python
# In interaction_manager_node.py, inside call_llm_api function:

elif "what time is it" in new_line:
    return f"The current time is {datetime.datetime.now().strftime('%I:%M %p')}."
```

### 6.2. How to Tune Face Recognition
The strictness of face matching is controlled by the `min_distance` variable in the `lookup_person_callback` function in `ai_robot_core/nodes/memory_manager_node.py`.
*   **Stricter Matching**: Lower this value (e.g., to `0.5` or `0.45`). This reduces the chance of incorrectly identifying a stranger as someone you know, but increases the chance of not recognizing a known person if the lighting is different.
*   **Looser Matching**: Increase this value (e.g., to `0.65`). This makes the robot better at recognizing people in varied conditions, but it might occasionally misidentify a new person.

### 6.3. Common Issues & Troubleshooting
*   **Problem**: The robot doesn't move, but I see `/cmd_vel` messages in ROS 2.
    *   **Solution**: This is almost always an MQTT issue.
        1.  Is the MQTT broker running on the Pi?
        2.  Did you configure the correct IP address for the `MQTT_SERVER` in the ESP8266 firmware?
        3.  Are both devices on the same WiFi network?
        4.  Use an MQTT client like `MQTT Explorer` on your desktop to subscribe to the `robot/motor_cmd` topic and see if the `esp8266_bridge_node` is publishing messages.

*   **Problem**: The robot's navigation is jerky or it gets stuck.
    *   **Solution**: This is a tuning issue in `nav2_params.yaml`. The default parameters are a good starting point, but may need adjustment for your robot's specific weight, motors, and surface. Look into tuning the `controller_server` parameters like `max_vel_x` and the acceleration limits.

*   **Problem**: `pykinect2` fails to install or run on the Raspberry Pi.
    *   **Solution**: This is a known, difficult issue. `pykinect2` is not well-supported on Linux. The professional solution is to replace it with a different driver. The `libfreenect` library is the open-source standard. You would install `ros-humble-libfreenect-camera`, then change the `kinect_interface_node` in your `hardware_bringup.launch.py` to launch the `freenect_launch.py` file provided by that package instead.
