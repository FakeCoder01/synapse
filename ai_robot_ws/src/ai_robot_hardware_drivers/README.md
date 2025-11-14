# AI Robot Hardware Drivers

This package contains the necessary drivers and launch files to run the AI Pet Robot on physical hardware.

## Features

*   **Motor/Sensor Bridge**: The `esp8266_bridge_node` communicates via MQTT with an ESP8266 to control motors and receive sensor data. It also provides dead-reckoned odometry required for navigation.
*   **Camera Driver**: The `kinect_interface_node` provides a driver for an Xbox Kinect camera to publish color and depth images.
*   **Audio Interface**: The `audio_interface_node` uses the host computer's microphone and speakers for speech-to-text and text-to-speech, enabling voice interaction.

## Hardware Requirements

*   **Robot Base**: A differential drive robot with an ESP8266 running the provided firmware for motor control.
*   **Host Computer**: A small computer (e.g., Raspberry Pi 4 or laptop) to run the main ROS 2 stack.
*   **Camera**: An Xbox Kinect or similar 3D camera compatible with the `kinect_interface_node`.
*   **Audio**: A standard USB microphone and speakers connected to the host computer.
*   **MQTT Broker**: An MQTT broker running on your network, accessible by both the host computer and the ESP8266.

## Firmware Setup

1.  Open `firmware/esp_firmware_8266.ino` in the Arduino IDE.
2.  Update the `SSID`, `PASSWORD`, and `MQTT_SERVER` variables to match your network configuration.
3.  Verify your L298N motor driver wiring matches the pins defined in the firmware.
4.  Flash the firmware to your ESP8266.

## Launching the Real Robot

1.  **Build the Workspace**: Ensure you have built the workspace with `colcon build --symlink-install`.
2.  **Source the Workspace**: `source install/setup.bash`
3.  **Run the Hardware Launch File**: This command starts all the necessary drivers (camera, motors, audio) and the complete AI and navigation stack. You must provide the path to your pre-made map.

    ```bash
    # Launch the full hardware stack
    ros2 launch ai_robot_hardware_drivers hardware_bringup.launch.py map_file:=/path/to/your/maps/my_map.yaml
    ```

The robot should now be running entirely on its hardware, ready to navigate and interact in the real world.
