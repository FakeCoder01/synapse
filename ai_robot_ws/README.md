---

# Chapter 5: Bringing the Physical Robot to Life

This chapter guides you through the process of running the ROS 2 stack on your fully assembled robot.

## 5.1. Pre-Launch Checklist

Before you attempt to launch the software on your robot, go through this checklist meticulously. Skipping a step is the most common source of errors.

1.  **Power On**:
    *   [ ] The main motor battery (7.4V-12V) is charged and connected to the L298N `12V` and `GND` terminals.
    *   [ ] The Raspberry Pi's power bank is charged and connected to the Pi's USB-C power port.
    *   [ ] The ESP8266 should have a light on, indicating it's receiving 5V power from the L298N.

2.  **Network Connectivity**:
    *   [ ] The Raspberry Pi has booted and is connected to your WiFi network.
    *   [ ] The ESP8266 has automatically connected to the same WiFi network. You can verify this by opening the Arduino IDE's Serial Monitor and watching its boot-up messages; it will print its IP address.
    *   [ ] From a terminal on the Raspberry Pi, successfully `ping` the ESP8266's IP address. If this fails, your devices are not on the same network, and nothing else will work.

3.  **MQTT Broker**:
    *   [ ] The Mosquitto MQTT broker is running on the Raspberry Pi. Verify with `systemctl status mosquitto`. It should say "active (running)".
    *   [ ] **(Optional but Recommended)** Use a tool like `MQTT Explorer` on a desktop computer to connect to your Raspberry Pi's IP address. You should see the ESP8266 connect and start publishing messages to the `robot/telemetry` and `robot/status` topics. This confirms the entire communication link is working before ROS is even involved.

4.  **ROS 2 Environment**:
    *   [ ] You have an SSH session open to your Raspberry Pi.
    *   [ ] You have navigated to your workspace (`cd ~/ai_robot_ws`) and sourced the setup file (`source install/setup.bash`).

## 5.2. Tutorial: Mapping a Real-World Room

You cannot use navigation in a new space without first creating a map. This process uses SLAM to build the map from live sensor data.

1.  **Place the Robot**: Put your robot on the floor in a relatively open area of the room you want to map. The robot will consider this its `(0,0)` origin point.

2.  **Launch the Mapping Stack**: On your Raspberry Pi, run the following command. This launch file is specifically designed for SLAM on hardware. It starts the hardware drivers and the `slam_toolbox` node.
    ```bash
    ros2 launch ai_robot_core mapping.launch.py use_sim_time:=false
    ```
    *   **`use_sim_time:=false`**: This is the most critical part. It tells all nodes to use the system's real-world clock instead of a simulated clock from Gazebo. **This argument is mandatory for all hardware operations.**

3.  **Launch Keyboard Control**: Open a **second SSH terminal** to your Raspberry Pi. In this new terminal, navigate to your workspace, source it, and launch the teleop node:
    ```bash
    cd ~/ai_robot_ws && source install/setup.bash
    ros2 launch ai_robot_core teleop.launch.py
    ```

4.  **Visualize and Build the Map**:
    *   On a separate, more powerful desktop computer that is on the **same WiFi network** as the robot, install ROS 2 Humble Desktop.
    *   Source the ROS 2 setup file (`source /opt/ros/humble/setup.bash`).
    *   Launch RViz: `rviz2`.
    *   In RViz, change the "Global Options" -> "Fixed Frame" from `map` to `odom`.
    *   Click the "Add" button in the bottom-left, and add the following displays:
        *   By topic -> `/map` -> `Map`
        *   By topic -> `/scan` -> `LaserScan`
        *   `TF`
    *   You should now see the robot's TF frames and the laser scan data. As you drive the robot using the teleop terminal on the Pi, you will see the map being built in RViz.

5.  **Save the Map**: Once the map is complete, open a **third SSH terminal** to the Pi, source the workspace, and run the map saver:
    ```bash
    cd ~/ai_robot_ws
    mkdir -p maps
    ros2 run nav2_map_server map_saver_cli -f maps/my_map
    ```
    This saves `maps/my_map.yaml` and `maps/my_map.pgm` inside a new `maps` directory on your Raspberry Pi. You have now successfully mapped your environment. You can `Ctrl+C` the mapping and teleop terminals.

## 5.3. Tutorial: Autonomous Operation on Hardware

Now that you have a map, you can run the robot in its fully autonomous, interactive mode.

1.  **Install Hardware Dependencies**: If you haven't already, install the necessary Python packages on the Raspberry Pi:
    ```bash
    sudo apt-get install python3-paho-mqtt python3-pyttsx3 python3-pyaudio
    # WARNING: pykinect2 is difficult to install on Linux. This is a known
    # challenge. You may need to research how to build it from source or
    # use an alternative like the 'ros-humble-libfreenect-camera' package.
    ```
2.  **Launch the Main Hardware Stack**: This single command starts all hardware drivers, the AI stack, and the Nav2 stack in navigation mode.

    **You must provide the full, absolute path to your map file.**

    ```bash
    # Source your workspace
    source install/setup.bash
    
    # Launch the hardware bringup, providing the path to your map
    ros2 launch ai_robot_hardware_drivers hardware_bringup.launch.py map:=/home/ubuntu/ai_robot_ws/maps/my_map.yaml
    ```
    *   **Dissecting the command**:
        *   `ros2 launch ai_robot_hardware_drivers hardware_bringup.launch.py`: This runs the main launch file from the hardware driver package.
        *   `map:=/path/to/your/map.yaml`: This **launch argument** is critical. It passes the location of your map file into the launch process, where it is then used by the Nav2 `map_server` node. Without a valid map, Nav2 will fail to start.

3.  **Localize and Interact**:
    *   Launch RViz on your desktop computer as you did during mapping.
    *   The robot will now attempt to localize itself using AMCL. You may need to give it an initial pose using the "2D Pose Estimate" tool in RViz.
    *   Once localized, you can give it navigation goals.
    *   Walk in front of the robot. It should detect your face, and you can begin interacting with it through voice commands.

---

# Chapter 6: Developer's Guide & Customization

## 6.1. How to Customize the AI's Personality

The robot's conversational ability is defined entirely within the `call_llm_api` function in `ai_robot_core/nodes/interaction_manager_node.py`. This function is a simple state machine that you can easily expand.

**Example: Add a new response**
```python
# In interaction_manager_node.py, inside call_llm_api function:

elif "what time is it" in new_line:
    # Add an import for datetime at the top of the file
    import datetime
    return f"The current time is {datetime.datetime.now().strftime('%I:%M %p')}."
```
After saving the file, the change will take effect immediately because the workspace was built with `--symlink-install`.

## 6.2. How to Tune Face Recognition
The strictness of face matching is controlled by the `min_distance` variable in the `lookup_person_callback` function in `ai_robot_core/nodes/memory_manager_node.py`. This value is the threshold for the Euclidean distance between face vectors.

*   **Stricter Matching**: Lower this value (e.g., to `0.5` or `0.45`). This reduces the chance of incorrectly identifying a stranger as someone you know, but increases the chance of not recognizing a known person if the lighting is different.
*   **Looser Matching**: Increase this value (e.g., to `0.65`). This makes the robot better at recognizing people in varied conditions, but it might occasionally misidentify a new person.

## 6.3. Common Issues & Troubleshooting
*   **Problem**: The robot doesn't move, but I see `/cmd_vel` messages in ROS 2 (`ros2 topic echo /cmd_vel`).
    *   **Cause**: This is almost always a communication breakdown between the Raspberry Pi and the ESP8266.
    *   **Solution**:
        1.  **Check MQTT**: Is the broker running on the Pi (`systemctl status mosquitto`)?
        2.  **Check IP**: Did you configure the correct IP address for the `MQTT_SERVER` in the ESP8266 firmware?
        3.  **Check Network**: Are both devices on the same WiFi network with no client isolation?
        4.  **Debug with MQTT Explorer**: Use a desktop MQTT client to subscribe to the `robot/motor_cmd` topic. Do you see JSON messages appearing when you try to drive the robot? If not, the problem is in the `esp8266_bridge_node`. If you do, the problem is in the ESP8266's connection or code.

*   **Problem**: The robot's navigation is jerky, overshoots its goal, or gets stuck.
    *   **Cause**: The default Nav2 parameters are not tuned for your robot's specific dynamics (weight, wheel friction, motor power).
    *   **Solution**: This is a complex topic called "tuning." Start by looking at the `controller_server` parameters in `ai_robot_core/config/nav2_params.yaml`. Try slightly reducing `max_vel_x` and the acceleration limits (`acc_lim_x`, `acc_lim_theta`) to make the robot's movements smoother.

*   **Problem**: The `kinect_interface_node` fails to start with an error about `pykinect2`.
    *   **Cause**: `pykinect2` is a wrapper around the official Microsoft Kinect SDK, which is designed for Windows and is notoriously difficult to get working on Linux.
    *   **Solution**: The professional solution is to replace it with a different driver that uses `libfreenect`, the open-source Kinect library.
        1.  Install the ROS 2 `libfreenect` driver: `sudo apt-get install ros-humble-freenect-camera`
        2.  In `ai_robot_hardware_drivers/launch/hardware_bringup.launch.py`, comment out the `kinect_node` and replace it with an `IncludeLaunchDescription` for the `freenect_launch.py` file provided by the new package. You will also need to adjust the topic remappings.