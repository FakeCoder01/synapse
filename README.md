# Synapse : AI Pet Robot Simulation

This project is a complete ROS 2 Humble simulation of an intelligent, interactive mobile robot. It includes a full navigation stack (SLAM, Nav2) and an AI "brain" for face recognition, long-term memory, and autonomous interaction.

## Features

- **Simulation**: A 4-wheel differential drive robot with a 3D depth camera in a Gazebo world with various obstacles.
- **Navigation**:
  - **SLAM**: Uses `slam_toolbox` for creating a map of the environment.
  - **Localization**: Uses `amcl` for localizing the robot in a pre-existing map.
  - **Path Planning**: Uses `Nav2` for planning and following a path to a goal.
- **AI Brain**:
  - **Face Recognition**: Detects faces in the camera feed and uses a stubbed recognition system to identify them. New faces are registered in a database.
  - **Long-Term Memory**: A SQLite database stores information about known people and past conversations.
  - **Autonomous Interaction**: The robot can autonomously greet new and known people, referencing its memory of past interactions.
  - **Person Following**: The robot can be commanded to follow a specific person by publishing the person's ID to the `/follow_person_start` topic.
- **UI**: A custom RQT plugin serves as the robot's "face", showing the live camera feed with annotations and the robot's current "emotion" (🙂 for happy, 🤔 for curious, 😐 for neutral).

## Dependencies

- ROS 2 Humble
- `gazebo_ros`
- `slam_toolbox`
- `nav2_bringup`
- `python3-opencv`
- `rqt`
- All other standard ROS 2 packages.

## Build Instructions

1.  Clone this repository into a new workspace's `src` directory (e.g., `ai_robot_ws/src`).
2.  Navigate to the workspace root: `cd ai_robot_ws`
3.  Install dependencies: `rosdep install -i --from-path src -y`
4.  Build the workspace: `colcon build --symlink-install`

## How to Run the Simulation

Source the workspace in every new terminal: `source install/setup.bash`

### Task 1: Mapping

This launch file starts the simulation and SLAM. Drive the robot around with your keyboard to build a map.

```bash
# Launch the mapping stack
ros2 launch ai_robot_core mapping.launch.py
```

**In another terminal, run `teleop_twist_keyboard`:**

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

When you have a good map, save it:

```bash
# Make a 'maps' directory if it doesn't exist
mkdir -p maps
# Save the map
ros2 run nav2_map_server map_saver_cli -f maps/my_map
```

### Task 2: Navigation

This launch file starts the simulation and the full Nav2 stack, loading your saved map. You can set navigation goals in RViz.

```bash
# Launch the navigation stack
ros2 launch ai_robot_core navigation.launch.py map_file:=$(pwd)/maps/my_map.yaml
```

### Task 3: Full AI Application (The "Everything" File)

This starts everything: the simulation, Nav2, and all AI "brain" nodes and the UI.

```bash
# Launch the full application
ros2 launch ai_robot_core full_ai_app.launch.py map_file:=$(pwd)/maps/my_map.yaml
```

The robot will now autonomously navigate and interact with people (or a simple human model) you can add to the Gazebo world.

## How to Deploy on Real Hardware

To run this project on a real robot, you will need to make some modifications to the launch files and potentially the code.

### Hardware Requirements

- A differential drive robot base with ROS 2 drivers that publish odometry (`/odom`) and subscribe to velocity commands (`/cmd_vel`).
- A computer to run the ROS 2 nodes (e.g., a Raspberry Pi 4 or a laptop).
- A 3D camera (e.g., Intel RealSense) with ROS 2 drivers that publish camera images and depth information.

### Configuration for Real Hardware

1.  **Disable Gazebo**: In the launch files (`mapping.launch.py`, `navigation.launch.py`, `full_ai_app.launch.py`), you will need to remove or comment out the Gazebo launch include.

2.  **Use Real Sensor Drivers**: Instead of launching Gazebo, you will need to launch the drivers for your robot's hardware (robot base, camera, etc.).

3.  **Update `use_sim_time`**: In all launch files and configuration files, you will need to set `use_sim_time` to `false`.

### Example Launch File for Real Hardware

Here is an example of how you might modify the `full_ai_app.launch.py` for a real robot:

```python
# full_ai_app_real.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_ai_robot_core = get_package_share_directory('ai_robot_core')
    map_file = LaunchConfiguration('map_file')

    # Real robot drivers (replace with your robot's drivers)
    robot_drivers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('your_robot_driver_package'), 'launch', 'your_robot.launch.py')
        )
    )

    # Navigation
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ai_robot_core, 'launch', 'navigation.launch.py')
        ),
        launch_arguments={'map_file': map_file, 'use_sim_time': 'false'}.items()
    )

    # Brain
    # ... (the same as in the simulation launch file)

    # UI
    # ... (the same as in the simulation launch file)

    return LaunchDescription([
        DeclareLaunchArgument(
            'map_file',
            default_value=os.path.join(pkg_ai_robot_core, 'maps', 'my_map.yaml'),
            description='Full path to map file to load'),
        robot_drivers,
        navigation,
        # ... (the rest of the nodes)
    ])
```

## How to Customize

- **LLM Integration**: The "brain" is in `ai_robot_core/nodes/interaction_manager_node.py`. WIP
- **Face Recognition**: The recognition logic in `face_recognition_node.py` is a stub. You can replace the fake embedding with a real one (e.g., from `face_recognition` or a similar library) to enable true recognition.
