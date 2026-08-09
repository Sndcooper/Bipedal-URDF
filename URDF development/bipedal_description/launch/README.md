# Launch Files (`bipedal_description/launch`)

Contains Python-based launch configurations for ROS 2 simulation and visualization environments.

## Available Launch Scripts

### 1. `display.launch.py`
Launches the robot visualizer:
- Processes `Bipedal.xacro` into URDF XML.
- Starts `robot_state_publisher` to publish TF transforms.
- Runs `joint_state_publisher_gui` (or non-GUI fallback) to manipulate joint angles interactively.
- Opens `rviz2` with `config/display.rviz`.

### 2. `gazebo.launch.py`
Spawns the robot model into a Gazebo physics world:
- Starts Gazebo server (`gzserver`) and client (`gzclient`).
- Publishes robot description TF tree.
- Uses `spawn_entity.py` to place the robot at world origin in Gazebo.
