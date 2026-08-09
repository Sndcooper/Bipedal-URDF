# URDF Specifications & Kinematics Tools (`bipedal_description/urdf`)

Contains complete Xacro / URDF definitions, joint transmission configurations, Gazebo physics surface parameters, color materials, and automated calibration scripts.

## File Breakdown

### Xacro & URDF Models
- **`bipedal.xacro` / `bipedal.urdf.xacro`**: Primary macro definition file containing links, inertial matrices, visuals, collisions, and joint constraints.
- **`bipedal_visual.urdf`**: Cleaned standalone URDF compiled for fast visual rendering.
- **`materials.xacro`**: Color definitions (silver, black, dark grey, etc.) for visual links.
- **`bipedal.trans`**: ROS control transmission tags mapping joint actuators to `hardware_interface/PositionJointInterface`.
- **`bipedal.gazebo`**: Gazebo-specific properties (`<gazebo reference="...">`) specifying friction coefficients (`mu1`, `mu2`), stiffness (`kp`, `kd`), and plugin definitions (`gazebo_ros_control`).

### Python Automation & Utility Scripts
- **`align.py`**: Calculates 2D hypotenuse lengths and yaw angles for femur joints, automatically adjusting parent `rpy` and child `xyz` offsets.
- **`fix.py`**: Fixes corrupted XML `rpy` attribute strings and missing quotes across Xacro files.
- **`sync.py`**: Synchronizes joint origins and link dimensions between `bipedal_visual.urdf` and `bipedal.xacro`.
- **`rename_script.py`**: Standardizes link/joint naming conventions across legacy exports.
