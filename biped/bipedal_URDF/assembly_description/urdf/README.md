# Integrated Assembly URDF Models & Builder (`assembly_description/urdf`)

Contains full mechanical CAD assembly Xacro/URDF specifications and automated URDF generation tooling.

## Files

### Xacro & URDF Models
- **`assembly.xacro`**: Master Xacro file linking materials, transmission tags, Gazebo parameters, and main assembly links/joints.
- **`assembly.urdf`**: Monolithic URDF generated from `assembly.xacro` with resolved package relative paths.
- **`assembly.gazebo`**: Gazebo surface friction, joint damping, and plugin configuration.
- **`assembly.trans`**: Hardware transmission interfaces for ROS control actuators.
- **`materials.xacro`**: Material definitions and RGBA values.

### URDF Builder Tool
- **`generate_urdf.py`**: Python script that reads `assembly.xacro`, recursively expands `<xacro:include>` directives, converts `file://$(find assembly_description)` mesh URIs to standard `package://` paths, and generates a clean, standalone `assembly.urdf`.
