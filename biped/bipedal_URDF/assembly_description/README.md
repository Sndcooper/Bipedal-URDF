# `assembly_description` ROS 2 Package

The **`assembly_description`** package contains the complete CAD assembly description of the bipedal robot. It integrates combined sub-assemblies (such as combined motor+tibia links and combined femurs) into a unified URDF model.

---

## 📂 Subdirectories

| Directory | Description |
| :--- | :--- |
| [`assembly_description/`](./assembly_description/) | Python module initialization marker. |
| [`config/`](./config/) | RViz visualization configuration profiles. |
| [`launch/`](./launch/) | Launch scripts (`display.launch.py`, `gazebo.launch.py`). |
| [`meshes/`](./meshes/) | Combined sub-assembly STL meshes (`Combined_femur_1.stl`, `combined_motor_tibia_1.stl`, `base_link.stl`). |
| [`resource/`](./resource/) | ROS 2 Ament index resource marker. |
| [`test/`](./test/) | Code formatting and quality unit tests (`flake8`, `pep257`, `copyright`). |
| [`urdf/`](./urdf/) | Integrated Xacro files (`assembly.xacro`), compiled URDF (`assembly.urdf`), and automated compiler script (`generate_urdf.py`). |

---

## 🚀 Quick Commands
```bash
# Launch RViz visualization
ros2 launch assembly_description display.launch.py

# Launch Gazebo simulation
ros2 launch assembly_description gazebo.launch.py
```
