# `Bipedal_description` ROS 2 Package

The **`Bipedal_description`** package is the primary ROS 2 package containing the main URDF/Xacro models, visualization configurations, Gazebo simulation launch files, and alignment calibration tools for the Bipedal Walking Robot.

---

## 📂 Subdirectory Structure

| Directory | Description |
| :--- | :--- |
| [`bipedal_description/`](./bipedal_description/) | Python package root module initialization marker. |
| [`config/`](./config/) | RViz display configuration profiles (`display.rviz`). |
| [`launch/`](./launch/) | ROS 2 Python launch scripts (`display.launch.py`, `gazebo.launch.py`). |
| [`meshes/`](./meshes/) | STL 3D mesh files for base link, upper/lower femurs, tibias, and inner linkages. |
| [`resource/`](./resource/) | Package resource index marker for ROS 2 package discovery. |
| [`rviz/`](./rviz/) | Additional RViz environment profiles (`urdf.rviz`). |
| [`test/`](./test/) | Code formatting and quality tests (`flake8`, `pep257`, `copyright`). |
| [`urdf/`](./urdf/) | Core Xacro specifications, transmission definitions, Gazebo tags, and alignment scripts (`align.py`, `fix.py`, `sync.py`). |

---

## 🚀 Execution Commands

### Visualize in RViz 2
```bash
ros2 launch Bipedal_description display.launch.py
```

### Launch Gazebo Simulation
```bash
ros2 launch Bipedal_description gazebo.launch.py
```
