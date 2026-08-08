# `bipedal_export_temp_description` ROS 2 Package

The **`bipedal_export_temp_description`** package stores intermediate CAD exports generated directly from SolidWorks / Fusion 360 URDF exporter plugins. It serves as a reference and staging environment before mesh optimization and URDF consolidation into `bipedal_description`.

---

## 📁 Subdirectory Overview

| Directory | Purpose |
| :--- | :--- |
| [`bipedal_export_temp_description/`](./bipedal_export_temp_description/) | Python module initialization marker. |
| [`config/`](./config/) | RViz display profiles (`display.rviz`). |
| [`launch/`](./launch/) | Launch scripts for raw model testing (`display.launch.py`, `gazebo.launch.py`). |
| [`meshes/`](./meshes/) | Raw exported STL meshes (Dynamixel AX-12 motors, Bonka batteries, unmerged femur/tibia parts). |
| [`resource/`](./resource/) | Package discovery marker for ROS 2. |
| [`test/`](./test/) | Quality tests (`flake8`, `pep257`, `copyright`). |
| [`urdf/`](./urdf/) | Raw CAD export URDF and Xacro files (`bipedal_export_temp.xacro`, `bipedal_clean_visual.urdf`). |
