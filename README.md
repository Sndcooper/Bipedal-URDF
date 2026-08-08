# 🤖 ROS 2 Bipedal Robot Description & URDF Workspace (`Bipedal-URDF`)

[![ROS 2 Distribution](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![License](https://img.shields.io/badge/License-Apache%202.0-green.svg)](LICENSE)
[![Build Status](https://img.shields.io/badge/build-passing-brightgreen.svg)]()

Welcome to the **Bipedal Robot URDF & Simulation Repository** (`Bipedal-URDF`). This repository contains the complete kinematics definitions, 3D CAD mesh representations, Gazebo physics configurations, RViz display profiles, and URDF/Xacro models for a **5-bar parallel-linkage bipedal walking robot**.

---

## 📌 Repository Organization Note

> [!NOTE]
> The primary ROS 2 workspace and package modules are located inside the [`biped/`](file:///c:/Users/vilas/Documents/CIR/ROS%202%20hum/biped) directory. When building with `colcon`, point your workspace source directory to `biped/` or source from the root.

---

## 🦾 Bipedal Robot Hardware & Kinematics Overview

The robot is engineered around a **5-bar closed kinematic chain leg design**. Moving heavy actuators away from distal leg segments to the central pelvis reduces swinging inertia and enables rapid, dynamic walking strides.

### Kinematic & Structural Breakdown:
- **Pelvis & Main Base (`base_link`)**: Houses primary compute, IMU sensor suites, Bonka LiPo battery packs, and high-torque Dynamixel AX-12 servo motors.
- **Actuated Parallel Femur Linkages**:
  - **`right_upper_femur` / `left_upper_femur`**: Primary upper driven femur links.
  - **`right_lower_femur` / `left_lower_femur`**: Secondary parallel femur links ensuring pitch stabilization.
- **Distal Tibia & Foot Assemblies**:
  - **`right_tibia` / `left_tibia`**: Lightweight structural lower legs terminating in dynamic ground contact pads.
  - **`inner_linkage_1` / `inner_linkage_2`**: Transmission links transferring torque across closed loops.

---

## 📁 Repository & Package Directory Layout

Below is the directory map starting from the [`biped/`](file:///c:/Users/vilas/Documents/CIR/ROS%202%20hum/biped) directory:

```
ROS 2 hum/
├── README.md                            # Main repository documentation (this file)
└── biped/                               # Primary ROS 2 Workspace Root
    ├── README.md                        # Workspace detailed guide
    ├── ROS 2 hum.code-workspace         # VS Code workspace settings
    ├── bipedal_description/             # Cleaned & Aligned Kinematic ROS 2 Package
    │   ├── README.md                    # Package overview
    │   ├── config/                      # RViz visualization settings
    │   ├── launch/                      # ROS 2 display & Gazebo launch scripts
    │   ├── meshes/                      # High-precision link STL models
    │   ├── rviz/                        # RViz display profiles (`urdf.rviz`)
    │   ├── test/                        # Quality & linting tests (flake8, pep257)
    │   └── urdf/                        # Modular Xacro, visual URDF, and python scripts
    ├── bipedal_URDF/                    # CAD Assembly Package Container
    │   ├── README.md                    # Assembly container overview
    │   └── assembly_description/        # Integrated assembly package
    │       ├── launch/                  # Assembly display launch scripts
    │       ├── meshes/                  # Full assembly STL models
    │       └── urdf/                    # Integrated `assembly.xacro` & generator script
    └── bipedal_export_temp_description/ # CAD Raw Export Package
        ├── meshes/                      # Unprocessed motor & chassis STLs
        └── urdf/                        # Raw exported URDF & visual models
```

---

## 📐 URDF Files & Models Summary

| Package | Key Model Files | Description |
| :--- | :--- | :--- |
| **`bipedal_description`** | [`bipedal.xacro`](file:///c:/Users/vilas/Documents/CIR/ROS%202%20hum/biped/bipedal_description/urdf/bipedal.xacro)<br>[`bipedal_visual.urdf`](file:///c:/Users/vilas/Documents/CIR/ROS%202%20hum/biped/bipedal_description/urdf/bipedal_visual.urdf) | Parameterized, clean Xacro model and standalone visual URDF for kinematics calculation and Gazebo simulation. |
| **`assembly_description`** | [`assembly.xacro`](file:///c:/Users/vilas/Documents/CIR/ROS%202%20hum/biped/bipedal_URDF/assembly_description/urdf/assembly.xacro)<br>[`assembly.urdf`](file:///c:/Users/vilas/Documents/CIR/ROS%202%20hum/biped/bipedal_URDF/assembly_description/urdf/assembly.urdf) | Complete CAD assembly URDF built via [`generate_urdf.py`](file:///c:/Users/vilas/Documents/CIR/ROS%202%20hum/biped/bipedal_URDF/assembly_description/urdf/generate_urdf.py). |
| **`bipedal_export_temp_description`** | [`bipedal_export_temp.xacro`](file:///c:/Users/vilas/Documents/CIR/ROS%202%20hum/biped/bipedal_export_temp_description/urdf/bipedal_export_temp.xacro)<br>[`bipedal_clean_visual.urdf`](file:///c:/Users/vilas/Documents/CIR/ROS%202%20hum/biped/bipedal_export_temp_description/urdf/bipedal_clean_visual.urdf) | Raw output directly exported from Fusion 360 / SolidWorks URDF exporter. |

---

## ⚡ Quickstart Guide

### 1. System Requirements & Dependencies

Install ROS 2 Humble alongside required dependencies:

```bash
sudo apt update
sudo apt install -y \
  ros-humble-desktop \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-rviz2 \
  ros-humble-gazebo-ros-pkgs \
  python3-pytest
```

### 2. Build Workspace

Navigate to your workspace root (containing `biped/`) and build using `colcon`:

```bash
# Navigate to workspace
cd ~/ros2_ws/src
git clone https://github.com/Sndcooper/Bipedal-URDF.git

# Build all packages from biped workspace
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 3. Interactive Visualization in RViz 2

To view the robot with joint control sliders:

```bash
ros2 launch bipedal_description display.launch.py
```

### 4. Gazebo Physics Simulation

To launch the robot inside Gazebo physics engine:

```bash
ros2 launch bipedal_description gazebo.launch.py
```

---

## 🛠️ Python Automation Tools

Located in [`biped/bipedal_description/urdf/`](file:///c:/Users/vilas/Documents/CIR/ROS%202%20hum/biped/bipedal_description/urdf):
- **`align.py`**: Computes length and yaw alignments for femur linkages.
- **`fix.py`**: Corrects formatting errors and quote escaping in XML/Xacro files.
- **`sync.py`**: Keeps `bipedal_visual.urdf` synchronized with `bipedal.xacro`.
- **`generate_urdf.py`**: Expands Xacro includes into standalone URDFs.

---

## 📜 License

Distributed under the Apache 2.0 License. See `LICENSE` for details.
