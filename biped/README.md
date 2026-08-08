# ROS 2 Bipedal Robot Description & Simulation Workspace

[![ROS 2 Distribution](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![License](https://img.shields.io/badge/License-Apache%202.0-green.svg)](LICENSE)
[![Build Status](https://img.shields.io/badge/build-passing-brightgreen.svg)]()

Comprehensive ROS 2 (Humble Hawksbill) package repository for modeling, visualizing, and simulating a **parallel-linkage bipedal walking robot**. This repository houses complete URDF/Xacro descriptions, Gazebo physics simulation configurations, RViz display profiles, 3D CAD STL meshes, and custom Python kinematics helper scripts.

---

## 🤖 System Overview

The robot features a **five-bar parallel linkage leg architecture** designed for high dynamic performance, low swinging inertia, and rigid load distribution.

### Key Hardware Specs & Kinematic Design:
- **Degree of Freedom (DoF)**: 4 active actuated joints per leg (Femur and Tibia active drivers) with closed-loop kinematic linkages.
- **Actuators**: Dynamixel AX-12 servo motors / high-torque digital servomotors mounted high on the pelvis to lower leg inertia.
- **Leg Kinematics**:
  - **Upper Femur**: Connects hip joint to primary upper leg drive link.
  - **Lower Femur**: Parallel link ensuring constrained pitch rotation.
  - **Inner Linkages 1 & 2**: Transmit torque to lower tibia sections.
  - **Tibia Assembly**: Lightweight structural lower leg ending in contact foot pads.
- **Main Chassis**: Central base link housing core avionics, power distribution, and battery units (Bonka LiPo battery mounts).

---

## 📁 Repository Structure

```
biped/
├── README.md                            # Root workspace documentation
├── bipedal_description/                  # Primary ROS 2 package for robot simulation & URDF
│   ├── README.md                        # Package documentation
│   ├── bipedal_description/             # Python package module root
│   ├── config/                          # RViz display configurations (`display.rviz`)
│   ├── launch/                          # ROS 2 launch files (`display.launch.py`, `gazebo.launch.py`)
│   ├── meshes/                          # Cleaned 3D CAD STL files for links and chassis
│   ├── resource/                        # Ament resource marker
│   ├── rviz/                            # RViz visualizer profiles (`urdf.rviz`)
│   ├── test/                            # Quality linting & unit tests (flake8, pep257, copyright)
│   └── urdf/                            # Core Xacro files (`bipedal.xacro`, `materials.xacro`, etc.) & tools
├── bipedal_export_temp_description/     # Raw/Intermediate CAD export package
│   ├── README.md                        # Package documentation
│   ├── bipedal_export_temp_description/ # Python module marker
│   ├── config/                          # Display settings
│   ├── launch/                          # Test launch files
│   ├── meshes/                          # Raw exported CAD STLs (AX-12, Bonka batteries, etc.)
│   ├── resource/                        # Ament resource marker
│   ├── test/                            # Standard ROS 2 lint tests
│   └── urdf/                            # Exported raw URDF models (`bipedal_export_temp.xacro`)
└── bipedal_URDF/                        # Assembly URDF packages container
    ├── README.md                        # Assembly container overview
    └── assembly_description/            # High-precision CAD assembly package
        ├── README.md                    # Assembly package documentation
        ├── assembly_description/        # Python module marker
        ├── config/                      # RViz configuration
        ├── launch/                      # Assembly launch scripts
        ├── meshes/                      # Combined assembly mesh assets
        ├── resource/                    # Ament index resource
        ├── test/                        # Quality tests
        └── urdf/                        # Integrated assembly model (`assembly.xacro`, `generate_urdf.py`)
```

---

## ⚙️ Dependencies & Prerequisites

Ensure your system has ROS 2 Humble installed alongside the following required packages:

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

---

## 🚀 Quickstart Guide

### 1. Build the Workspace
Clean and build the workspace using `colcon`:

```bash
# Navigate to your workspace root
cd ~/biped_ws/src
git clone https://github.com/Sndcooper/Bipedal-URDF.git .

# Build all packages
cd ~/biped_ws
colcon build --symlink-install
source install/setup.bash
```

### 2. Visualize Robot in RViz 2
Launch the interactive joint GUI and RViz visualizer:

```bash
ros2 launch Bipedal_description display.launch.py
```

### 3. Launch Gazebo Physics Simulation
Spawn the bipedal robot model into Gazebo simulation:

```bash
ros2 launch Bipedal_description gazebo.launch.py
```

---

## 🛠️ Calibration & Utility Tools

The `bipedal_description/urdf/` directory contains specialized utility scripts for link alignment and URDF updates:
- **`align.py`**: Computes vector lengths and yaws for femur links to ensure accurate joint origin offsets.
- **`fix.py`**: Repairs XML tag quotes and `rpy` rotational attributes across Xacro files.
- **`sync.py`**: Synchronizes URDF link properties and joint origins between `bipedal_visual.urdf` and `bipedal.xacro`.
- **`generate_urdf.py`**: Compiles modular Xacro includes into standalone URDFs (`assembly.urdf`).

---

## 📜 License

This project is licensed under the Apache 2.0 License. See [LICENSE](LICENSE) for details.
