# `assembly_description` ROS 2 Package

Kinematic description of the bipedal robot: two **5-bar parallel closed-loop
legs**, 2 actuated DOF each.

---

## The mechanism

Each leg is a planar 5-bar in the sagittal plane:

```
        pelvis
  H_rear o========= 60 mm ========o H_front     ground link (base_link)
          \                      /
           \ 55 mm        55 mm /               the two femurs   (ACTUATED at the hips)
            \                  /
      K_rear o                o K_front         the two knees    (PASSIVE)
              \              /
       107.5   \            /   107.5 mm        the two shanks
                \          /
                 o========o
                  FOOT                          loop-closure pivot
```

Link lengths were measured off the CAD meshes (bolt-hole ring centres), and the
loop closes to within **1 micron** at the zero pose. The zero configuration is
the CAD assembly pose.

The two legs are related by a 180° rotation about the CAD vertical axis, not by
a mirror — so on the left leg the motor-carrying shank is the *rear* one, and on
the right leg it is the *front* one. That is how the assembly is built; it is
preserved here rather than "corrected".

---

## Why there is no joint for the foot pivot

URDF can only describe a kinematic **tree**. A 5-bar has one independent loop
per leg, so exactly one joint per leg has to be left out. Here that is the foot
pivot. Each leg contributes 4 tree joints (2 hips + 2 knees) plus 1 cut joint.

**The cut is not a bug, and it must not be "fixed" by giving the inner linkage a
second parent.** A link with two parents is invalid URDF, and that is precisely
what made an earlier version of this model load with dangling, disconnected
knees.

Instead:

- **RViz** — the `five_bar_state_publisher` node takes the 4 hip angles, solves
  each leg's closure analytically, and publishes all 8 joint values. The linkage
  then stays joined at the foot in every pose.
- **Gazebo** — the real revolute constraint is re-attached as an SDF joint in
  `urdf/assembly.gazebo`.

---

## Joints

| Joint | Type | Limits | Notes |
| :--- | :--- | :--- | :--- |
| `left_hip_rear`, `left_hip_front` | revolute | ±60° | actuated |
| `right_hip_rear`, `right_hip_front` | revolute | ±60° | actuated |
| `left_knee_rear`, `right_knee_rear` | revolute | −87.2°…+68.2° | passive, set by closure |
| `left_knee_front`, `right_knee_front` | revolute | −68.7°…+100.6° | passive, set by closure |
| `*_foot_closure` | — | — | the cut joint; Gazebo only |

The ±60° hip range was chosen by sweeping the closure solver: over that box
every pose is reachable, and the mechanism stays 45 mm clear of the stretched
singularity. The knee limits are the range the dependent knees actually sweep
over that hip box, plus a small margin. **Do not widen the hips to ±90°** without
re-checking self-collision — beyond about ±75° the linkage folds back through
itself.

---

## Frames

The CAD was exported with **+X down** and **+Z across the hips**, so loading it
raw makes the robot lie on its side. `base_footprint` re-expresses that in the
ROS convention (REP-103: X forward, Y left, Z up) through a single fixed joint,
and also recentres the model, which the CAD origin is not. The robot therefore
stands upright on the RViz grid with its feet straddling the origin at y = ±78 mm.

All link and joint numbers inside `base_link` are kept in the original CAD frame
so they can still be checked line-by-line against the Fusion export.

---

## 📂 Subdirectories

| Directory | Description |
| :--- | :--- |
| [`assembly_description/`](./assembly_description/) | Python module — contains `five_bar_state_publisher.py`. |
| [`config/`](./config/) | RViz configuration (`display.rviz`). |
| [`launch/`](./launch/) | `display.launch.py`, `gazebo.launch.py`. |
| [`meshes/`](./meshes/) | STL meshes, named by role (`left_femur_rear.stl`, `right_shank_front.stl`, …). |
| [`resource/`](./resource/) | Ament index marker. |
| [`test/`](./test/) | flake8 / pep257 / copyright tests. |
| [`urdf/`](./urdf/) | `assembly.xacro` (source of truth), `assembly.urdf` (generated), `generate_urdf.py`. |

---

## 🚀 Quick Commands

```bash
ros2 launch assembly_description display.launch.py
```

Sliders in RViz, with the loop kept closed. The GUI publishes to `hip_states`;
only the four hip values are used, and the knee sliders are ignored because the
closure determines them.

```bash
ros2 launch assembly_description display.launch.py mode:=demo
```

Animated crouch/extend cycle — the quickest way to confirm the linkage tracks
as one closed mechanism.

```bash
ros2 launch assembly_description display.launch.py mode:=raw
```

Escape hatch: drives all 8 joints independently. The legs **will** come apart at
the feet — that is the URDF tree without the closure constraint, and it is only
useful for inspecting the bare tree.

```bash
ros2 launch assembly_description gazebo.launch.py
```

---

## Which URDF file to open

| File | Use it for |
| :--- | :--- |
| `assembly.xacro` | **source of truth** — edit this |
| `assembly.urdf` | ROS: RViz, Gazebo, robot_state_publisher |
| `assembly_view.urdf` | **standalone URDF viewers** (VS Code URDF Visualizer, Foxglove, web viewers) |

`assembly.urdf` renders **blank** in drag-and-drop viewers, and this is not a
fault in the model. Those viewers search the document recursively for `joint`
elements and read `parent`/`child` as *attributes*. A ROS-complete URDF
contains three things that breaks:

1. `<transmission><joint name="..."/></transmission>` — a joint element with no
   parent or child at all.
2. The SDF loop-closure joints inside `<gazebo>`, which use
   `<parent>link_name</parent>` as element *text*, not an attribute.
3. `package://` mesh paths, which need a resolvable ROS package root.

A recursive search finds **17** "joints" in `assembly.urdf` — only 11 of them
are real. `assembly_view.urdf` strips the `<gazebo>` and `<transmission>`
blocks and uses `../meshes/...` paths, leaving exactly the 11 real joints. It is
generated from the same xacro, so the kinematics are identical.

Regenerate it alongside the main file after any edit:

```bash
python3 urdf/generate_urdf.py --standalone --output assembly_view.urdf
```

---

## Regenerating the plain URDF

`assembly.xacro` is the source of truth. After editing it:

```bash
python3 urdf/generate_urdf.py
```

This runs the real xacro parser (the old version of this script used regexes and
silently mangled anything it did not anticipate) and rewrites mesh paths to
`package://`.
