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
| `left_hip_rear`, `left_hip_front` | revolute | ±30° | actuated |
| `right_hip_rear`, `right_hip_front` | revolute | ±30° | actuated |
| `left_knee_rear`, `right_knee_rear` | revolute | −60°…+80° | passive, set by closure |
| `left_knee_front`, `right_knee_front` | revolute | −60°…+80° | passive, set by closure |
| `*_foot_closure` | — | — | the cut joint; Gazebo only |

**The knee limits are wider than the closure ever uses.** Driven correctly
(hips within the real ±30° range), the knees only ever reach about ±58°/46° —
see the "practical ceiling" note below. The extra headroom to −60°/+80° exists
so the joints can be posed by hand past the mechanism's working range, e.g. in
a static URDF viewer where you drag one joint at a time.

**Widening these does not make a static viewer show the leg closed.** A knee's
correct position is a function of *both* hip angles — a nonlinear one (an arc
closing, not a line) — and a viewer that drags joints independently has no way
to enforce that. This is a property of representing a closed loop as an open
tree, not a limit that can be tuned away; URDF's only joint-coupling feature,
`<mimic>`, is strictly linear and the best possible linear fit to this
relationship misses by **up to 370 mm** at ±60–80°. See
[`doc/five_bar_viewer.html`](doc/five_bar_viewer.html) — an interactive page
running the exact same solver, with a "Manual" mode that reproduces this
failure live so you can see why widening the limit doesn't help. RViz
`mode:=demo` / `mode:=gui` is the equivalent for the real robot and is exact.

### Zero pose and why the hip range is what it is

`zero_splay` in the xacro sets how far each femur sits from vertical at q = 0
(rear at −splay, front at +splay). It defaults to **0 — both femurs hang
straight down.**

The binding constraint on hip travel is **not** the loop closure, it is the two
femurs of a leg hitting each other: they are coplanar, and counter-rotating them
closes the gap between their knee pivots as
`hip_spacing − 2·femur_len·sin(range)`. At `zero_splay = 0` that gap reaches
zero at **33.06°**, so 30° is the practical ceiling — and even there the worst
corner (rear +30, front −30) leaves only **5 mm** between the pivots.

Two consequences worth knowing:

- **The default pose is nearly fully extended.** Foot depth is 223.2 mm and the
  mechanism's absolute maximum reach is 227.5 mm. Over the whole hip box the leg
  travels only 203.5 → 224.7 mm, about **21 mm**. Splaying the zero pose gets
  that range back: the raw CAD pose is ~52° of splay, sits at 176.6 mm, and
  tolerated ±60° hips.
- **`demo` mode only crouches.** The sweep runs one-sided (0 → +spread) rather
  than symmetric about zero, because there is nothing above the default to
  extend into.

If you want a stance that stands rather than locks out, set:

```xml
<xacro:property name="zero_splay" value="0.5" />   <!-- ~29 deg -->
```

then re-derive `hip_range`, the four knee limits and `foot_overhang`. Everything
else — the baked joint rotations, `stand_height`, `phi_zero` — recomputes itself
from `zero_splay`.

> **Keep `ZERO_SPLAY` in `five_bar_state_publisher.py` equal to `zero_splay` in
> the xacro.** The xacro bakes the matching rotations into the joint origins;
> the node has to measure its angles from the same reference or the published
> knee values will be offset and the linkage will render open.

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
