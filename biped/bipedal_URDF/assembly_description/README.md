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

The two legs are related by a 180° rotation about the CAD X axis, not by a
mirror — so on the left leg the motor-carrying shank is the *rear* one, and on
the right leg it is the *front* one. That is how the assembly is built; it is
preserved here rather than "corrected".

**That rotation describes the part geometry, not the joint kinematics.** Both
legs use `axis="0 0 1"` with joint origins differing by under 0.01 rad, so
**the legs are not sign-inverted** — the same joint value produces the same
motion on each. The sign asymmetry in this model is *rear vs front*, never
*left vs right*.

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
- **Plain URDF viewers, no ROS at all** — `assembly_view.urdf` carries a
  hardcoded `<mimic>` chain that holds the loop shut on its own. See below.

---

## Standalone viewer mode (mimic-coupled 1-DOF) ⭐

> **This is the bottleneck-breaker.** Every other way of closing this loop needs
> something *running* — a ROS node, a physics engine. `assembly_view.urdf` closes
> it with nothing but the XML. Drag one slider in a VS Code URDF Visualizer,
> Foxglove, or any web viewer and the whole 5-bar tracks as one piece. No ROS, no
> Python, no build step.

### How it works

URDF's `<mimic>` is strictly linear: `q = multiplier · q_driver + offset`. That
cannot express a 2-DOF closure, whose knee angle depends nonlinearly on *two*
hip angles. The trick is to stop trying — **give up the second DOF on purpose**
and collapse each leg onto its single most useful path: the symmetric spread
that produces crouch.

One driver per leg, three followers:

| Joint | Mimics | Multiplier | Why |
| :--- | :--- | ---: | :--- |
| `*_hip_rear` | — *(driver)* | — | the one free slider per leg |
| `*_hip_front` | `*_hip_rear` | **−1.0** | counter-rotates the femurs → symmetric spread |
| `*_knee_rear` | `*_hip_rear` | **−1.477400** | minimax fit to the true closure |
| `*_knee_front` | `*_hip_rear` | **+1.477400** | mirror of the rear knee |

Along that symmetric path the true knee-to-driver ratio is *nearly* constant —
it only drifts from **−1.5400** at −20° to **−1.4416** at −80°. That narrow
spread is what makes a single multiplier viable at all, and it is specific to
this path; the ratio is not this well-behaved anywhere else in the workspace.

### Why −1.477400 and not the exact tangent

Fitting the multiplier exactly at the zero pose gives −1.532795 and a **15.63 mm**
gap at full crouch. `−1.477400` is instead the **minimax** fit over the whole
driver range, trading a fraction of a millimetre near zero for a **2.4×** better
worst case:

| Fit | Multiplier | Worst gap |
| :--- | ---: | ---: |
| Tangent at zero pose | −1.532795 | 15.63 mm |
| Least-squares | −1.492859 | 9.14 mm |
| **Minimax (used)** | **−1.477400** | **6.48 mm** |

Measured residual across the driver's full −80°…+30° range:

| Driver | Foot gap | Foot depth |
| ---: | ---: | ---: |
| 0° | **0.001 mm** | 223.23 mm |
| −30° | 5.79 mm | 205.23 mm |
| −70° | 0.42 mm | 153.45 mm |
| −80° | 6.48 mm | 137.13 mm |

The gap is **not monotonic** — it peaks near −40°, dips to 0.42 mm at −70°, then
rises again. That is the straight-line fit crossing the true curve twice, and it
is expected. Against 361 mm uncoupled, a 6.48 mm worst case is a **56× improvement**.

### The trade-off, stated plainly

**This costs you the second DOF.** Each leg becomes a 1-DOF stiff pendulum: you
get crouch, and only crouch. Poses needing the hips to move independently — foot
lift, asymmetric stance, anything off the symmetric path — are unreachable in
this file. That is a deliberate exchange of workspace for zero-dependency
rendering, and it is why the mimic chain lives **only** in `assembly_view.urdf`.

`assembly.xacro` and `assembly.urdf` stay mimic-free and fully 2-DOF, because
the ROS path runs the exact solver and has no reason to give anything up.

> ⚠️ `assembly_view.urdf` is a generated file. Re-running `generate_urdf.py
> --standalone` **overwrites the mimic chain**, since the xacro does not contain
> it. Re-apply the four blocks per leg after regenerating, or keep a patch.

---

## Joints

Hip travel is **directional**, not symmetric — see
[Hip sign convention](#hip-sign-convention) below.

| Joint | Type | Limits | Notes |
| :--- | :--- | :--- | :--- |
| `left_hip_rear`, `right_hip_rear` | revolute | −80°…+30° | actuated |
| `left_hip_front`, `right_hip_front` | revolute | −30°…+80° | actuated |
| `left_knee_rear`, `right_knee_rear` | revolute | −60°…+120° | passive, set by closure |
| `left_knee_front`, `right_knee_front` | revolute | −120°…+60° | passive, set by closure |
| `*_foot_closure` | — | — | the cut joint; Gazebo only |

**The knee limits must cover everything the closure can produce.** Swept over
the full hip box, the closure actually reaches `knee_rear ∈ [−58.87°, +115.32°]`
and `knee_front ∈ [−115.32°, +58.87°]`; the bounds above are those rounded out
to ±120°. Note the knees are **mirrored in sign** (rear opens positive, front
negative), matching the hips. An earlier symmetric −60°/+80° pair was both too
narrow on the spread side and wrong-handed on the front knee, which made the
solver command poses the URDF itself declared illegal.

**Widening these does not, on its own, make a static viewer show the leg
closed.** A knee's correct position is a function of *both* hip angles, and a
viewer that drags joints independently has no way to enforce that. Left
uncoupled, the feet separate by up to **361 mm** — wider than the robot is
tall — and dragging all four sliders freely reaches **383 mm**. That is a
property of representing a closed loop as an open tree, not a limit that can be
tuned away.

There is, however, a way to get a *usable* closed render out of a plain viewer
with no solver running — see
[Standalone viewer mode](#standalone-viewer-mode-mimic-coupled-1-dof) below.
See also [`doc/five_bar_viewer.html`](doc/five_bar_viewer.html), an interactive
page running the exact solver, whose "Manual" mode reproduces the uncoupled
failure live. RViz `mode:=demo` / `mode:=gui` is the equivalent for the real
robot and is exact.

### Zero pose and why the hip range is what it is

`zero_splay` in the xacro sets how far each femur sits from vertical at q = 0
(rear at −splay, front at +splay). It defaults to **0 — both femurs hang
straight down.**

The binding constraint on hip travel is **not** the loop closure, and it applies
in one direction only. Closing the femurs makes them hit each other: they are
coplanar, and counter-rotating them shrinks the gap between their knee pivots as
`hip_spacing − 2·femur_len·sin(range)`. At `zero_splay = 0` that gap reaches
zero at **33.06°**, so **30° inward** is the ceiling — and even there the worst
corner (rear +30, front −30) leaves only **5.00 mm** between the pivots.

Spreading them has no such limit: at 80° of spread the knee pivots are 168 mm
apart against a 215 mm two-shank span, so the closure stays comfortably
solvable. Hence the asymmetric **80° out / 30° in** range.

Two consequences worth knowing:

- **The default pose is nearly fully extended.** Foot depth is 223.2 mm and the
  mechanism's absolute maximum reach is 227.5 mm — so the leg can essentially
  only retract from the zero pose. The 80° outward range is what buys back a
  real crouch: **223.2 → 141.4 mm**, about **82 mm** of travel. (Under the old
  symmetric ±30° limits it was only ~21 mm.) Splaying the zero pose is the other
  way to get range: the raw CAD pose is ~52° of splay and sits at 176.6 mm.
- **`demo` mode only crouches.** The sweep runs one-sided (0 → +spread) rather
  than symmetric about zero, because there is nothing above the default to
  extend into.

If you want a stance that stands rather than locks out, set:

```xml
<xacro:property name="zero_splay" value="0.5" />   <!-- ~29 deg -->
```

then re-derive `hip_in`, the four knee limits and `foot_overhang`. Everything
else — the baked joint rotations, `stand_height`, `phi_zero` — recomputes itself
from `zero_splay`.

> **Keep `ZERO_SPLAY` in `five_bar_state_publisher.py` equal to `zero_splay` in
> the xacro.** The xacro bakes the matching rotations into the joint origins;
> the node has to measure its angles from the same reference or the published
> knee values will be offset and the linkage will render open.
>
> The same applies to `HIP_OUT` / `HIP_IN` in the node and `hip_out` / `hip_in`
> in the xacro — the node clamps to its own copies, so if they disagree the
> sliders will either stop short of the URDF limit or command past it.

### Hip sign convention

Hip travel is **directional**, and the asymmetry is **rear vs front**, not left
vs right — both legs use `axis="0 0 1"` with joint origins differing by under
0.01 rad, so identical signs produce identical motion on each leg:

| motion | `*_hip_rear` | `*_hip_front` | effect |
|---|---|---|---|
| **outward** (spread, 80°) | **negative** | **positive** | crouch — foot rises 223 → 141 mm |
| **inward** (close, 30°) | **positive** | **negative** | slight extension, femurs converge |

Inward is capped at 30° because the two femurs are coplanar and their knee
pivots collide at 33.06°; the worst corner (rear +30, front −30) leaves 5.00 mm.
Outward is limited only by taste — the closure stays solvable well past 80°.

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

| File | DOF/leg | Loop closed by | Use it for |
| :--- | :--- | :--- | :--- |
| `assembly.xacro` | 2 | *(nothing — source)* | **source of truth** — edit this |
| `assembly.urdf` | 2 | solver node / Gazebo SDF | ROS: RViz, Gazebo, robot_state_publisher |
| `assembly_view.urdf` | **1** | **built-in `<mimic>` chain** | **standalone viewers** (VS Code URDF Visualizer, Foxglove, web) — [details](#standalone-viewer-mode-mimic-coupled-1-dof) |

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

> ⚠️ **This wipes the `<mimic>` chain.** The coupling that makes
> `assembly_view.urdf` render closed without ROS is hand-applied and is not in
> the xacro, so regenerating produces a 2-DOF file whose tibias dangle in a plain
> viewer. Re-apply the four joint blocks per leg afterwards — see
> [Standalone viewer mode](#standalone-viewer-mode-mimic-coupled-1-dof).

---

## Regenerating the plain URDF

`assembly.xacro` is the source of truth. After editing it:

```bash
python3 urdf/generate_urdf.py
```

This runs the real xacro parser (the old version of this script used regexes and
silently mangled anything it did not anticipate) and rewrites mesh paths to
`package://`.
