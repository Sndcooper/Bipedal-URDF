# gazebo_prep

Gazebo Harmonic simulation of the STM32-based self-balancing wheeled bipedal
robot, for **comparing cascaded PID gain sets** — how oscillations differ and
how stability changes as values change — without risking the hardware.

The simulated control stack is a deliberate port of the firmware in
`RelevantPhysicalRobotFiles/RC_mcu_IK_wireless`, so gains transfer in both
directions. The robot spawns lying on its back and stands up when armed, as it
does on the bench.

---

## Build and run

```bash
# 1. Copy into a workspace (the ROS package name is gazebo_prep, lowercase)
mkdir -p ~/ros2_ws/src
cp -r /path/to/GazeboPrep ~/ros2_ws/src/gazebo_prep

# 2. Strip CR characters if the copy came over Windows filesharing.
#    The shipped files are already LF; this is belt and braces.
sudo apt install -y dos2unix
find ~/ros2_ws/src/gazebo_prep -type f ! -name '*.stl' -exec dos2unix -q {} +

# 3. Dependencies
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
rosdep install --from-paths src -yi

# 4. Build
colcon build --symlink-install --packages-select gazebo_prep
source install/setup.bash

# 5. Run
ros2 launch gazebo_prep simulation.launch.py
```

The robot appears lying on its back and stays there. Arm it:

```bash
ros2 service call /motors_enabled std_srvs/srv/SetBool "{data: true}"
```

The legs drive to the stance angle and the balance loop takes over.

### Driving

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1}, angular: {z: 0.3}}"
```

Top speed is **0.125 m/s**, set by the firmware's `max_vel_cmd` of 800
encoder counts/s (`800 / 1320 × 2π × 0.033`). Requesting more just saturates,
which looks like an unresponsive robot rather than a limit. To go faster,
raise `max_vel_cmd` and `rc_teleop`'s `max_speed` together.

### Live tuning

Either through ROS parameters:

```bash
ros2 param set /balance_point kp 110.0
ros2 service call /reset_state std_srvs/srv/Trigger   # flush integrators
```

or through the firmware's own ASCII protocol, so the existing wireless GUI
works unchanged:

```bash
nc localhost 3333
P110.0|          # set Kp          M|    toggle motors
I670.0|          # set Ki          C|    calibrate IMU
D2.0|            # set Kd          R|    reset integrators
VP0.02|          # velocity-loop Kp      S0.5|  base target angle
```

### Useful launch arguments

```bash
ros2 launch gazebo_prep simulation.launch.py gui:=false          # headless
ros2 launch gazebo_prep simulation.launch.py spawn_pitch:=-0.8   # start reclined
ros2 launch gazebo_prep simulation.launch.py use_closure_joints:=true
```

---

## Verify before trusting any tuning result

Run these in order. Steps 1–3 catch the failures that look like control
problems but are not.

**1. The model parses.**

```bash
xacro install/gazebo_prep/share/gazebo_prep/urdf/assembly.urdf.xacro > /tmp/a.urdf
check_urdf /tmp/a.urdf
```

Expect 15 links and 14 joints, root `base_footprint`, no mimic errors.

**2. Controllers are up.**

```bash
ros2 control list_controllers
```

`joint_state_broadcaster`, `hip_position_controller` and
`wheel_effort_controller` active; `wheel_velocity_controller` inactive.

**3. The IMU is publishing and oriented correctly.**

```bash
ros2 topic echo /imu --once
```

Lying on its back, `linear_acceleration.x` should read about **−9.8** (gravity
toward the robot's rear). If it reads near zero on x and −9.8 on z, the robot
is upright, not fallen.

**4. The sign convention — do this before any gain work.**

Arm the robot and watch the recovery. The wheels must drive **under** the
falling body. If it accelerates in the direction it is falling, flip the sign:

```bash
ros2 param set /balance_point effort_sign 1.0
```

**5. Log a run.**

```bash
ros2 topic echo /balance/telemetry --csv > run1.csv
```

Telemetry publishes at the full 100 Hz control rate, so oscillations are
captured without aliasing.

---

## Topics, services and parameters

### Topics

| Topic | Type | Direction | Notes |
|---|---|---|---|
| `/imu` | `sensor_msgs/Imu` | in | bridged from Gazebo, 200 Hz |
| `/joint_states` | `sensor_msgs/JointState` | in | all 10 joints |
| `/cmd_vel` | `geometry_msgs/Twist` | in | m/s, rad/s |
| `/balance/cmd_vel` | `geometry_msgs/Twist` | internal | `angular.z` carries PWM turn bias, not rad/s |
| `/wheel_effort_controller/commands` | `Float64MultiArray` | out | `[left, right]` N·m |
| `/hip_position_controller/commands` | `Float64MultiArray` | out | `[left_hip_rear, right_hip_rear]` rad |
| `/balance/telemetry` | `Float64MultiArray` | out | 16 fields, 100 Hz — see below |
| `/balance/armed` | `std_msgs/Bool` | out | latched |
| `/standing_mode/status` | `std_msgs/String` | out | `standing` / `holding` / `relaxing` |

### Telemetry field order

```
 0 pitch        4 integral     8  vel_error    12 safety_latched
 1 target       5 tilt_bias    9  pos_error    13 left_pwm
 2 error        6 gyro_rate    10 cascade_state 14 right_pwm
 3 pid_output   7 vel_current  11 motors_enabled 15 dt
```

`cascade_state`: 0 driving, 1 rampdown, 2 holding.

### Services

| Service | Type | Purpose |
|---|---|---|
| `/motors_enabled` | `SetBool` | arm/disarm; clears the safety latch |
| `/calibrate` | `Trigger` | zero the pitch offset (firmware `C`) |
| `/reset_state` | `Trigger` | flush integrators — **use between gain changes** |
| `/standing_mode/set_stance` | `Trigger` | re-run the stand-up |

### Key parameters

All in `config/balance_params.yaml`, all settable live.

| Parameter | Default | Notes |
|---|---|---|
| `kp` / `ki` / `kd` | 95.0 / 670.0 / 2.0 | hardware-tuned |
| `alpha` | 0.96 | complementary filter |
| `max_safe_tilt` | 25.0 | safety latch, degrees |
| `kp_vel` / `ki_vel` | 0.02 / 0.001 | middle loop |
| `kp_pos` | 0.5 | outer loop |
| `counts_per_rev` | 1320.0 | **measure this** — see below |
| `stall_torque` | 0.15 | N·m per wheel |
| `effort_sign` | −1.0 | sign escape hatch |

---

## Things that will bite you

**Wheel torque is the binding constraint.** At 0.15 N·m per wheel, total
authority is 0.30 N·m. Against a first-order pendulum model of this robot
(1.83 kg, COM ≈ 0.15 m up) gravity torque reaches 0.30 N·m at only about
**6.4°** of lean. The robot can hold itself near upright but may not be able to
stand up from fully flat. If the stand-up fails, that is a torque limit, not a
tuning failure — start it partially reclined:

```bash
ros2 launch gazebo_prep simulation.launch.py spawn_pitch:=-0.8
```

**`Ki = 670` may be unstable in simulation even though it works on hardware.**
The firmware's inner integrator has **no clamp** — only the safety latch and
the kill switch reset it. In a closed-loop check of this port, `Ki` above
roughly 100 wound up during the first oscillation and saturated the output,
while `Ki = 0` and `Kd ≥ 10` stayed stable. The real robot differs from that
crude model in COM height, wheel inertia and ground compliance, so treat this
as a prompt to watch field 4 (`integral`) in telemetry rather than as a
prediction. Gains ship at your hardware-tuned values and were not altered.

**Always `/reset_state` after changing a gain mid-run.** A loaded integrator
produces a transient that looks exactly like instability caused by the new
gain.

**`counts_per_rev` is not documented anywhere in the hardware repo.** The
middle and outer loops are expressed in encoder counts, so a wrong value
silently re-gains both. To measure: turn one wheel exactly one revolution and
read the count delta. The firmware counts RISING edges only, so it is
`pulses_per_motor_rev × gear_ratio`, not ×4.

**Do not enable `use_closure_joints` together with the mimic joints.** The
knee ratio −1.4774 is a minimax linear fit to a nonlinear closure (up to
6.48 mm of foot gap at −80°), so the two constraints would demand different
poses and fight each other. See the header of `urdf/assembly_gazebo.xacro`.

---

## Design notes

**Mimic retargeting.** In the source URDF, `<side>_knee_front` mimics
`<side>_hip_front`, which is itself a mimic of `<side>_hip_rear`.
ros2_control enforces a mimic by mirroring the mimicked joint's *command
interface*, and `hip_front` has none, so a chained mimic fails validation or
silently freezes. Both front knees were retargeted to their leg's driver with
multiplier `+1.477400`, which is mathematically identical since
`hip_front = −1.0 × hip_rear`.

**Wheel axes are pre-mirrored** (`0 0 -1` left, `0 0 1` right), so a positive
joint value drives both wheels forward and the firmware's mixing ports over
unchanged.

**IMU pitch uses `atan2(ax, az)`, not the firmware's `atan2(-ax, ...)`.** The
`imu_link` frame is REP-103 aligned — the composite of the CAD→ROS rotation and
the `imu_joint` rotation is exactly the identity, verified numerically. The
firmware's negation compensates for its physical MPU6050 mounting; carrying it
over here would invert the feedback sign and turn the balance loop into
positive feedback.

**The Gazebo IMU orientation quaternion is deliberately unused.** gz-sensors
references it to the sensor's *initial* pose, and the robot spawns fallen, so
it would read near-zero pitch while lying down. The complementary filter runs
on raw accelerometer and gyro data instead.

**Actuator authority.** Four AX-12+ servos, one per femur, 1.5 N·m each. The
URDF exposes two independent DOF because the 5-bar closure determines the front
hip from the rear, and mimic joints contribute no torque in gz_ros2_control, so
each driver joint carries the combined 3.0 N·m of its leg's two servos.

**Total model mass is 1.891 kg**, matching the ~1.831 kg figure for the
physical robot to within 3.3%. Masses and inertia tensors are copied verbatim
from the CAD-derived `assembly_view.urdf`.

---

## Layout

```
urdf/assembly.urdf.xacro           robot model (ported, mimics retargeted)
urdf/assembly_ros2_control.xacro   hardware interface declaration
urdf/assembly_gazebo.xacro         plugin, IMU sensor, friction, closures
config/ros2_controllers.yaml       controller definitions
config/balance_params.yaml         all gains and runtime parameters
worlds/balance_arena.sdf           flat test world, 1 ms physics step
launch/simulation.launch.py        full bring-up
src/balance_point.cpp              three-layer cascade (C++)
gazebo_prep/standing_mode.py       5-bar posture control
gazebo_prep/rc_teleop.py           /cmd_vel mapping
gazebo_prep/wireless_gui_bridge.py firmware ASCII protocol over TCP
```
