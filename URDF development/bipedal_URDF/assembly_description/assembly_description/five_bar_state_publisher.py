#!/usr/bin/env python3
"""Publish closure-consistent joint states for the 5-bar parallel legs.

Each leg is a closed 5-bar loop, so its two knees are not free: once the two
hip angles are chosen, the knee angles follow from the loop-closure equation.
URDF cannot represent that (a tree has no cycles), so if you drive all eight
joints independently - which is what plain joint_state_publisher_gui does -
the linkage visibly comes apart at the feet.

This node takes only the four HIP angles, solves each leg's closure in closed
form, and publishes all eight joint values. The result is a linkage that stays
joined at the foot for every pose.

Modes (parameter ``mode``):
  gui   - read hip angles from ``hip_states`` (drive it with
          joint_state_publisher_gui remapped onto that topic); default
  demo   - sweep the legs through a smooth crouch/extend cycle
  hold   - hold the pose given by the ``initial_hips`` parameter
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

# --- Mechanism geometry, in the CAD frame, measured off the exported meshes ---
# Femur: hip pivot -> knee pivot.  Shank: knee pivot -> foot pivot.
FEMUR_LEN = 0.055
SHANK_LEN = 0.1075

HIP_REAR = (0.065, -0.005)
HIP_FRONT = (0.065, 0.055)
HIP_SPACING = HIP_FRONT[1] - HIP_REAR[1]

# Splay of each femur from straight-down at the zero pose. MUST match
# zero_splay in assembly.xacro, which bakes the matching rotations into the
# joint origins. 0.0 means both femurs hang vertically.
ZERO_SPLAY = 0.0

# Hip limits, matching hip_out / hip_in in assembly.xacro. DIRECTIONAL: the
# travel is not symmetric about the zero pose, and the sign convention is a
# REAR/FRONT distinction rather than a LEFT/RIGHT one - both legs are identical
# in joint space, so the same signs mean the same motion on each.
#
#     spread (outward, crouch) : hip_rear NEGATIVE, hip_front POSITIVE
#     close  (inward)          : hip_rear POSITIVE, hip_front NEGATIVE
#
# Outward is free (the closure stays solvable well past 80 deg). Inward is
# bounded by the two femurs of a leg colliding, not by the closure: their knee
# pivots meet when 2*FEMUR_LEN*sin(range) reaches
# HIP_SPACING + 2*FEMUR_LEN*sin(ZERO_SPLAY), i.e. at 33.06 deg for zero splay.
HIP_OUT = 1.396263  # 80 deg, femurs spreading
HIP_IN = 0.523599   # 30 deg, femurs closing

# (lower, upper) per joint, so a slider cannot ask for a pose the URDF forbids.
HIP_BOUNDS = {
    'left_hip_rear': (-HIP_OUT, HIP_IN),
    'left_hip_front': (-HIP_IN, HIP_OUT),
    'right_hip_rear': (-HIP_OUT, HIP_IN),
    'right_hip_front': (-HIP_IN, HIP_OUT),
}

HIP_JOINTS = ['left_hip_rear', 'left_hip_front', 'right_hip_rear', 'right_hip_front']
KNEE_JOINTS = ['left_knee_rear', 'left_knee_front', 'right_knee_rear', 'right_knee_front']

LEGS = {'left': {}, 'right': {}}


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def _clamp(joint, q):
    lo, hi = HIP_BOUNDS[joint]
    return max(lo, min(hi, q))


class LegModel:
    """Closed-form kinematics for one 5-bar leg.

    Both legs share this model. The URDF re-zeros every leg joint onto the
    symmetric splay pose, so in joint space the legs are identical even though
    their raw CAD poses differ by about half a degree.
    """

    def __init__(self, geom=None):
        # femur angles at q = 0, measured from CAD +X (straight down)
        self.theta_rear_0 = -ZERO_SPLAY
        self.theta_front_0 = ZERO_SPLAY
        # the closure at that pose is symmetric about the hip midline
        gap = HIP_SPACING + 2.0 * FEMUR_LEN * math.sin(ZERO_SPLAY)
        reach = math.sqrt(SHANK_LEN ** 2 - (0.5 * gap) ** 2)
        self.phi_rear_0 = math.atan2(0.5 * HIP_SPACING + FEMUR_LEN * math.sin(ZERO_SPLAY),
                                     reach)
        self.phi_front_0 = -self.phi_rear_0

    def solve(self, q_hip_rear, q_hip_front):
        """Hip joint angles -> (knee_rear, knee_front) joint angles.

        Returns None when the two knees drift further apart than the shanks can
        span, which cannot happen inside the URDF's hip limits but is checked
        anyway so a bad command degrades to "hold last pose" instead of NaN.
        """
        theta_rear = self.theta_rear_0 + q_hip_rear
        theta_front = self.theta_front_0 + q_hip_front

        kr = (HIP_REAR[0] + FEMUR_LEN * math.cos(theta_rear),
              HIP_REAR[1] + FEMUR_LEN * math.sin(theta_rear))
        kf = (HIP_FRONT[0] + FEMUR_LEN * math.cos(theta_front),
              HIP_FRONT[1] + FEMUR_LEN * math.sin(theta_front))

        dx, dy = kf[0] - kr[0], kf[1] - kr[1]
        d = math.hypot(dx, dy)
        if d < 1e-9 or d > 2.0 * SHANK_LEN:
            return None

        half = 0.5 * d
        h_sq = SHANK_LEN * SHANK_LEN - half * half
        if h_sq < 0.0:
            return None
        h = math.sqrt(h_sq)

        mx, my = 0.5 * (kr[0] + kf[0]), 0.5 * (kr[1] + kf[1])
        ux, uy = dx / d, dy / d

        # Two mirror solutions about the knee-to-knee line. Pick the one on a
        # FIXED side of that line - the assembly mode the robot is built in.
        #
        # Do not be tempted to pick "whichever foot is further along +X"
        # instead: the two candidates differ by 2*h*uy in X, so that rule
        # silently swaps branches the moment the knees cross over in Y, which
        # is reachable once the zero pose is unsplayed. Choosing by the normal
        # is continuous everywhere except the stretched singularity, which the
        # mechanism cannot reach.
        foot = (mx + h * uy, my - h * ux)

        phi_rear = math.atan2(foot[1] - kr[1], foot[0] - kr[0])
        phi_front = math.atan2(foot[1] - kf[1], foot[0] - kf[0])

        # Every joint in the chain turns about +Z, and the fixed rotations the
        # URDF bakes into the origins are already accounted for by measuring
        # phi/theta from the zero pose. So a link's absolute angle is the sum
        # of the joint angles above it: knee = (shank swing) - (hip swing).
        return (_wrap((phi_rear - self.phi_rear_0) - q_hip_rear),
                _wrap((phi_front - self.phi_front_0) - q_hip_front))


class FiveBarStatePublisher(Node):

    def __init__(self):
        super().__init__('five_bar_state_publisher')

        self.declare_parameter('mode', 'gui')
        self.declare_parameter('rate', 50.0)
        self.declare_parameter('initial_hips', [0.0, 0.0, 0.0, 0.0])
        # 1.0 rad of spread ~= 57 deg, which crouches the foot from 223 mm to
        # about 170 mm below the base origin. Capped at HIP_OUT (80 deg).
        self.declare_parameter('demo_amplitude', 1.0)
        self.declare_parameter('demo_period', 6.0)

        self.mode = self.get_parameter('mode').value
        rate = float(self.get_parameter('rate').value)

        hips = list(self.get_parameter('initial_hips').value)
        if len(hips) != 4:
            self.get_logger().warn(
                f'initial_hips needs 4 values, got {len(hips)}; using zeros')
            hips = [0.0] * 4
        self.hips = dict(zip(HIP_JOINTS, hips))

        self.legs = {side: LegModel() for side in LEGS}
        self.last_knees = {side: (0.0, 0.0) for side in LEGS}

        self.pub = self.create_publisher(JointState, 'joint_states', 10)

        if self.mode == 'gui':
            self.create_subscription(JointState, 'hip_states', self._on_hip_states, 10)
            self.get_logger().info(
                'mode=gui: reading hip angles from "hip_states" '
                '(knee sliders there are ignored - the closure sets them)')
        elif self.mode == 'demo':
            self.get_logger().info('mode=demo: sweeping the legs')
        elif self.mode == 'hold':
            self.get_logger().info(f'mode=hold: {self.hips}')
        else:
            self.get_logger().warn(f'unknown mode "{self.mode}", falling back to hold')
            self.mode = 'hold'

        self.start = self.get_clock().now()
        self.create_timer(1.0 / rate, self._tick)

    def _on_hip_states(self, msg):
        for name, pos in zip(msg.name, msg.position):
            if name in self.hips:
                self.hips[name] = pos

    def _demo_hips(self, t):
        amp = min(float(self.get_parameter('demo_amplitude').value), HIP_OUT)
        period = max(0.1, float(self.get_parameter('demo_period').value))
        w = 2.0 * math.pi * t / period

        # Spreading the femurs retracts the leg, closing them extends it. The
        # sweep is deliberately ONE-SIDED (0 .. +amp of spread) rather than
        # symmetric about zero: with zero_splay = 0 the default pose is already
        # within 4 mm of full extension, so there is nothing to gain by driving
        # the femurs together - it only walks them towards each other, and they
        # are coplanar. So we only ever crouch from the default.
        crouch = amp * 0.5 * (1.0 - math.cos(w))
        crouch_other = amp * 0.5 * (1.0 - math.cos(w + math.pi))
        return {
            'left_hip_rear': -crouch,
            'left_hip_front': crouch,
            'right_hip_rear': -crouch_other,
            'right_hip_front': crouch_other,
        }

    def _tick(self):
        if self.mode == 'demo':
            t = (self.get_clock().now() - self.start).nanoseconds * 1e-9
            self.hips = self._demo_hips(t)

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        for side in ('left', 'right'):
            q_rear = _clamp(f'{side}_hip_rear', self.hips[f'{side}_hip_rear'])
            q_front = _clamp(f'{side}_hip_front', self.hips[f'{side}_hip_front'])
            knees = self.legs[side].solve(q_rear, q_front)
            if knees is None:
                # unreachable command: keep the last valid knee pair rather
                # than publishing a broken pose
                knees = self.last_knees[side]
                self.get_logger().warn(
                    f'{side} leg: hips ({q_rear:.3f}, {q_front:.3f}) have no '
                    'closure solution; holding previous knee angles',
                    throttle_duration_sec=2.0)
            else:
                self.last_knees[side] = knees

            msg.name += [f'{side}_hip_rear', f'{side}_hip_front',
                         f'{side}_knee_rear', f'{side}_knee_front']
            msg.position += [q_rear, q_front, knees[0], knees[1]]

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FiveBarStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
