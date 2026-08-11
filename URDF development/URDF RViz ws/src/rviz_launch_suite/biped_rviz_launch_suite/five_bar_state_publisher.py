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
  demo  - sweep the legs through a smooth crouch/extend cycle
  hold  - hold the pose given by the ``initial_hips`` parameter
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
HIP_OUT = 1.396263
HIP_IN = 0.523599

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
    """Closed-form kinematics for one 5-bar leg."""

    def __init__(self, geom=None):
        self.theta_rear_0 = -ZERO_SPLAY
        self.theta_front_0 = ZERO_SPLAY
        gap = HIP_SPACING + 2.0 * FEMUR_LEN * math.sin(ZERO_SPLAY)
        reach = math.sqrt(SHANK_LEN ** 2 - (0.5 * gap) ** 2)
        self.phi_rear_0 = math.atan2(0.5 * HIP_SPACING + FEMUR_LEN * math.sin(ZERO_SPLAY),
                                     reach)
        self.phi_front_0 = -self.phi_rear_0

    def solve(self, q_hip_rear, q_hip_front):
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

        foot = (mx + h * uy, my - h * ux)

        phi_rear = math.atan2(foot[1] - kr[1], foot[0] - kr[0])
        phi_front = math.atan2(foot[1] - kf[1], foot[0] - kf[0])

        return (_wrap((phi_rear - self.phi_rear_0) - q_hip_rear),
                _wrap((phi_front - self.phi_front_0) - q_hip_front))


class FiveBarStatePublisher(Node):

    def __init__(self):
        super().__init__('five_bar_state_publisher')

        self.declare_parameter('mode', 'gui')
        self.declare_parameter('rate', 50.0)
        self.declare_parameter('initial_hips', [0.0, 0.0, 0.0, 0.0])
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