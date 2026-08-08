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

# Knee and foot pivots at the CAD zero pose, per leg. The two legs are related
# by a 180 deg rotation about the CAD X axis rather than a mirror, so their
# zero-pose angles differ slightly; both are kept exactly as built.
LEGS = {
    'left': {
        'knee_rear': (0.092496, -0.052634),
        'knee_front': (0.103696, 0.094085),
        'foot': (0.176248, 0.014760),
    },
    'right': {
        'knee_rear': (0.092504, -0.052629),
        'knee_front': (0.104057, 0.093724),
        'foot': (0.176574, 0.014366),
    },
}

HIP_JOINTS = ['left_hip_rear', 'left_hip_front', 'right_hip_rear', 'right_hip_front']
KNEE_JOINTS = ['left_knee_rear', 'left_knee_front', 'right_knee_rear', 'right_knee_front']

HIP_LIMIT = 1.047198  # +-60 deg; matches the URDF


def _angle(a, b):
    return math.atan2(b[1] - a[1], b[0] - a[0])


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class LegModel:
    """Closed-form kinematics for one 5-bar leg."""

    def __init__(self, geom):
        self.theta_rear_0 = _angle(HIP_REAR, geom['knee_rear'])
        self.theta_front_0 = _angle(HIP_FRONT, geom['knee_front'])
        self.phi_rear_0 = _angle(geom['knee_rear'], geom['foot'])
        self.phi_front_0 = _angle(geom['knee_front'], geom['foot'])

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

        # Two mirror solutions about the knee-to-knee line. The leg always
        # reaches away from the pelvis, i.e. towards +X in the CAD frame; the
        # other branch folds the shanks back up into the body. The branches
        # only meet at the stretched singularity, which this mechanism stays
        # 45 mm clear of, so this choice never flips mid-motion.
        cand_a = (mx + h * uy, my - h * ux)
        cand_b = (mx - h * uy, my + h * ux)
        foot = cand_a if cand_a[0] >= cand_b[0] else cand_b

        phi_rear = math.atan2(foot[1] - kr[1], foot[0] - kr[0])
        phi_front = math.atan2(foot[1] - kf[1], foot[0] - kf[0])

        # Every joint in the chain turns about +Z with no fixed rotation
        # between frames, so a link's absolute angle is just the sum of the
        # joint angles above it. Hence knee = (shank swing) - (hip swing).
        return (_wrap((phi_rear - self.phi_rear_0) - q_hip_rear),
                _wrap((phi_front - self.phi_front_0) - q_hip_front))


class FiveBarStatePublisher(Node):

    def __init__(self):
        super().__init__('five_bar_state_publisher')

        self.declare_parameter('mode', 'gui')
        self.declare_parameter('rate', 50.0)
        self.declare_parameter('initial_hips', [0.0, 0.0, 0.0, 0.0])
        self.declare_parameter('demo_amplitude', 0.45)
        self.declare_parameter('demo_period', 6.0)

        self.mode = self.get_parameter('mode').value
        rate = float(self.get_parameter('rate').value)

        hips = list(self.get_parameter('initial_hips').value)
        if len(hips) != 4:
            self.get_logger().warn(
                f'initial_hips needs 4 values, got {len(hips)}; using zeros')
            hips = [0.0] * 4
        self.hips = dict(zip(HIP_JOINTS, hips))

        self.legs = {side: LegModel(geom) for side, geom in LEGS.items()}
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
        amp = float(self.get_parameter('demo_amplitude').value)
        period = max(0.1, float(self.get_parameter('demo_period').value))
        w = 2.0 * math.pi * t / period
        # Counter-rotating hips squat and extend the leg; the legs run half a
        # cycle apart so the result reads as a slow step.
        squat = amp * math.sin(w)
        squat_other = amp * math.sin(w + math.pi)
        return {
            'left_hip_rear': -squat,
            'left_hip_front': squat,
            'right_hip_rear': -squat_other,
            'right_hip_front': squat_other,
        }

    def _tick(self):
        if self.mode == 'demo':
            t = (self.get_clock().now() - self.start).nanoseconds * 1e-9
            self.hips = self._demo_hips(t)

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        for side in ('left', 'right'):
            q_rear = max(-HIP_LIMIT, min(HIP_LIMIT, self.hips[f'{side}_hip_rear']))
            q_front = max(-HIP_LIMIT, min(HIP_LIMIT, self.hips[f'{side}_hip_front']))
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
