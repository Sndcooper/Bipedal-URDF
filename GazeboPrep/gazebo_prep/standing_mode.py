#!/usr/bin/env python3
"""Leg posture controller for the 5-bar parallel linkages.

Reproduces the physical robot's start-up sequence: the robot rests fallen, and
when the motors are armed the legs drive to the calibrated stance while the
balance controller catches the body and brings it upright.

Only the two driver joints are commanded. The front hips and all four knees
follow through the URDF ``<mimic>`` tags, so publishing two values moves the
whole linkage.

Topics
------
subscribes  /balance/armed              std_msgs/Bool     (transient local)
subscribes  /joint_states               sensor_msgs/JointState
publishes   /hip_position_controller/commands
                                        std_msgs/Float64MultiArray  [L, R]
publishes   /standing_mode/status       std_msgs/String

Services
--------
/standing_mode/set_stance   std_srvs/Trigger
    Re-run the stance transition from the current pose. Useful for retrying a
    stand-up without a full disarm/arm cycle.
"""

import math

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray, String
from std_srvs.srv import Trigger

# Driver joints, in the order declared by hip_position_controller. Reordering
# these silently swaps the legs.
DRIVER_JOINTS = ('left_hip_rear', 'right_hip_rear')

# Position limits from the URDF. Commands are clamped to these so a bad
# parameter cannot drive the linkage into its hard stops.
HIP_LOWER = -1.396263
HIP_UPPER = 0.087266


class StandingMode(Node):
    """Interpolates the leg drivers between the fallen and stance postures."""

    def __init__(self):
        super().__init__('standing_mode')

        self.declare_parameter('stance_angle', -0.60)
        self.declare_parameter('fallen_angle', -0.30)
        self.declare_parameter('stand_duration', 1.5)
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('relax_on_disarm', False)

        self.stance_angle = self._clamp_hip(
            self.get_parameter('stance_angle').value)
        self.fallen_angle = self._clamp_hip(
            self.get_parameter('fallen_angle').value)
        self.stand_duration = float(self.get_parameter('stand_duration').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.relax_on_disarm = bool(self.get_parameter('relax_on_disarm').value)

        # Trajectory state
        self._traj_active = False
        self._traj_start_time = None
        self._traj_from = [self.fallen_angle, self.fallen_angle]
        self._traj_to = [self.stance_angle, self.stance_angle]
        self._last_command = [self.fallen_angle, self.fallen_angle]

        # Feedback
        self._joint_positions = {}
        self._armed = False
        self._joints_ready = False

        self.cmd_pub = self.create_publisher(
            Float64MultiArray, '/hip_position_controller/commands', 10)
        self.status_pub = self.create_publisher(
            String, '/standing_mode/status', 10)

        # The arm state is latched by balance_point, so a late-starting
        # standing_mode still receives the current value rather than waiting
        # for the next transition.
        armed_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(Bool, '/balance/armed', self._on_armed, armed_qos)
        self.create_subscription(
            JointState, '/joint_states', self._on_joint_states, 10)

        self.create_service(
            Trigger, '/standing_mode/set_stance', self._on_set_stance)

        self.add_on_set_parameters_callback(self._on_parameter_update)

        self.timer = self.create_timer(
            1.0 / self.publish_rate, self._publish_step)

        self.get_logger().info(
            'standing_mode ready | stance=%.3f rad fallen=%.3f rad '
            'transition=%.2f s' % (
                self.stance_angle, self.fallen_angle, self.stand_duration))

    # ------------------------------------------------------------------
    # Parameters
    # ------------------------------------------------------------------

    @staticmethod
    def _clamp_hip(value):
        return max(HIP_LOWER, min(float(value), HIP_UPPER))

    def _on_parameter_update(self, params):
        for p in params:
            if p.name == 'stance_angle':
                self.stance_angle = self._clamp_hip(p.value)
            elif p.name == 'fallen_angle':
                self.fallen_angle = self._clamp_hip(p.value)
            elif p.name == 'stand_duration':
                if p.value <= 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason='stand_duration must be positive')
                self.stand_duration = float(p.value)
            elif p.name == 'relax_on_disarm':
                self.relax_on_disarm = bool(p.value)
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _on_joint_states(self, msg):
        for name, position in zip(msg.name, msg.position):
            self._joint_positions[name] = position
        if all(j in self._joint_positions for j in DRIVER_JOINTS):
            self._joints_ready = True

    def _on_armed(self, msg):
        if msg.data == self._armed:
            return
        self._armed = msg.data

        if self._armed:
            self._begin_transition(self.stance_angle, 'standing')
        elif self.relax_on_disarm:
            self._begin_transition(self.fallen_angle, 'relaxing')
        else:
            # Hold the current pose. Leaving the legs put keeps consecutive
            # A/B tuning runs starting from an identical posture.
            self._traj_active = False
            self._publish_status('holding')

    def _on_set_stance(self, request, response):
        del request
        self._begin_transition(self.stance_angle, 'standing')
        response.success = True
        response.message = 'Stance transition started to %.3f rad' % self.stance_angle
        return response

    # ------------------------------------------------------------------
    # Trajectory
    # ------------------------------------------------------------------

    def _begin_transition(self, target, label):
        """Start a smooth move from the measured pose to ``target``."""
        # Starting from the MEASURED position, never from an assumed one, is
        # what stops the position controller stepping and kicking the robot.
        if self._joints_ready:
            start = [self._joint_positions[j] for j in DRIVER_JOINTS]
        else:
            start = list(self._last_command)

        self._traj_from = start
        self._traj_to = [target, target]
        self._traj_start_time = self.get_clock().now()
        self._traj_active = True
        self._publish_status(label)

        self.get_logger().info(
            '%s: [%.3f, %.3f] -> %.3f rad over %.2f s' % (
                label.capitalize(), start[0], start[1], target,
                self.stand_duration))

    def _publish_step(self):
        if self._traj_active:
            elapsed = (self.get_clock().now() - self._traj_start_time).nanoseconds / 1e9
            fraction = min(elapsed / self.stand_duration, 1.0)

            # Cosine (smoothstep) blend: zero velocity at both ends, so the leg
            # inertia does not jolt the body at the start or overshoot at the
            # end. A linear ramp would step the velocity and can tip the robot
            # before the balance loop has authority.
            blend = 0.5 * (1.0 - math.cos(math.pi * fraction))

            command = [
                self._traj_from[i] + (self._traj_to[i] - self._traj_from[i]) * blend
                for i in range(2)
            ]

            if fraction >= 1.0:
                self._traj_active = False
                self._publish_status('holding')
        else:
            command = self._last_command

        command = [self._clamp_hip(c) for c in command]
        self._last_command = command

        msg = Float64MultiArray()
        msg.data = command
        self.cmd_pub.publish(msg)

    def _publish_status(self, status):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = StandingMode()
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
