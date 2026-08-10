#!/usr/bin/env python3
"""Teleop mapper: ``/cmd_vel`` to the balance controller's targets.

This node is a pure mapper and holds no control state. All three cascade
layers live in the ``balance_point`` C++ node, exactly as they live together
on the STM32, so that changing a gain affects one place and the simulated
control path matches the firmware's structure.

What it reproduces from the FlySky RC handling:

* the stick deadband (the transmitter's 1425-1575 window on a 1000-2000 range)
* the channel-4 steering scale onto a +-425 PWM turn bias
* a slew limit, because a physical stick cannot step instantly and an
  instantaneous command would destabilise the robot for reasons unrelated to
  the gains being studied

Topics
------
subscribes  /cmd_vel               geometry_msgs/Twist   (m/s, rad/s)
publishes   /balance/cmd_vel       geometry_msgs/Twist

On the published message, ``linear.x`` stays in m/s (balance_point converts to
encoder counts) and ``angular.z`` carries the already-scaled PWM turn bias
rather than rad/s. Keeping it a plain Twist avoids a custom message for two
floats; the unit change is documented here and in balance_point's onCmdVel.
"""

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node

from geometry_msgs.msg import Twist


class RCTeleop(Node):
    """Scales, deadbands and rate-limits velocity commands."""

    def __init__(self):
        super().__init__('rc_teleop')

        self.declare_parameter('max_speed', 0.5)
        self.declare_parameter('max_turn', 2.0)
        self.declare_parameter('turn_gain', 212.5)
        self.declare_parameter('deadband', 0.075)
        self.declare_parameter('accel_limit', 1.0)
        self.declare_parameter('turn_slew_limit', 4.0)
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('cmd_timeout', 0.5)

        self.max_speed = float(self.get_parameter('max_speed').value)
        self.max_turn = float(self.get_parameter('max_turn').value)
        self.turn_gain = float(self.get_parameter('turn_gain').value)
        self.deadband = float(self.get_parameter('deadband').value)
        self.accel_limit = float(self.get_parameter('accel_limit').value)
        self.turn_slew_limit = float(self.get_parameter('turn_slew_limit').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)

        # Raw request from /cmd_vel
        self._req_linear = 0.0
        self._req_angular = 0.0
        self._last_cmd_time = None

        # Slew-limited output
        self._out_linear = 0.0
        self._out_angular = 0.0

        self.cmd_pub = self.create_publisher(Twist, '/balance/cmd_vel', 10)
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd_vel, 10)

        self.add_on_set_parameters_callback(self._on_parameter_update)

        self.timer = self.create_timer(
            1.0 / self.publish_rate, self._publish_step)

        self.get_logger().info(
            'rc_teleop ready | max_speed=%.2f m/s max_turn=%.2f rad/s '
            'turn_gain=%.1f deadband=%.1f%%' % (
                self.max_speed, self.max_turn, self.turn_gain,
                self.deadband * 100.0))

    # ------------------------------------------------------------------
    # Parameters
    # ------------------------------------------------------------------

    def _on_parameter_update(self, params):
        for p in params:
            if p.name == 'max_speed':
                if p.value <= 0.0:
                    return SetParametersResult(
                        successful=False, reason='max_speed must be positive')
                self.max_speed = float(p.value)
            elif p.name == 'max_turn':
                if p.value <= 0.0:
                    return SetParametersResult(
                        successful=False, reason='max_turn must be positive')
                self.max_turn = float(p.value)
            elif p.name == 'turn_gain':
                self.turn_gain = float(p.value)
            elif p.name == 'deadband':
                self.deadband = max(0.0, min(float(p.value), 0.9))
            elif p.name == 'accel_limit':
                self.accel_limit = float(p.value)
            elif p.name == 'turn_slew_limit':
                self.turn_slew_limit = float(p.value)
            elif p.name == 'cmd_timeout':
                self.cmd_timeout = float(p.value)
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------------
    # Mapping
    # ------------------------------------------------------------------

    def _apply_deadband(self, value, limit):
        """Zero small inputs and rescale the remainder to the full range.

        Without the rescale, leaving the deadband would produce a step in the
        output. The firmware does the same thing with its 1425/1575 window.
        """
        if limit <= 0.0:
            return 0.0

        normalised = max(-1.0, min(value / limit, 1.0))

        if abs(normalised) < self.deadband:
            return 0.0

        span = 1.0 - self.deadband
        if span <= 1e-9:
            return 0.0

        sign = 1.0 if normalised > 0.0 else -1.0
        scaled = (abs(normalised) - self.deadband) / span
        return sign * scaled * limit

    @staticmethod
    def _slew(current, target, max_delta):
        if max_delta <= 0.0:
            return target
        delta = target - current
        if delta > max_delta:
            return current + max_delta
        if delta < -max_delta:
            return current - max_delta
        return target

    def _on_cmd_vel(self, msg):
        self._req_linear = max(-self.max_speed,
                               min(msg.linear.x, self.max_speed))
        self._req_angular = max(-self.max_turn,
                                min(msg.angular.z, self.max_turn))
        self._last_cmd_time = self.get_clock().now()

    def _publish_step(self):
        dt = 1.0 / self.publish_rate

        # Fail safe to zero if commands stop arriving, so a dead publisher
        # leaves the robot balancing in place rather than driving away.
        target_linear = self._req_linear
        target_angular = self._req_angular

        if self._last_cmd_time is not None and self.cmd_timeout > 0.0:
            age = (self.get_clock().now() - self._last_cmd_time).nanoseconds / 1e9
            if age > self.cmd_timeout:
                target_linear = 0.0
                target_angular = 0.0

        target_linear = self._apply_deadband(target_linear, self.max_speed)
        target_angular = self._apply_deadband(target_angular, self.max_turn)

        self._out_linear = self._slew(
            self._out_linear, target_linear, self.accel_limit * dt)
        self._out_angular = self._slew(
            self._out_angular, target_angular, self.turn_slew_limit * dt)

        out = Twist()
        out.linear.x = self._out_linear
        # rad/s -> PWM turn bias, matching the transmitter's +-425 span.
        out.angular.z = self._out_angular * self.turn_gain
        self.cmd_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = RCTeleop()
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
