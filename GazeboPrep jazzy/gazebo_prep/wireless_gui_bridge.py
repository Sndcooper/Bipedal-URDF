#!/usr/bin/env python3
"""TCP bridge exposing the STM32 firmware's ASCII tuning protocol.

Replaces the 3DR telemetry radio with a TCP socket so the existing wireless
tuning GUI can drive the simulated robot without modification. Commands and
telemetry use the same wire format as the hardware, including the ``|``
terminator.

Command set (mirrors RC_mcu_IK_wireless/gui/serial_link.py)
-----------------------------------------------------------
    P<val>   inner Kp                VP<val>  velocity-loop Kp
    I<val>   inner Ki                VI<val>  velocity-loop Ki
    D<val>   inner Kd                VA<val>  velocity EMA alpha
    A<val>   complementary alpha     PP<val>  position-loop Kp
    S<val>   base target angle       T<val>   max safe tilt
    O<val>   gimbal/trim offset
    M        toggle motors           C        calibrate IMU
    R        reset integrators

PARSER PRECEDENCE MATTERS. Two-letter prefixes must be tested before their
single-letter counterparts or ``VP12.0`` is read as ``V`` followed by garbage,
or worse as ``P`` with a corrupted value. The firmware orders its checks the
same way; this module keeps that order explicit in COMMAND_TABLE.

Telemetry (20 Hz, matching the radio link the GUI expects)
----------------------------------------------------------
    S<seq> DT<us> P<pitch> O<pid_out> I<integral> V<vel> TB<tilt_bias>
    EP<pos_err> EV<vel_err> VR<vel_alpha> ST<state> A<alpha> T<tilt>
    M<motors> L<latched>|

Threading
---------
Socket I/O runs on background threads; every ROS interaction is marshalled
through a queue drained by a timer on the executor thread. This avoids calling
rclpy from arbitrary threads, which is the usual source of deadlocks in bridge
nodes of this shape.
"""

import queue
import socket
import threading

import rclpy
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from std_srvs.srv import SetBool, Trigger

BALANCE_NODE = 'balance_point'

# Ordered longest-prefix-first. The parser walks this list in order and takes
# the first match, so two-letter commands must precede single-letter ones.
COMMAND_TABLE = (
    ('VP', 'kp_vel'),
    ('VI', 'ki_vel'),
    ('VA', 'vel_alpha'),
    ('PP', 'kp_pos'),
    ('P', 'kp'),
    ('I', 'ki'),
    ('D', 'kd'),
    ('A', 'alpha'),
    ('S', 'base_target_angle'),
    ('T', 'max_safe_tilt'),
)

# Telemetry field indices, matching publishTelemetry() in balance_point.cpp.
T_PITCH = 0
T_TARGET = 1
T_ERROR = 2
T_OUTPUT = 3
T_INTEGRAL = 4
T_TILT_BIAS = 5
T_GYRO = 6
T_VEL = 7
T_VEL_ERR = 8
T_POS_ERR = 9
T_STATE = 10
T_MOTORS = 11
T_LATCHED = 12
T_LEFT_PWM = 13
T_RIGHT_PWM = 14
T_DT = 15


class WirelessGuiBridge(Node):
    """Serves the firmware tuning protocol to GUI clients over TCP."""

    def __init__(self):
        super().__init__('wireless_gui_bridge')

        self.declare_parameter('bind_address', '0.0.0.0')
        self.declare_parameter('port', 3333)
        self.declare_parameter('telemetry_rate', 20.0)
        self.declare_parameter('max_clients', 4)

        self.bind_address = self.get_parameter('bind_address').value
        self.port = int(self.get_parameter('port').value)
        self.telemetry_rate = float(self.get_parameter('telemetry_rate').value)
        self.max_clients = int(self.get_parameter('max_clients').value)

        # Latest telemetry frame and a sequence counter for the GUI.
        self._telemetry = None
        self._seq = 0

        # Motors state is tracked locally so the "M" toggle knows which way to
        # go; balance_point remains the authority via its telemetry field.
        self._motors_enabled = False

        # Mirror of the gains that appear in telemetry but not in the
        # telemetry array. Refreshed periodically and on every successful set,
        # so the GUI's displayed values follow changes made by other clients
        # or by `ros2 param set`.
        self._mirrored = {
            'vel_alpha': 0.85,
            'alpha': 0.96,
            'max_safe_tilt': 25.0,
        }

        # Inbound commands from socket threads, drained on the ROS thread.
        self._command_queue = queue.Queue()

        # Connected clients, guarded by a lock.
        self._clients = []
        self._clients_lock = threading.Lock()
        self._shutdown = threading.Event()

        # --- ROS interfaces ---------------------------------------------
        self.create_subscription(
            Float64MultiArray, '/balance/telemetry', self._on_telemetry, 50)

        self.param_client = self.create_client(
            SetParameters, '/%s/set_parameters' % BALANCE_NODE)
        self.get_param_client = self.create_client(
            GetParameters, '/%s/get_parameters' % BALANCE_NODE)
        self.arm_client = self.create_client(SetBool, '/motors_enabled')
        self.calibrate_client = self.create_client(Trigger, '/calibrate')
        self.reset_client = self.create_client(Trigger, '/reset_state')

        # Drain inbound commands at 50 Hz on the executor thread.
        self.create_timer(0.02, self._drain_commands)

        # Refresh the mirrored gains once a second.
        self.create_timer(1.0, self._refresh_mirrored)

        if self.telemetry_rate > 0.0:
            self.create_timer(1.0 / self.telemetry_rate, self._broadcast_telemetry)

        # --- Socket server ----------------------------------------------
        self._server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server_socket.bind((self.bind_address, self.port))
        self._server_socket.listen(self.max_clients)
        self._server_socket.settimeout(0.5)

        self._accept_thread = threading.Thread(
            target=self._accept_loop, daemon=True)
        self._accept_thread.start()

        self.get_logger().info(
            'wireless_gui_bridge listening on %s:%d | telemetry %.0f Hz | '
            'connect with: nc %s %d' % (
                self.bind_address, self.port, self.telemetry_rate,
                'localhost' if self.bind_address == '0.0.0.0' else self.bind_address,
                self.port))

    # ------------------------------------------------------------------
    # Socket handling (background threads)
    # ------------------------------------------------------------------

    def _accept_loop(self):
        while not self._shutdown.is_set():
            try:
                client, address = self._server_socket.accept()
            except socket.timeout:
                continue
            except OSError:
                break

            with self._clients_lock:
                if len(self._clients) >= self.max_clients:
                    try:
                        client.sendall(b'ERR max clients reached|')
                        client.close()
                    except OSError:
                        pass
                    continue
                self._clients.append(client)

            self.get_logger().info('GUI client connected from %s:%d' % address)
            threading.Thread(
                target=self._client_loop, args=(client, address),
                daemon=True).start()

    def _client_loop(self, client, address):
        client.settimeout(0.5)
        buffer = ''

        while not self._shutdown.is_set():
            try:
                chunk = client.recv(256)
            except socket.timeout:
                continue
            except OSError:
                break

            if not chunk:
                break

            buffer += chunk.decode('utf-8', errors='ignore')

            # Accept '|' (the firmware terminator) as well as newlines, so a
            # human using netcat and the GUI both work.
            for separator in ('|', '\n', '\r'):
                buffer = buffer.replace(separator, '\x00')

            while '\x00' in buffer:
                line, buffer = buffer.split('\x00', 1)
                line = line.strip()
                if line:
                    self._command_queue.put(line.upper())

            # Bound the buffer so a client that never sends a terminator
            # cannot grow it without limit.
            if len(buffer) > 512:
                buffer = ''

        self._remove_client(client)
        self.get_logger().info('GUI client disconnected from %s:%d' % address)

    def _remove_client(self, client):
        with self._clients_lock:
            if client in self._clients:
                self._clients.remove(client)
        try:
            client.close()
        except OSError:
            pass

    def _send_to_all(self, text):
        payload = text.encode('utf-8')
        with self._clients_lock:
            targets = list(self._clients)

        for client in targets:
            try:
                client.sendall(payload)
            except OSError:
                self._remove_client(client)

    # ------------------------------------------------------------------
    # Command handling (ROS thread)
    # ------------------------------------------------------------------

    def _drain_commands(self):
        while True:
            try:
                command = self._command_queue.get_nowait()
            except queue.Empty:
                return
            self._handle_command(command)

    def _handle_command(self, command):
        # Single-letter actions first; they carry no payload.
        if command == 'M':
            self._toggle_motors()
            return
        if command == 'C':
            self._call_trigger(self.calibrate_client, 'calibrate')
            return
        if command == 'R':
            self._call_trigger(self.reset_client, 'reset_state')
            return

        # Prefix table is ordered longest-first; first match wins.
        for prefix, parameter in COMMAND_TABLE:
            if command.startswith(prefix):
                payload = command[len(prefix):].strip()
                try:
                    value = float(payload)
                except ValueError:
                    self.get_logger().warning(
                        "Malformed value in '%s' - ignoring" % command)
                    return
                self._set_parameter(parameter, value)
                return

        # Unknown commands are dropped, as the firmware does.
        self.get_logger().debug("Unknown command '%s' - ignoring" % command)

    def _set_parameter(self, name, value):
        if not self.param_client.service_is_ready():
            self.get_logger().warning(
                'balance_point parameter service unavailable; dropped %s=%.4f'
                % (name, value))
            return

        request = SetParameters.Request()
        parameter = Parameter()
        parameter.name = name
        parameter.value = ParameterValue(
            type=ParameterType.PARAMETER_DOUBLE, double_value=float(value))
        request.parameters = [parameter]

        future = self.param_client.call_async(request)
        future.add_done_callback(
            lambda f, n=name, v=value: self._on_parameter_set(f, n, v))

    def _on_parameter_set(self, future, name, value):
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001 - report and continue serving
            self.get_logger().error('Parameter set failed: %s' % exc)
            return

        ok = bool(response.results) and response.results[0].successful
        if ok:
            if name in self._mirrored:
                self._mirrored[name] = value
            self.get_logger().info('Set %s = %.4f' % (name, value))
            self._send_to_all('Updated %s%.4f|' % (name, value))
        else:
            reason = response.results[0].reason if response.results else 'unknown'
            self.get_logger().warning(
                'Rejected %s = %.4f (%s)' % (name, value, reason))
            self._send_to_all('ERR %s rejected: %s|' % (name, reason))

    def _toggle_motors(self):
        if not self.arm_client.service_is_ready():
            self.get_logger().warning('/motors_enabled unavailable')
            self._send_to_all('ERR motors service unavailable|')
            return

        request = SetBool.Request()
        request.data = not self._motors_enabled

        future = self.arm_client.call_async(request)
        future.add_done_callback(self._on_motors_toggled)

    def _on_motors_toggled(self, future):
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error('Motor toggle failed: %s' % exc)
            return

        self.get_logger().info('%s' % response.message)
        self._send_to_all('%s|' % response.message)

    def _call_trigger(self, client, label):
        if not client.service_is_ready():
            self.get_logger().warning('/%s unavailable' % label)
            self._send_to_all('ERR %s unavailable|' % label)
            return

        future = client.call_async(Trigger.Request())
        future.add_done_callback(
            lambda f, l=label: self._on_trigger_done(f, l))

    def _on_trigger_done(self, future, label):
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error('%s failed: %s' % (label, exc))
            return

        self.get_logger().info('%s: %s' % (label, response.message))
        self._send_to_all('%s|' % response.message)

    def _refresh_mirrored(self):
        """Re-read gains that appear in telemetry but not in the array.

        Keeps the GUI's displayed values correct when another client or
        ``ros2 param set`` changes them behind this bridge's back.
        """
        if not self.get_param_client.service_is_ready():
            return

        names = list(self._mirrored.keys())
        request = GetParameters.Request()
        request.names = names

        future = self.get_param_client.call_async(request)
        future.add_done_callback(
            lambda f, n=names: self._on_mirrored_read(f, n))

    def _on_mirrored_read(self, future, names):
        try:
            response = future.result()
        except Exception:  # noqa: BLE001 - a failed refresh is not fatal
            return

        for name, value in zip(names, response.values):
            if value.type == ParameterType.PARAMETER_DOUBLE:
                self._mirrored[name] = value.double_value

    # ------------------------------------------------------------------
    # Telemetry
    # ------------------------------------------------------------------

    def _on_telemetry(self, msg):
        if len(msg.data) >= 16:
            self._telemetry = list(msg.data)
            # Track arm state from the authoritative source so the M toggle
            # stays in step even if the robot safety-latched on its own.
            self._motors_enabled = self._telemetry[T_MOTORS] > 0.5

    def _broadcast_telemetry(self):
        if self._telemetry is None:
            return

        with self._clients_lock:
            if not self._clients:
                return

        t = self._telemetry
        self._seq = (self._seq + 1) % 65536

        frame = (
            'S%d DT%d P%.2f O%.2f I%.4f V%.1f TB%.3f EP%.1f EV%.1f '
            'VR%.3f ST%d A%.4f T%.1f M%d L%d|'
        ) % (
            self._seq,
            int(t[T_DT] * 1e6),
            t[T_PITCH],
            t[T_OUTPUT],
            t[T_INTEGRAL],
            t[T_VEL],
            t[T_TILT_BIAS],
            t[T_POS_ERR],
            t[T_VEL_ERR],
            self._mirrored['vel_alpha'],
            int(t[T_STATE]),
            self._mirrored['alpha'],
            self._mirrored['max_safe_tilt'],
            int(t[T_MOTORS]),
            int(t[T_LATCHED]),
        )
        self._send_to_all(frame)

    # ------------------------------------------------------------------

    def shutdown(self):
        self._shutdown.set()
        try:
            self._server_socket.close()
        except OSError:
            pass
        with self._clients_lock:
            targets = list(self._clients)
        for client in targets:
            self._remove_client(client)


def main(args=None):
    rclpy.init(args=args)
    node = WirelessGuiBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
