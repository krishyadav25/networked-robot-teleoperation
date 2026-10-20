"""
test_safety.py – Unit tests for src.safety, src.robot_interface,
and integrated TCP safety behaviour.

Uses a controllable fake clock for deterministic watchdog tests.

Run with:
    python -m unittest discover -s tests -v
"""

import math
import socket
import threading
import time
import unittest

from src.protocol import MessageBuffer, encode_message
from src.robot_interface import MockRobot, RobotStatus
from src.safety import RobotState, SafetyConfig, SafetyController


# ============================================================================
# Helpers
# ============================================================================


class FakeClock:
    """A manually advanceable clock for deterministic tests."""

    def __init__(self, start: float = 0.0) -> None:
        self._now = start

    def __call__(self) -> float:
        return self._now

    def advance(self, seconds: float) -> None:
        self._now += seconds


def make_controller(
    clock: FakeClock | None = None,
    config: SafetyConfig | None = None,
) -> tuple[MockRobot, SafetyController, FakeClock]:
    """Create a (robot, controller, clock) triple for testing."""
    if clock is None:
        clock = FakeClock()
    robot = MockRobot()
    ctrl = SafetyController(robot, config=config, clock=clock)
    return robot, ctrl, clock


# ============================================================================
# 1. Initial state
# ============================================================================


class TestInitialState(unittest.TestCase):
    """The controller must start in IDLE with the robot stopped."""

    def test_initial_state_is_idle(self):
        _, ctrl, _ = make_controller()
        self.assertEqual(ctrl.state, RobotState.IDLE)

    def test_initial_robot_not_moving(self):
        robot, _, _ = make_controller()
        status = robot.get_status()
        self.assertFalse(status.is_moving)
        self.assertEqual(status.linear_velocity, 0.0)


# ============================================================================
# 2. Valid movement commands
# ============================================================================


class TestMovementCommands(unittest.TestCase):
    """Movement commands should update robot state correctly."""

    def test_forward_moves_robot(self):
        robot, ctrl, _ = make_controller()
        resp = ctrl.dispatch({"type": "FORWARD", "speed": 1.0})
        self.assertEqual(resp["status"], "ok")
        self.assertEqual(ctrl.state, RobotState.MOVING)
        self.assertTrue(robot.get_status().is_moving)

    def test_backward_moves_robot(self):
        robot, ctrl, _ = make_controller()
        resp = ctrl.dispatch({"type": "BACKWARD", "speed": 0.5})
        self.assertEqual(resp["status"], "ok")
        self.assertEqual(ctrl.state, RobotState.MOVING)

    def test_turn_left(self):
        robot, ctrl, _ = make_controller()
        resp = ctrl.dispatch({"type": "TURN_LEFT", "speed": 30.0})
        self.assertEqual(resp["status"], "ok")
        self.assertEqual(ctrl.state, RobotState.MOVING)

    def test_turn_right(self):
        robot, ctrl, _ = make_controller()
        resp = ctrl.dispatch({"type": "TURN_RIGHT"})
        self.assertEqual(resp["status"], "ok")
        self.assertEqual(ctrl.state, RobotState.MOVING)

    def test_move_with_velocities(self):
        robot, ctrl, _ = make_controller()
        resp = ctrl.dispatch({"type": "MOVE", "linear": 1.0, "angular": 45.0})
        self.assertEqual(resp["status"], "ok")
        self.assertEqual(ctrl.state, RobotState.MOVING)
        self.assertEqual(resp["linear"], 1.0)
        self.assertEqual(resp["angular"], 45.0)

    def test_move_zero_stays_idle(self):
        _, ctrl, _ = make_controller()
        resp = ctrl.dispatch({"type": "MOVE", "linear": 0, "angular": 0})
        self.assertEqual(resp["status"], "ok")
        self.assertEqual(ctrl.state, RobotState.IDLE)

    def test_goto(self):
        robot, ctrl, _ = make_controller()
        resp = ctrl.dispatch({"type": "GOTO", "x": 3.0, "y": 4.0})
        self.assertEqual(resp["status"], "ok")
        self.assertEqual(ctrl.state, RobotState.MOVING)
        self.assertEqual(resp["target_x"], 3.0)
        self.assertEqual(resp["target_y"], 4.0)

    def test_stop_halts_robot(self):
        robot, ctrl, _ = make_controller()
        ctrl.dispatch({"type": "FORWARD"})
        resp = ctrl.dispatch({"type": "STOP"})
        self.assertEqual(resp["status"], "ok")
        self.assertEqual(ctrl.state, RobotState.IDLE)
        self.assertFalse(robot.get_status().is_moving)

    def test_forward_default_speed(self):
        """FORWARD without a speed field uses the default."""
        _, ctrl, _ = make_controller()
        resp = ctrl.dispatch({"type": "FORWARD"})
        self.assertEqual(resp["speed"], 0.5)

    def test_command_log_records_actions(self):
        """The mock robot should log each command."""
        robot, ctrl, _ = make_controller()
        ctrl.dispatch({"type": "FORWARD", "speed": 1.0})
        ctrl.dispatch({"type": "STOP"})
        self.assertEqual(len(robot.command_log), 2)
        self.assertEqual(robot.command_log[0].method, "forward")
        self.assertEqual(robot.command_log[1].method, "stop")


# ============================================================================
# 3. Invalid commands and values
# ============================================================================


class TestInvalidCommands(unittest.TestCase):
    """Invalid input should be rejected without crashing."""

    def test_unknown_command_type(self):
        _, ctrl, _ = make_controller()
        resp = ctrl.dispatch({"type": "DANCE"})
        self.assertEqual(resp["status"], "unknown_command")

    def test_nan_speed_rejected(self):
        _, ctrl, _ = make_controller()
        resp = ctrl.dispatch({"type": "FORWARD", "speed": float("nan")})
        self.assertEqual(resp["status"], "invalid")

    def test_infinity_speed_rejected(self):
        _, ctrl, _ = make_controller()
        resp = ctrl.dispatch({"type": "FORWARD", "speed": float("inf")})
        self.assertEqual(resp["status"], "invalid")

    def test_negative_infinity_rejected(self):
        _, ctrl, _ = make_controller()
        resp = ctrl.dispatch({"type": "MOVE", "linear": float("-inf"), "angular": 0})
        self.assertEqual(resp["status"], "invalid")

    def test_speed_over_limit_rejected(self):
        _, ctrl, _ = make_controller()
        resp = ctrl.dispatch({"type": "FORWARD", "speed": 999.0})
        self.assertEqual(resp["status"], "invalid")
        self.assertIn("exceeds", resp["message"])

    def test_angular_over_limit_rejected(self):
        _, ctrl, _ = make_controller()
        resp = ctrl.dispatch({"type": "MOVE", "linear": 0, "angular": 999.0})
        self.assertEqual(resp["status"], "invalid")

    def test_non_numeric_speed_rejected(self):
        _, ctrl, _ = make_controller()
        resp = ctrl.dispatch({"type": "FORWARD", "speed": "fast"})
        self.assertEqual(resp["status"], "invalid")

    def test_goto_missing_coordinates(self):
        _, ctrl, _ = make_controller()
        resp = ctrl.dispatch({"type": "GOTO"})
        self.assertEqual(resp["status"], "invalid")

    def test_goto_nan_rejected(self):
        _, ctrl, _ = make_controller()
        resp = ctrl.dispatch({"type": "GOTO", "x": float("nan"), "y": 0})
        self.assertEqual(resp["status"], "invalid")

    def test_goto_out_of_range(self):
        _, ctrl, _ = make_controller()
        resp = ctrl.dispatch({"type": "GOTO", "x": 5000, "y": 0})
        self.assertEqual(resp["status"], "invalid")


# ============================================================================
# 4. Heartbeat refreshes the watchdog
# ============================================================================


class TestHeartbeat(unittest.TestCase):
    """Heartbeat recording and watchdog refresh."""

    def test_heartbeat_returns_ack(self):
        _, ctrl, _ = make_controller()
        resp = ctrl.dispatch({"type": "HEARTBEAT"})
        self.assertEqual(resp["type"], "HEARTBEAT_ACK")
        self.assertEqual(resp["status"], "ok")

    def test_heartbeat_refreshes_watchdog(self):
        """Heartbeat before timeout keeps the robot alive."""
        robot, ctrl, clock = make_controller()
        ctrl.dispatch({"type": "FORWARD"})
        clock.advance(1.0)  # within 1.5 s timeout
        ctrl.record_heartbeat()
        ctrl._check_watchdog()
        self.assertEqual(ctrl.state, RobotState.MOVING)


# ============================================================================
# 5. Watchdog expiry
# ============================================================================


class TestWatchdogExpiry(unittest.TestCase):
    """Watchdog must stop the robot on heartbeat timeout."""

    def test_watchdog_triggers_comm_lost(self):
        robot, ctrl, clock = make_controller()
        ctrl.dispatch({"type": "FORWARD"})
        self.assertEqual(ctrl.state, RobotState.MOVING)

        clock.advance(2.0)  # past 1.5 s timeout
        ctrl._check_watchdog()

        self.assertEqual(ctrl.state, RobotState.COMM_LOST)
        self.assertFalse(robot.get_status().is_moving)

    def test_watchdog_stops_robot(self):
        robot, ctrl, clock = make_controller()
        ctrl.dispatch({"type": "FORWARD"})
        clock.advance(2.0)
        ctrl._check_watchdog()
        # Robot should have been stopped.
        last_cmd = robot.command_log[-1]
        self.assertEqual(last_cmd.method, "stop")


# ============================================================================
# 6. Heartbeat recovery does NOT resume motion
# ============================================================================


class TestHeartbeatRecovery(unittest.TestCase):
    """After COMM_LOST, heartbeat resumes to IDLE, not MOVING."""

    def test_recovery_sets_idle_not_moving(self):
        robot, ctrl, clock = make_controller()
        ctrl.dispatch({"type": "FORWARD"})
        clock.advance(2.0)
        ctrl._check_watchdog()
        self.assertEqual(ctrl.state, RobotState.COMM_LOST)

        # Heartbeat resumes
        ctrl.record_heartbeat()
        self.assertEqual(ctrl.state, RobotState.IDLE)
        # Robot is still stopped
        self.assertFalse(robot.get_status().is_moving)

    def test_late_heartbeat_does_not_restart(self):
        """A late heartbeat must NOT restart previous motion."""
        robot, ctrl, clock = make_controller()
        ctrl.dispatch({"type": "FORWARD"})
        clock.advance(2.0)
        ctrl._check_watchdog()

        clock.advance(0.1)
        ctrl.record_heartbeat()
        # Must be IDLE, not MOVING
        self.assertEqual(ctrl.state, RobotState.IDLE)
        self.assertFalse(robot.get_status().is_moving)


# ============================================================================
# 7. New movement after recovery
# ============================================================================


class TestMovementAfterRecovery(unittest.TestCase):
    """A fresh movement command should work after COMM_LOST → IDLE."""

    def test_move_after_comm_recovery(self):
        robot, ctrl, clock = make_controller()
        ctrl.dispatch({"type": "FORWARD"})
        clock.advance(2.0)
        ctrl._check_watchdog()
        ctrl.record_heartbeat()
        self.assertEqual(ctrl.state, RobotState.IDLE)

        resp = ctrl.dispatch({"type": "FORWARD"})
        self.assertEqual(resp["status"], "ok")
        self.assertEqual(ctrl.state, RobotState.MOVING)


# ============================================================================
# 8. Emergency stop
# ============================================================================


class TestEmergencyStop(unittest.TestCase):
    """ESTOP must stop immediately and latch."""

    def test_estop_stops_robot(self):
        robot, ctrl, _ = make_controller()
        ctrl.dispatch({"type": "FORWARD"})
        resp = ctrl.dispatch({"type": "ESTOP"})
        self.assertEqual(resp["type"], "ESTOP_ACK")
        self.assertEqual(ctrl.state, RobotState.ESTOP_LATCHED)
        self.assertFalse(robot.get_status().is_moving)

    def test_estop_from_idle(self):
        _, ctrl, _ = make_controller()
        ctrl.dispatch({"type": "ESTOP"})
        self.assertEqual(ctrl.state, RobotState.ESTOP_LATCHED)

    def test_double_estop_safe(self):
        """Sending ESTOP twice must not crash."""
        _, ctrl, _ = make_controller()
        ctrl.dispatch({"type": "ESTOP"})
        resp = ctrl.dispatch({"type": "ESTOP"})
        self.assertEqual(ctrl.state, RobotState.ESTOP_LATCHED)
        self.assertEqual(resp["status"], "ok")


# ============================================================================
# 9. Movement rejected while E-stop latched
# ============================================================================


class TestEstopBlocksMovement(unittest.TestCase):
    """All movement must be rejected while E-stop is latched."""

    def test_forward_rejected(self):
        _, ctrl, _ = make_controller()
        ctrl.dispatch({"type": "ESTOP"})
        resp = ctrl.dispatch({"type": "FORWARD"})
        self.assertEqual(resp["status"], "rejected")
        self.assertEqual(ctrl.state, RobotState.ESTOP_LATCHED)

    def test_backward_rejected(self):
        _, ctrl, _ = make_controller()
        ctrl.dispatch({"type": "ESTOP"})
        resp = ctrl.dispatch({"type": "BACKWARD"})
        self.assertEqual(resp["status"], "rejected")

    def test_move_rejected(self):
        _, ctrl, _ = make_controller()
        ctrl.dispatch({"type": "ESTOP"})
        resp = ctrl.dispatch({"type": "MOVE", "linear": 1, "angular": 0})
        self.assertEqual(resp["status"], "rejected")

    def test_turn_rejected(self):
        _, ctrl, _ = make_controller()
        ctrl.dispatch({"type": "ESTOP"})
        resp = ctrl.dispatch({"type": "TURN_LEFT"})
        self.assertEqual(resp["status"], "rejected")

    def test_goto_rejected(self):
        _, ctrl, _ = make_controller()
        ctrl.dispatch({"type": "ESTOP"})
        resp = ctrl.dispatch({"type": "GOTO", "x": 1, "y": 1})
        self.assertEqual(resp["status"], "rejected")

    def test_stop_during_estop_stays_latched(self):
        """STOP is allowed but does not clear the latch."""
        _, ctrl, _ = make_controller()
        ctrl.dispatch({"type": "ESTOP"})
        resp = ctrl.dispatch({"type": "STOP"})
        self.assertEqual(resp["status"], "ok")
        self.assertEqual(ctrl.state, RobotState.ESTOP_LATCHED)


# ============================================================================
# 10. Reset clears latch without restarting motion
# ============================================================================


class TestReset(unittest.TestCase):
    """RESET must clear the E-stop latch and leave robot in IDLE."""

    def test_reset_clears_latch(self):
        _, ctrl, _ = make_controller()
        ctrl.dispatch({"type": "ESTOP"})
        resp = ctrl.dispatch({"type": "RESET"})
        self.assertEqual(resp["status"], "ok")
        self.assertEqual(ctrl.state, RobotState.IDLE)

    def test_reset_does_not_resume_motion(self):
        """If the robot was moving before ESTOP, RESET leaves it stopped."""
        robot, ctrl, _ = make_controller()
        ctrl.dispatch({"type": "FORWARD"})
        ctrl.dispatch({"type": "ESTOP"})
        ctrl.dispatch({"type": "RESET"})
        self.assertEqual(ctrl.state, RobotState.IDLE)
        self.assertFalse(robot.get_status().is_moving)

    def test_reset_without_estop_is_error(self):
        _, ctrl, _ = make_controller()
        resp = ctrl.dispatch({"type": "RESET"})
        self.assertEqual(resp["status"], "error")

    def test_move_after_reset(self):
        """After RESET, a fresh FORWARD should succeed."""
        _, ctrl, _ = make_controller()
        ctrl.dispatch({"type": "ESTOP"})
        ctrl.dispatch({"type": "RESET"})
        resp = ctrl.dispatch({"type": "FORWARD"})
        self.assertEqual(resp["status"], "ok")
        self.assertEqual(ctrl.state, RobotState.MOVING)


# ============================================================================
# 11. Communication loss never clears E-stop latch
# ============================================================================


class TestCommLossDoesNotClearEstop(unittest.TestCase):
    """COMM_LOST must never override ESTOP_LATCHED."""

    def test_watchdog_preserves_estop(self):
        _, ctrl, clock = make_controller()
        ctrl.dispatch({"type": "ESTOP"})
        clock.advance(5.0)  # well past timeout
        ctrl._check_watchdog()
        # E-stop still active
        self.assertEqual(ctrl.state, RobotState.ESTOP_LATCHED)

    def test_disconnect_preserves_estop(self):
        _, ctrl, _ = make_controller()
        ctrl.dispatch({"type": "ESTOP"})
        ctrl.on_client_disconnect()
        self.assertEqual(ctrl.state, RobotState.ESTOP_LATCHED)


# ============================================================================
# 12. Client disconnect stops the robot
# ============================================================================


class TestClientDisconnect(unittest.TestCase):
    """Disconnecting the client must stop the robot."""

    def test_disconnect_stops_moving_robot(self):
        robot, ctrl, _ = make_controller()
        ctrl.dispatch({"type": "FORWARD"})
        ctrl.on_client_disconnect()
        self.assertFalse(robot.get_status().is_moving)
        self.assertEqual(ctrl.state, RobotState.COMM_LOST)

    def test_disconnect_from_idle(self):
        robot, ctrl, _ = make_controller()
        ctrl.on_client_disconnect()
        self.assertFalse(robot.get_status().is_moving)


# ============================================================================
# 13. Status reporting
# ============================================================================


class TestStatusReporting(unittest.TestCase):
    """STATUS command returns current robot state and status."""

    def test_status_idle(self):
        _, ctrl, _ = make_controller()
        resp = ctrl.dispatch({"type": "STATUS"})
        self.assertEqual(resp["type"], "STATUS_RESPONSE")
        self.assertEqual(resp["state"], "IDLE")
        self.assertFalse(resp["is_moving"])

    def test_status_while_moving(self):
        _, ctrl, _ = make_controller()
        ctrl.dispatch({"type": "FORWARD", "speed": 1.0})
        resp = ctrl.dispatch({"type": "STATUS"})
        self.assertEqual(resp["state"], "MOVING")
        self.assertTrue(resp["is_moving"])

    def test_status_estop(self):
        _, ctrl, _ = make_controller()
        ctrl.dispatch({"type": "ESTOP"})
        resp = ctrl.dispatch({"type": "STATUS"})
        self.assertEqual(resp["state"], "ESTOP_LATCHED")

    def test_status_comm_lost(self):
        _, ctrl, clock = make_controller()
        ctrl.dispatch({"type": "FORWARD"})
        clock.advance(2.0)
        ctrl._check_watchdog()
        resp = ctrl.dispatch({"type": "STATUS"})
        self.assertEqual(resp["state"], "COMM_LOST")


# ============================================================================
# 14. TCP integration with safety controller
# ============================================================================


class TestTCPSafetyIntegration(unittest.TestCase):
    """End-to-end test: real TCP with SafetyController."""

    def _run_server(
        self,
        server_sock: socket.socket,
        controller: SafetyController,
        ready: threading.Event,
    ) -> None:
        ready.set()
        conn, _ = server_sock.accept()
        conn.settimeout(5)
        buf = MessageBuffer()
        try:
            while True:
                data = conn.recv(4096)
                if not data:
                    break
                buf.feed(data)
                for msg in buf.get_messages():
                    resp = controller.dispatch(msg)
                    conn.sendall(encode_message(resp))
        except socket.timeout:
            pass
        finally:
            controller.on_client_disconnect()
            conn.close()

    def _setup_server(self) -> tuple[socket.socket, SafetyController, threading.Thread, threading.Event]:
        robot = MockRobot()
        clock = FakeClock()
        controller = SafetyController(robot, clock=clock)

        server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_sock.bind(("127.0.0.1", 0))
        server_sock.listen(1)

        ready = threading.Event()
        t = threading.Thread(
            target=self._run_server,
            args=(server_sock, controller, ready),
            daemon=True,
        )
        t.start()
        ready.wait(timeout=5)
        return server_sock, controller, t, ready

    def test_forward_and_stop_over_tcp(self):
        server_sock, controller, t, _ = self._setup_server()
        port = server_sock.getsockname()[1]

        try:
            client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            client.settimeout(5)
            client.connect(("127.0.0.1", port))

            # Send FORWARD
            client.sendall(encode_message({"type": "FORWARD", "speed": 1.0}))
            time.sleep(0.1)
            buf = MessageBuffer()
            buf.feed(client.recv(4096))
            responses = buf.get_messages()
            self.assertEqual(responses[0]["type"], "FORWARD_ACK")
            self.assertEqual(responses[0]["state"], "MOVING")

            # Send STOP
            client.sendall(encode_message({"type": "STOP"}))
            time.sleep(0.1)
            buf.feed(client.recv(4096))
            responses = buf.get_messages()
            self.assertEqual(responses[0]["type"], "STOP_ACK")
            self.assertEqual(responses[0]["state"], "IDLE")

            client.close()
        finally:
            server_sock.close()
            t.join(timeout=5)

    def test_estop_and_reset_over_tcp(self):
        server_sock, controller, t, _ = self._setup_server()
        port = server_sock.getsockname()[1]

        try:
            client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            client.settimeout(5)
            client.connect(("127.0.0.1", port))
            buf = MessageBuffer()

            # ESTOP
            client.sendall(encode_message({"type": "ESTOP"}))
            time.sleep(0.1)
            buf.feed(client.recv(4096))
            responses = buf.get_messages()
            self.assertEqual(responses[0]["state"], "ESTOP_LATCHED")

            # FORWARD should be rejected
            client.sendall(encode_message({"type": "FORWARD"}))
            time.sleep(0.1)
            buf.feed(client.recv(4096))
            responses = buf.get_messages()
            self.assertEqual(responses[0]["status"], "rejected")

            # RESET
            client.sendall(encode_message({"type": "RESET"}))
            time.sleep(0.1)
            buf.feed(client.recv(4096))
            responses = buf.get_messages()
            self.assertEqual(responses[0]["state"], "IDLE")

            # FORWARD should work now
            client.sendall(encode_message({"type": "FORWARD"}))
            time.sleep(0.1)
            buf.feed(client.recv(4096))
            responses = buf.get_messages()
            self.assertEqual(responses[0]["type"], "FORWARD_ACK")

            client.close()
        finally:
            server_sock.close()
            t.join(timeout=5)

    def test_multiple_commands_over_tcp(self):
        server_sock, controller, t, _ = self._setup_server()
        port = server_sock.getsockname()[1]

        try:
            client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            client.settimeout(5)
            client.connect(("127.0.0.1", port))
            buf = MessageBuffer()

            # Send HEARTBEAT + STATUS in one burst
            m1 = encode_message({"type": "HEARTBEAT"})
            m2 = encode_message({"type": "STATUS"})
            client.sendall(m1 + m2)
            time.sleep(0.2)
            buf.feed(client.recv(4096))
            responses = buf.get_messages()

            self.assertEqual(len(responses), 2)
            self.assertEqual(responses[0]["type"], "HEARTBEAT_ACK")
            self.assertEqual(responses[1]["type"], "STATUS_RESPONSE")

            client.close()
        finally:
            server_sock.close()
            t.join(timeout=5)


# ============================================================================
# 15. MockRobot unit tests
# ============================================================================


class TestMockRobot(unittest.TestCase):
    """Basic sanity tests for the MockRobot backend."""

    def test_initial_status(self):
        robot = MockRobot()
        s = robot.get_status()
        self.assertFalse(s.is_moving)
        self.assertEqual(s.linear_velocity, 0.0)

    def test_forward_sets_velocity(self):
        robot = MockRobot()
        robot.forward(1.5)
        s = robot.get_status()
        self.assertTrue(s.is_moving)
        self.assertEqual(s.linear_velocity, 1.5)

    def test_stop_clears_velocity(self):
        robot = MockRobot()
        robot.forward(1.0)
        robot.stop()
        s = robot.get_status()
        self.assertFalse(s.is_moving)
        self.assertEqual(s.linear_velocity, 0.0)

    def test_get_status_returns_copy(self):
        robot = MockRobot()
        s1 = robot.get_status()
        s1.x = 999.0
        s2 = robot.get_status()
        self.assertNotEqual(s2.x, 999.0)


# ============================================================================
# 16. Watchdog thread integration (with real timer, short interval)
# ============================================================================


class TestWatchdogThread(unittest.TestCase):
    """Test that the watchdog thread actually fires."""

    def test_watchdog_thread_triggers_comm_lost(self):
        robot = MockRobot()
        config = SafetyConfig(heartbeat_timeout=0.3)
        controller = SafetyController(robot, config=config)

        controller.dispatch({"type": "FORWARD"})
        self.assertEqual(controller.state, RobotState.MOVING)

        controller.start_watchdog(interval=0.1)
        try:
            # Wait long enough for the watchdog to trigger
            time.sleep(0.8)
            self.assertEqual(controller.state, RobotState.COMM_LOST)
            self.assertFalse(robot.get_status().is_moving)
        finally:
            controller.stop_watchdog()


# ============================================================================
# 17. Advanced safety, validation, and COMM_LOST edge cases
# ============================================================================


class TestAdvancedSafetyEdgeCases(unittest.TestCase):
    """Edge cases for input types, COMM_LOST blocking, and PING."""

    def test_negative_speed_forward_rejected(self):
        _, ctrl, _ = make_controller()
        resp = ctrl.dispatch({"type": "FORWARD", "speed": -0.5})
        self.assertEqual(resp["status"], "invalid")
        self.assertIn("non-negative", resp["message"])

    def test_negative_speed_backward_rejected(self):
        _, ctrl, _ = make_controller()
        resp = ctrl.dispatch({"type": "BACKWARD", "speed": -0.5})
        self.assertEqual(resp["status"], "invalid")
        self.assertIn("non-negative", resp["message"])

    def test_negative_speed_turn_left_rejected(self):
        _, ctrl, _ = make_controller()
        resp = ctrl.dispatch({"type": "TURN_LEFT", "speed": -10.0})
        self.assertEqual(resp["status"], "invalid")
        self.assertIn("non-negative", resp["message"])

    def test_negative_speed_turn_right_rejected(self):
        _, ctrl, _ = make_controller()
        resp = ctrl.dispatch({"type": "TURN_RIGHT", "speed": -10.0})
        self.assertEqual(resp["status"], "invalid")
        self.assertIn("non-negative", resp["message"])

    def test_boolean_speed_rejected(self):
        _, ctrl, _ = make_controller()
        resp = ctrl.dispatch({"type": "FORWARD", "speed": True})
        self.assertEqual(resp["status"], "invalid")

    def test_boolean_coordinate_rejected(self):
        _, ctrl, _ = make_controller()
        resp = ctrl.dispatch({"type": "GOTO", "x": True, "y": 10.0})
        self.assertEqual(resp["status"], "invalid")

    def test_dispatch_non_dict_rejected(self):
        _, ctrl, _ = make_controller()
        resp = ctrl.dispatch("not a dict")  # type: ignore[arg-type]
        self.assertEqual(resp["status"], "invalid")

    def test_dispatch_missing_type_rejected(self):
        _, ctrl, _ = make_controller()
        resp = ctrl.dispatch({"speed": 1.0})
        self.assertEqual(resp["status"], "invalid")

    def test_dispatch_whitespace_type_rejected(self):
        _, ctrl, _ = make_controller()
        resp = ctrl.dispatch({"type": "   "})
        self.assertEqual(resp["status"], "invalid")

    def test_dispatch_ping_pong(self):
        _, ctrl, _ = make_controller()
        resp = ctrl.dispatch({"type": "PING"})
        self.assertEqual(resp["type"], "ACK")
        self.assertEqual(resp["received_type"], "PING")
        self.assertEqual(resp["status"], "ok")
        self.assertEqual(resp["state"], "IDLE")

    def test_movement_rejected_in_comm_lost(self):
        robot, ctrl, clock = make_controller()
        ctrl.dispatch({"type": "FORWARD"})
        clock.advance(2.0)
        ctrl._check_watchdog()
        self.assertEqual(ctrl.state, RobotState.COMM_LOST)

        # Movement commands must be rejected
        for cmd in [
            {"type": "FORWARD"},
            {"type": "BACKWARD"},
            {"type": "MOVE", "linear": 0.5, "angular": 0.0},
            {"type": "TURN_LEFT"},
            {"type": "TURN_RIGHT"},
            {"type": "GOTO", "x": 1.0, "y": 1.0},
        ]:
            resp = ctrl.dispatch(cmd)
            self.assertEqual(resp["status"], "rejected")
            self.assertEqual(ctrl.state, RobotState.COMM_LOST)

    def test_stop_in_comm_lost_stays_comm_lost(self):
        robot, ctrl, clock = make_controller()
        ctrl.dispatch({"type": "FORWARD"})
        clock.advance(2.0)
        ctrl._check_watchdog()
        self.assertEqual(ctrl.state, RobotState.COMM_LOST)

        resp = ctrl.dispatch({"type": "STOP"})
        self.assertEqual(resp["status"], "ok")
        self.assertEqual(ctrl.state, RobotState.COMM_LOST)

    def test_watchdog_timeout_from_idle_sets_comm_lost(self):
        robot, ctrl, clock = make_controller()
        self.assertEqual(ctrl.state, RobotState.IDLE)
        clock.advance(2.0)
        ctrl._check_watchdog()
        self.assertEqual(ctrl.state, RobotState.COMM_LOST)


if __name__ == "__main__":
    unittest.main()
