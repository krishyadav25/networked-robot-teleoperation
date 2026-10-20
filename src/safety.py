"""
safety.py – Safety controller and command dispatcher for AISR.

This module enforces the robot's operational safety rules:
- State machine: IDLE → MOVING → IDLE / COMM_LOST / ESTOP_LATCHED
- Heartbeat watchdog: stops the robot if heartbeats expire.
- Emergency-stop latch: immediately stops the robot and blocks motion
  until explicitly reset.
- Velocity validation: rejects NaN, infinity, and out-of-range values.
- Command dispatch: maps incoming JSON commands to robot actions.

Classes
-------
RobotState
    Enum of the four operational states.
SafetyController
    Thread-safe controller wrapping a ``RobotInterface``.
"""

from __future__ import annotations

import enum
import math
import threading
from dataclasses import dataclass
from typing import Any, Callable, Optional


from src.robot_interface import RobotInterface, RobotStatus


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

@dataclass
class SafetyConfig:
    """Tunable safety parameters."""
    heartbeat_timeout: float = 1.5     # seconds without heartbeat → COMM_LOST
    max_linear_speed: float = 2.0      # m/s
    max_angular_speed: float = 90.0    # deg/s
    max_goto_range: float = 1000.0     # metres from origin
    default_speed: float = 0.5         # m/s for FORWARD / BACKWARD
    default_turn_speed: float = 45.0   # deg/s for TURN_LEFT / TURN_RIGHT
    max_duration: float = 300.0        # maximum duration in seconds for timed motions


# ---------------------------------------------------------------------------
# State enum
# ---------------------------------------------------------------------------

class RobotState(enum.Enum):
    """Operational states of the robot."""
    IDLE = "IDLE"
    MOVING = "MOVING"
    COMM_LOST = "COMM_LOST"
    ESTOP_LATCHED = "ESTOP_LATCHED"


# ---------------------------------------------------------------------------
# Safety controller
# ---------------------------------------------------------------------------

class SafetyController:
    """Thread-safe safety controller wrapping a robot backend.

    Parameters
    ----------
    robot : RobotInterface
        The robot backend to command.
    config : SafetyConfig, optional
        Tunable parameters.  Defaults are sensible for testing.
    clock : callable, optional
        A callable returning monotonic seconds.  Defaults to
        ``time.monotonic``.  Inject a fake clock for deterministic
        unit tests.
    """

    def __init__(
        self,
        robot: RobotInterface,
        config: SafetyConfig | None = None,
        clock: Callable[[], float] | None = None,
        timer_factory: Callable[[float, Callable[[], None]], Any] | None = None,
    ) -> None:
        import time as _time

        self._robot = robot
        self._config = config or SafetyConfig()
        self._clock = clock or _time.monotonic
        self._timer_factory = timer_factory or (
            lambda delay, cb: threading.Timer(delay, cb)
        )

        self._lock = threading.Lock()
        self._state = RobotState.IDLE
        self._last_heartbeat: float = self._clock()
        self._watchdog_running = False
        self._watchdog_thread: threading.Thread | None = None
        self._watchdog_stop = threading.Event()

        # Motion timer tracking for safe timed movements
        self._motion_generation: int = 0
        self._motion_timer: Any = None

    # -----------------------------------------------------------------------
    # Properties
    # -----------------------------------------------------------------------

    @property
    def state(self) -> RobotState:
        """Current operational state (thread-safe read)."""
        with self._lock:
            return self._state

    @property
    def config(self) -> SafetyConfig:
        return self._config

    # -----------------------------------------------------------------------
    # Heartbeat
    # -----------------------------------------------------------------------

    def record_heartbeat(self) -> dict:
        """Record a heartbeat and return an ACK response.

        If the robot was in ``COMM_LOST``, it transitions back to
        ``IDLE`` — but it does **not** resume motion.
        """
        with self._lock:
            self._last_heartbeat = self._clock()
            if self._state == RobotState.COMM_LOST:
                self._state = RobotState.IDLE
            return {
                "type": "HEARTBEAT_ACK",
                "state": self._state.value,
                "status": "ok",
            }

    def _check_watchdog(self) -> None:
        """Called periodically to check heartbeat freshness.

        If the heartbeat has expired, stop the robot and enter
        ``COMM_LOST`` — unless the E-stop is latched (which has
        higher priority).
        """
        with self._lock:
            elapsed = self._clock() - self._last_heartbeat
            if elapsed > self._config.heartbeat_timeout:
                if self._state == RobotState.ESTOP_LATCHED:
                    return  # E-stop takes precedence
                if self._state != RobotState.COMM_LOST:
                    self._cancel_motion_timer()
                    self._robot.stop()
                    self._state = RobotState.COMM_LOST

    def start_watchdog(self, interval: float = 0.25) -> None:
        """Start the background heartbeat-watchdog thread.

        Parameters
        ----------
        interval : float
            How often to check heartbeat freshness, in seconds.
        """
        if self._watchdog_running:
            return
        self._watchdog_stop.clear()
        self._watchdog_running = True
        self._watchdog_thread = threading.Thread(
            target=self._watchdog_loop,
            args=(interval,),
            daemon=True,
            name="heartbeat-watchdog",
        )
        self._watchdog_thread.start()

    def stop_watchdog(self) -> None:
        """Stop the background watchdog thread cleanly and cancel pending timers."""
        self._watchdog_stop.set()
        if self._watchdog_thread is not None:
            self._watchdog_thread.join(timeout=5)
        self._watchdog_running = False
        with self._lock:
            self._cancel_motion_timer()
            self._robot.stop()

    def _watchdog_loop(self, interval: float) -> None:
        """Internal loop for the watchdog thread."""
        while not self._watchdog_stop.wait(timeout=interval):
            self._check_watchdog()

    # -----------------------------------------------------------------------
    # Emergency stop
    # -----------------------------------------------------------------------

    def emergency_stop(self) -> dict:
        """Immediately stop the robot, cancel pending timers, and latch the E-stop.

        While latched, all movement commands are rejected.
        """
        with self._lock:
            self._cancel_motion_timer()
            self._robot.stop()
            self._state = RobotState.ESTOP_LATCHED
            return {
                "type": "ESTOP_ACK",
                "state": self._state.value,
                "status": "ok",
                "message": "Emergency stop activated. Use RESET to clear.",
            }

    def reset(self) -> dict:
        """Clear the E-stop latch and ensure timers are cancelled.  Robot stays stopped in IDLE.

        Only valid when the E-stop is latched.
        """
        with self._lock:
            if self._state != RobotState.ESTOP_LATCHED:
                return {
                    "type": "RESET_ACK",
                    "state": self._state.value,
                    "status": "error",
                    "message": "Cannot reset: emergency stop is not active.",
                }
            self._cancel_motion_timer()
            self._robot.stop()  # ensure stopped
            self._state = RobotState.IDLE
            return {
                "type": "RESET_ACK",
                "state": self._state.value,
                "status": "ok",
                "message": "Emergency stop cleared. Robot is IDLE.",
            }

    # -----------------------------------------------------------------------
    # Client disconnect
    # -----------------------------------------------------------------------

    def on_client_disconnect(self) -> None:
        """Called when a client disconnects. Cancel timers and stop the robot."""
        with self._lock:
            self._cancel_motion_timer()
            self._robot.stop()
            if self._state != RobotState.ESTOP_LATCHED:
                self._state = RobotState.COMM_LOST

    # -----------------------------------------------------------------------
    # Timed motion safety
    # -----------------------------------------------------------------------

    def _cancel_motion_timer(self) -> None:
        """Cancel any pending timed-motion timer and invalidate generation.

        Must be called while ``self._lock`` is held.
        """
        self._motion_generation += 1
        if self._motion_timer is not None:
            try:
                self._motion_timer.cancel()
            except Exception:
                pass
            self._motion_timer = None

    def _schedule_motion_timer(self, duration: float) -> None:
        """Schedule a timed stop after ``duration`` seconds.

        Must be called while ``self._lock`` is held.
        """
        self._cancel_motion_timer()
        current_gen = self._motion_generation

        def _on_timeout() -> None:
            self._on_motion_timer_expired(current_gen)

        timer = self._timer_factory(duration, _on_timeout)
        self._motion_timer = timer
        if hasattr(timer, "start"):
            timer.start()

    def _on_motion_timer_expired(self, generation: int) -> None:
        """Callback invoked when a motion timer fires."""
        with self._lock:
            # Stale timer check: if generation does not match, this timer was
            # superseded by a newer command or cancelled.
            if self._motion_generation != generation:
                return

            self._motion_timer = None
            if self._state == RobotState.MOVING:
                self._robot.stop()
                self._state = RobotState.IDLE

    # -----------------------------------------------------------------------
    # Parameter validation
    # -----------------------------------------------------------------------

    def _validate_duration(self, value: Any) -> float | None:
        """Validate optional motion duration in seconds.

        Returns None if duration is omitted or None.
        Raises ValueError if duration is boolean, non-numeric, non-positive,
        NaN, infinite, or exceeds max_duration.
        """
        if value is None:
            return None
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f"duration must be a number, got {type(value).__name__}")
        fval = float(value)
        if math.isnan(fval) or math.isinf(fval):
            raise ValueError(f"duration must be finite, got {fval}")
        if fval <= 0:
            raise ValueError(f"duration must be positive (> 0), got {fval}")
        if fval > self._config.max_duration:
            raise ValueError(
                f"duration ({fval}s) exceeds maximum allowed {self._config.max_duration}s"
            )
        return fval

    def _validate_finite(self, value: Any, name: str) -> float:
        """Validate that *value* is a finite number.

        Raises ValueError for non-numeric, boolean, NaN, or infinity.
        """
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f"{name} must be a number, got {type(value).__name__}")
        fval = float(value)
        if math.isnan(fval) or math.isinf(fval):
            raise ValueError(f"{name} must be finite, got {fval}")
        return fval

    def _validate_speed(
        self,
        value: Any,
        name: str,
        limit: float,
        allow_negative: bool = True,
    ) -> float:
        """Validate a speed is finite, within configured limit, and optionally non-negative."""
        fval = self._validate_finite(value, name)
        if not allow_negative and fval < 0:
            raise ValueError(f"{name} must be non-negative, got {fval}")
        if abs(fval) > limit:
            raise ValueError(
                f"{name} ({fval}) exceeds limit ±{limit}"
            )
        return fval

    # -----------------------------------------------------------------------
    # Movement commands (all check E-stop and COMM_LOST first)
    # -----------------------------------------------------------------------

    def _check_can_move(self) -> Optional[dict]:
        """Return an error response if movement is currently blocked.

        Must be called while ``self._lock`` is held.
        """
        if self._state == RobotState.ESTOP_LATCHED:
            return {
                "type": "ERROR",
                "status": "rejected",
                "state": self._state.value,
                "message": "Emergency stop is active. Send RESET first.",
            }
        if self._state == RobotState.COMM_LOST:
            return {
                "type": "ERROR",
                "status": "rejected",
                "state": self._state.value,
                "message": "Communication lost. Heartbeat required before movement.",
            }
        return None

    def handle_move(self, message: dict) -> dict:
        """Handle a MOVE command with linear/angular velocities."""
        try:
            linear = self._validate_speed(
                message.get("linear", 0),
                "linear",
                self._config.max_linear_speed,
            )
            angular = self._validate_speed(
                message.get("angular", 0),
                "angular",
                self._config.max_angular_speed,
            )
        except ValueError as exc:
            return {
                "type": "ERROR",
                "status": "invalid",
                "message": str(exc),
            }

        with self._lock:
            blocked = self._check_can_move()
            if blocked:
                return blocked
            self._cancel_motion_timer()
            self._robot.move(linear, angular)
            self._state = (
                RobotState.MOVING
                if (linear != 0.0 or angular != 0.0)
                else RobotState.IDLE
            )
            return {
                "type": "MOVE_ACK",
                "state": self._state.value,
                "status": "ok",
                "linear": linear,
                "angular": angular,
            }

    def handle_forward(self, message: dict) -> dict:
        """Handle a FORWARD command with optional duration."""
        speed = message.get("speed", self._config.default_speed)
        try:
            speed = self._validate_speed(
                speed, "speed", self._config.max_linear_speed, allow_negative=False
            )
            duration = self._validate_duration(message.get("duration"))
        except ValueError as exc:
            return {"type": "ERROR", "status": "invalid", "message": str(exc)}

        with self._lock:
            blocked = self._check_can_move()
            if blocked:
                return blocked
            self._cancel_motion_timer()
            self._robot.forward(speed)
            self._state = RobotState.MOVING
            if duration is not None:
                self._schedule_motion_timer(duration)
            ack = {
                "type": "FORWARD_ACK",
                "state": self._state.value,
                "status": "ok",
                "speed": speed,
            }
            if duration is not None:
                ack["duration"] = duration
            return ack

    def handle_backward(self, message: dict) -> dict:
        """Handle a BACKWARD command with optional duration."""
        speed = message.get("speed", self._config.default_speed)
        try:
            speed = self._validate_speed(
                speed, "speed", self._config.max_linear_speed, allow_negative=False
            )
            duration = self._validate_duration(message.get("duration"))
        except ValueError as exc:
            return {"type": "ERROR", "status": "invalid", "message": str(exc)}

        with self._lock:
            blocked = self._check_can_move()
            if blocked:
                return blocked
            self._cancel_motion_timer()
            self._robot.backward(speed)
            self._state = RobotState.MOVING
            if duration is not None:
                self._schedule_motion_timer(duration)
            ack = {
                "type": "BACKWARD_ACK",
                "state": self._state.value,
                "status": "ok",
                "speed": speed,
            }
            if duration is not None:
                ack["duration"] = duration
            return ack

    def handle_turn_left(self, message: dict) -> dict:
        """Handle a TURN_LEFT command with optional duration."""
        speed = message.get("speed", self._config.default_turn_speed)
        try:
            speed = self._validate_speed(
                speed, "speed", self._config.max_angular_speed, allow_negative=False
            )
            duration = self._validate_duration(message.get("duration"))
        except ValueError as exc:
            return {"type": "ERROR", "status": "invalid", "message": str(exc)}

        with self._lock:
            blocked = self._check_can_move()
            if blocked:
                return blocked
            self._cancel_motion_timer()
            self._robot.turn_left(speed)
            self._state = RobotState.MOVING
            if duration is not None:
                self._schedule_motion_timer(duration)
            ack = {
                "type": "TURN_LEFT_ACK",
                "state": self._state.value,
                "status": "ok",
                "speed": speed,
            }
            if duration is not None:
                ack["duration"] = duration
            return ack

    def handle_turn_right(self, message: dict) -> dict:
        """Handle a TURN_RIGHT command with optional duration."""
        speed = message.get("speed", self._config.default_turn_speed)
        try:
            speed = self._validate_speed(
                speed, "speed", self._config.max_angular_speed, allow_negative=False
            )
            duration = self._validate_duration(message.get("duration"))
        except ValueError as exc:
            return {"type": "ERROR", "status": "invalid", "message": str(exc)}

        with self._lock:
            blocked = self._check_can_move()
            if blocked:
                return blocked
            self._cancel_motion_timer()
            self._robot.turn_right(speed)
            self._state = RobotState.MOVING
            if duration is not None:
                self._schedule_motion_timer(duration)
            ack = {
                "type": "TURN_RIGHT_ACK",
                "state": self._state.value,
                "status": "ok",
                "speed": speed,
            }
            if duration is not None:
                ack["duration"] = duration
            return ack

    def handle_goto(self, message: dict) -> dict:
        """Handle a GOTO command with x/y target coordinates."""
        try:
            x = self._validate_finite(message.get("x"), "x")
            y = self._validate_finite(message.get("y"), "y")
        except (ValueError, TypeError) as exc:
            return {"type": "ERROR", "status": "invalid", "message": str(exc)}

        if abs(x) > self._config.max_goto_range or abs(y) > self._config.max_goto_range:
            return {
                "type": "ERROR",
                "status": "invalid",
                "message": (
                    f"Target ({x}, {y}) exceeds range "
                    f"±{self._config.max_goto_range}"
                ),
            }

        with self._lock:
            blocked = self._check_can_move()
            if blocked:
                return blocked
            self._cancel_motion_timer()
            self._robot.goto(x, y)
            self._state = RobotState.MOVING
            return {
                "type": "GOTO_ACK",
                "state": self._state.value,
                "status": "ok",
                "target_x": x,
                "target_y": y,
            }

    def handle_stop(self) -> dict:
        """Handle a STOP command."""
        with self._lock:
            self._cancel_motion_timer()
            self._robot.stop()
            if self._state not in (RobotState.ESTOP_LATCHED, RobotState.COMM_LOST):
                self._state = RobotState.IDLE
            return {
                "type": "STOP_ACK",
                "state": self._state.value,
                "status": "ok",
            }

    def handle_status(self) -> dict:
        """Handle a STATUS query."""
        with self._lock:
            status: RobotStatus = self._robot.get_status()
            return {
                "type": "STATUS_RESPONSE",
                "state": self._state.value,
                "status": "ok",
                "x": status.x,
                "y": status.y,
                "heading_deg": status.heading_deg,
                "linear_velocity": status.linear_velocity,
                "angular_velocity": status.angular_velocity,
                "is_moving": status.is_moving,
            }

    # -----------------------------------------------------------------------
    # Command dispatcher
    # -----------------------------------------------------------------------

    def dispatch(self, message: dict) -> dict:
        """Route a validated protocol message to the appropriate handler.

        Parameters
        ----------
        message : dict
            A decoded message with a ``type`` field.

        Returns
        -------
        dict
            A response to send back to the client.
        """
        if not isinstance(message, dict):
            return {
                "type": "ERROR",
                "status": "invalid",
                "message": f"message must be a dict, got {type(message).__name__}",
            }

        raw_cmd = message.get("type")
        if not isinstance(raw_cmd, str) or not raw_cmd.strip():
            return {
                "type": "ERROR",
                "status": "invalid",
                "message": "message must contain a non-empty string 'type'",
            }

        cmd = raw_cmd.strip().upper()

        dispatch_table: dict[str, Callable[[], dict]] = {
            "HEARTBEAT":  lambda: self.record_heartbeat(),
            "MOVE":       lambda: self.handle_move(message),
            "FORWARD":    lambda: self.handle_forward(message),
            "BACKWARD":   lambda: self.handle_backward(message),
            "TURN_LEFT":  lambda: self.handle_turn_left(message),
            "TURN_RIGHT": lambda: self.handle_turn_right(message),
            "GOTO":       lambda: self.handle_goto(message),
            "STOP":       lambda: self.handle_stop(),
            "STATUS":     lambda: self.handle_status(),
            "ESTOP":      lambda: self.emergency_stop(),
            "RESET":      lambda: self.reset(),
            "PING":       lambda: {
                "type": "ACK",
                "received_type": "PING",
                "status": "ok",
                "state": self._state.value,
            },
        }

        handler = dispatch_table.get(cmd)
        if handler is None:
            return {
                "type": "ERROR",
                "status": "unknown_command",
                "message": f"Unknown command type: {message.get('type')}",
            }

        return handler()
