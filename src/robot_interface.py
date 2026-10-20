"""
robot_interface.py – Robot abstraction layer for AISR.

Defines the abstract ``RobotInterface`` and a ``MockRobot`` backend that
records commands without real hardware.  The interface is designed so a
future CoppeliaSim Pioneer P3-DX backend can be swapped in.

Classes
-------
RobotInterface
    Abstract base class for robot backends.
MockRobot
    In-memory mock that logs commands for testing.
"""

from __future__ import annotations

import math
import threading
from abc import ABC, abstractmethod
from dataclasses import dataclass, field


# ---------------------------------------------------------------------------
# Data classes
# ---------------------------------------------------------------------------

@dataclass
class RobotStatus:
    """Snapshot of the robot's current state."""
    x: float = 0.0
    y: float = 0.0
    heading_deg: float = 0.0
    linear_velocity: float = 0.0
    angular_velocity: float = 0.0
    is_moving: bool = False


@dataclass
class CommandRecord:
    """One entry in the mock robot's command log."""
    method: str
    args: dict = field(default_factory=dict)


# ---------------------------------------------------------------------------
# Abstract interface
# ---------------------------------------------------------------------------

class RobotInterface(ABC):
    """Abstract interface for controlling a differential-drive robot.

    Every concrete backend must implement these methods.  The interface
    assumes velocities in m/s (linear) and deg/s (angular), matching a
    Pioneer P3-DX style robot.
    """

    @abstractmethod
    def move(self, linear: float, angular: float) -> None:
        """Set linear (m/s) and angular (deg/s) velocities."""

    @abstractmethod
    def forward(self, speed: float) -> None:
        """Move forward at *speed* m/s (positive)."""

    @abstractmethod
    def backward(self, speed: float) -> None:
        """Move backward at *speed* m/s (positive value, robot reverses)."""

    @abstractmethod
    def turn_left(self, speed: float) -> None:
        """Rotate left (counter-clockwise) at *speed* deg/s."""

    @abstractmethod
    def turn_right(self, speed: float) -> None:
        """Rotate right (clockwise) at *speed* deg/s."""

    @abstractmethod
    def goto(self, x: float, y: float) -> None:
        """Navigate to an (x, y) coordinate autonomously."""

    @abstractmethod
    def stop(self) -> None:
        """Stop all motion immediately."""

    @abstractmethod
    def get_status(self) -> RobotStatus:
        """Return the current robot status."""


# ---------------------------------------------------------------------------
# Mock implementation
# ---------------------------------------------------------------------------

class MockRobot(RobotInterface):
    """In-memory mock robot for local testing.

    Records every command in ``command_log`` and maintains simulated
    velocity / position state.  Thread-safe via a lock.

    This is **not** a physics simulation — it simply stores the last
    commanded velocities and target positions so that tests can verify
    the safety layer's behaviour.
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._status = RobotStatus()
        self.command_log: list[CommandRecord] = []

    # -- helpers --

    def _log(self, method: str, **kwargs: object) -> None:
        self.command_log.append(CommandRecord(method=method, args=kwargs))

    # -- interface implementation --

    def move(self, linear: float, angular: float) -> None:
        with self._lock:
            self._status.linear_velocity = linear
            self._status.angular_velocity = angular
            self._status.is_moving = (linear != 0.0 or angular != 0.0)
            self._log("move", linear=linear, angular=angular)

    def forward(self, speed: float) -> None:
        with self._lock:
            self._status.linear_velocity = abs(speed)
            self._status.angular_velocity = 0.0
            self._status.is_moving = True
            self._log("forward", speed=speed)

    def backward(self, speed: float) -> None:
        with self._lock:
            self._status.linear_velocity = -abs(speed)
            self._status.angular_velocity = 0.0
            self._status.is_moving = True
            self._log("backward", speed=speed)

    def turn_left(self, speed: float) -> None:
        with self._lock:
            self._status.linear_velocity = 0.0
            self._status.angular_velocity = abs(speed)
            self._status.is_moving = True
            self._log("turn_left", speed=speed)

    def turn_right(self, speed: float) -> None:
        with self._lock:
            self._status.linear_velocity = 0.0
            self._status.angular_velocity = -abs(speed)
            self._status.is_moving = True
            self._log("turn_right", speed=speed)

    def goto(self, x: float, y: float) -> None:
        with self._lock:
            self._status.is_moving = True
            self._log("goto", x=x, y=y)

    def stop(self) -> None:
        with self._lock:
            self._status.linear_velocity = 0.0
            self._status.angular_velocity = 0.0
            self._status.is_moving = False
            self._log("stop")

    def get_status(self) -> RobotStatus:
        with self._lock:
            # Return a copy so callers cannot mutate internal state.
            s = self._status
            return RobotStatus(
                x=s.x,
                y=s.y,
                heading_deg=s.heading_deg,
                linear_velocity=s.linear_velocity,
                angular_velocity=s.angular_velocity,
                is_moving=s.is_moving,
            )
