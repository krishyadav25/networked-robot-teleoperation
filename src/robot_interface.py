"""
robot_interface.py – Robot abstraction layer for AISR.

Defines the abstract ``RobotInterface``, an in-memory ``MockRobot`` backend
for deterministic testing, and a real ``CoppeliaSimRobot`` backend for
controlling a Pioneer P3-DX differential-drive mobile robot in CoppeliaSim.

Classes
-------
RobotInterface
    Abstract base class for robot backends.
MockRobot
    In-memory mock that logs commands for testing.
CoppeliaSimConfig
    Configuration parameters for the CoppeliaSim Pioneer P3-DX backend.
CoppeliaSimRobot
    Concrete backend interfacing with CoppeliaSim via the ZeroMQ Remote API.

Functions
---------
create_robot_backend(backend: str | None = None, ...) -> RobotInterface
    Factory function selecting backend based on explicit configuration.
"""

from __future__ import annotations

import math
import os
import socket
import threading
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Any, Optional

try:
    from coppeliasim_zmqremoteapi_client import RemoteAPIClient
except ImportError:
    RemoteAPIClient = None  # type: ignore[assignment,misc]


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

    @staticmethod
    def _clean_zero(val: float, threshold: float = 1e-6) -> float:
        """Eliminate IEEE -0.0 and noise below threshold."""
        return 0.0 if abs(val) < threshold else float(val)

    def get_status(self) -> RobotStatus:
        with self._lock:
            # Return a copy so callers cannot mutate internal state.
            s = self._status
            return RobotStatus(
                x=self._clean_zero(s.x),
                y=self._clean_zero(s.y),
                heading_deg=self._clean_zero(s.heading_deg),
                linear_velocity=self._clean_zero(s.linear_velocity),
                angular_velocity=self._clean_zero(s.angular_velocity),
                is_moving=s.is_moving,
            )


# ---------------------------------------------------------------------------
# CoppeliaSim Pioneer P3-DX backend
# ---------------------------------------------------------------------------

@dataclass
class CoppeliaSimConfig:
    """Configuration parameters for the CoppeliaSim Pioneer P3-DX backend."""
    host: str = "localhost"
    port: int = 23000
    connect_timeout: float = 2.0          # seconds for initial reachability check
    robot_name: str = "PioneerP3DX"       # Name or path of base robot object
    left_motor_name: str = "leftMotor"    # Name or path of left wheel motor joint
    right_motor_name: str = "rightMotor"  # Name or path of right wheel motor joint
    wheel_radius: float = 0.0975          # metres (Pioneer P3-DX 195mm diameter wheels)
    wheel_separation: float = 0.331       # metres (track width / separation: ~331mm)
    max_wheel_speed: float = 20.0         # rad/s max motor speed limit
    auto_start_simulation: bool = False   # Auto-start simulation when motion is commanded
    deactivate_child_scripts: bool = True # Inactivate default Braitenberg obstacle avoidance script

    def __post_init__(self) -> None:
        if self.wheel_radius <= 0:
            raise ValueError(f"wheel_radius must be positive, got {self.wheel_radius}")
        if self.wheel_separation <= 0:
            raise ValueError(f"wheel_separation must be positive, got {self.wheel_separation}")
        if self.max_wheel_speed <= 0:
            raise ValueError(f"max_wheel_speed must be positive, got {self.max_wheel_speed}")
        if not (1 <= self.port <= 65535):
            raise ValueError(f"port must be between 1 and 65535, got {self.port}")


class CoppeliaSimRobot(RobotInterface):
    """Concrete differential-drive robot backend connected to CoppeliaSim.

    Communicates via the ZeroMQ Remote API (port 23000 by default) to control
    a Pioneer P3-DX robot's drive motors.

    Parameters
    ----------
    config : CoppeliaSimConfig, optional
        Kinematic dimensions, network endpoints, and operational settings.
    client : Any, optional
        Pre-existing RemoteAPIClient instance. If None, creates a new one.
    sim : Any, optional
        Pre-existing sim API object. Useful for unit tests with a mock sim.
    """

    def __init__(
        self,
        config: CoppeliaSimConfig | None = None,
        client: Any = None,
        sim: Any = None,
    ) -> None:
        self._config = config or CoppeliaSimConfig()
        self._lock = threading.Lock()
        self._status = RobotStatus()
        self._target_pos: tuple[float, float] | None = None

        # Connect and obtain sim API object
        if sim is not None:
            self._client = client
            self._sim = sim
        else:
            self._client, self._sim = self._connect()

        # Resolve robot base and motor joints
        self._robot_handle = self._resolve_object(self._config.robot_name)
        self._left_motor = self._resolve_object(
            self._config.left_motor_name, parent_handle=self._robot_handle
        )
        self._right_motor = self._resolve_object(
            self._config.right_motor_name, parent_handle=self._robot_handle
        )

        # Deactivate child scripts on Pioneer model if requested so default
        # Braitenberg obstacle avoidance script does not overwrite motor velocities
        if self._config.deactivate_child_scripts:
            self._deactivate_model_scripts()

        # Initialize motors to zero velocity
        self.stop()

    # -- connection & resolution helpers --

    def _connect(self) -> tuple[Any, Any]:
        """Establish connection to CoppeliaSim ZeroMQ Remote API."""
        if RemoteAPIClient is None:
            raise ImportError(
                "The 'coppeliasim_zmqremoteapi_client' package is required to connect to CoppeliaSim. "
                "Install it using: pip install coppeliasim-zmqremoteapi-client"
            )

        # 1. Quick TCP socket probe to verify CoppeliaSim is listening
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
                probe.settimeout(self._config.connect_timeout)
                probe.connect((self._config.host, self._config.port))
        except (ConnectionRefusedError, socket.timeout, OSError) as exc:
            raise ConnectionError(
                f"Could not connect to CoppeliaSim at {self._config.host}:{self._config.port}. "
                f"Please verify that CoppeliaSim is running and the ZeroMQ Remote API is enabled "
                f"(listening on port {self._config.port}). Error: {exc}"
            ) from exc

        # 2. Instantiate RemoteAPIClient
        try:
            client = RemoteAPIClient(self._config.host, self._config.port)
            if hasattr(client, "timeout"):
                client.timeout = 5  # 5 second timeout on remote calls
            sim = client.getObject("sim")
            return client, sim
        except Exception as exc:
            raise ConnectionError(
                f"Failed to initialize CoppeliaSim RemoteAPIClient at "
                f"{self._config.host}:{self._config.port}: {exc}"
            ) from exc

    def _resolve_object(self, name: str, parent_handle: int | None = None) -> int:
        """Resolve an object handle in CoppeliaSim by name or path.

        Parameters
        ----------
        name : str
            The object name or path (e.g. 'PioneerP3DX', '/PioneerP3DX', 'leftMotor').
        parent_handle : int, optional
            Parent object handle to search relative to.

        Returns
        -------
        int
            The resolved CoppeliaSim object handle.

        Raises
        ------
        ValueError
            If the object cannot be found in the scene.
        """
        # 1. Try exact query as provided
        try:
            return self._sim.getObject(name)
        except Exception:
            pass

        # 2. Try with leading slash (scene root path)
        if not name.startswith("/"):
            try:
                return self._sim.getObject("/" + name)
            except Exception:
                pass

        # 3. Try relative to parent if provided
        if parent_handle is not None:
            try:
                return self._sim.getObject("./" + name, {"proxy": parent_handle})
            except Exception:
                pass

        # 4. Search all objects in tree by alias
        try:
            tree_root = getattr(self._sim, "handle_scene", 0)
            all_types = getattr(self._sim, "handle_all", -1)
            for h in self._sim.getObjectsInTree(tree_root, all_types, 0):
                try:
                    if self._sim.getObjectAlias(h) == name:
                        return h
                except Exception:
                    pass
        except Exception:
            pass

        parent_info = f" under parent handle {parent_handle}" if parent_handle is not None else ""
        raise ValueError(
            f"Object '{name}'{parent_info} not found in CoppeliaSim scene. "
            f"Please verify that the object exists in the scene hierarchy."
        )

    def _deactivate_model_scripts(self) -> None:
        """Deactivate model child scripts so they do not fight commanded velocities."""
        try:
            prop_flag = getattr(self._sim, "modelproperty_scripts_inactive", 512)
            current_prop = self._sim.getModelProperty(self._robot_handle)
            self._sim.setModelProperty(self._robot_handle, current_prop | prop_flag)
        except Exception:
            # If getModelProperty or setModelProperty fails or is mock, pass silently
            pass

    def _check_simulation_running(self) -> None:
        """Verify simulator is running before commanding motion."""
        try:
            state = self._sim.getSimulationState()
        except Exception as exc:
            raise RuntimeError(f"Failed to query CoppeliaSim simulation state: {exc}") from exc

        sim_stopped = getattr(self._sim, "simulation_stopped", 0)
        sim_paused = getattr(self._sim, "simulation_paused", 8)

        if state == sim_stopped:
            if self._config.auto_start_simulation:
                print("[CoppeliaSimRobot] Simulation was stopped. Starting simulation (sim.startSimulation())...")
                self._sim.startSimulation()
            else:
                raise RuntimeError(
                    "CoppeliaSim simulation is stopped. Please start the simulation in CoppeliaSim "
                    "before commanding motion, or configure auto_start_simulation=True."
                )
        elif state == sim_paused:
            raise RuntimeError(
                "CoppeliaSim simulation is paused. Please unpause the simulation in CoppeliaSim "
                "before commanding motion."
            )

    # -- kinematics calculations --

    def compute_wheel_velocities(self, linear: float, angular_deg: float) -> tuple[float, float]:
        """Convert linear (m/s) and angular (deg/s) velocities to wheel rad/s.

        Kinematics formula for differential drive:
            v_left  = linear - (angular_rad * L / 2)
            v_right = linear + (angular_rad * L / 2)
            omega_left  = v_left / R
            omega_right = v_right / R

        Parameters
        ----------
        linear : float
            Linear forward velocity in m/s (negative for reverse).
        angular_deg : float
            Angular velocity in deg/s (positive = left / CCW, negative = right / CW).

        Returns
        -------
        tuple[float, float]
            ``(omega_left, omega_right)`` in radians per second.
        """
        angular_rad = math.radians(angular_deg)
        half_track = self._config.wheel_separation / 2.0

        v_left = linear - (angular_rad * half_track)
        v_right = linear + (angular_rad * half_track)

        omega_left = v_left / self._config.wheel_radius
        omega_right = v_right / self._config.wheel_radius

        # Scale down proportionally if either wheel exceeds max_wheel_speed
        max_req = max(abs(omega_left), abs(omega_right))
        if max_req > self._config.max_wheel_speed:
            scale = self._config.max_wheel_speed / max_req
            omega_left *= scale
            omega_right *= scale

        return omega_left, omega_right

    def _apply_wheel_velocities(self, omega_left: float, omega_right: float) -> None:
        """Send target velocities to CoppeliaSim joints."""
        try:
            self._sim.setJointTargetVelocity(self._left_motor, float(omega_left))
            self._sim.setJointTargetVelocity(self._right_motor, float(omega_right))
        except Exception as exc:
            raise RuntimeError(f"Failed to set wheel velocities in CoppeliaSim: {exc}") from exc

    # -- RobotInterface implementation --

    def move(self, linear: float, angular: float) -> None:
        with self._lock:
            # If commanding complete stop (0, 0), apply zero velocity without requiring running sim
            if linear == 0.0 and angular == 0.0:
                self._apply_wheel_velocities(0.0, 0.0)
                self._status.linear_velocity = 0.0
                self._status.angular_velocity = 0.0
                self._status.is_moving = False
                return

            self._check_simulation_running()
            omega_left, omega_right = self.compute_wheel_velocities(linear, angular)
            self._apply_wheel_velocities(omega_left, omega_right)
            self._status.linear_velocity = linear
            self._status.angular_velocity = angular
            self._status.is_moving = True

    def forward(self, speed: float) -> None:
        self.move(abs(speed), 0.0)

    def backward(self, speed: float) -> None:
        self.move(-abs(speed), 0.0)

    def turn_left(self, speed: float) -> None:
        self.move(0.0, abs(speed))

    def turn_right(self, speed: float) -> None:
        self.move(0.0, -abs(speed))

    def goto(self, x: float, y: float) -> None:
        """Set autonomous navigation target coordinates.

        Note: Records target coordinates in the robot status. Closed-loop
        trajectory planning and obstacle navigation are deferred to
        Milestone 4. Does not claim accurate closed-loop GOTO.
        """
        with self._lock:
            self._target_pos = (x, y)
            self._status.is_moving = True

    def stop(self) -> None:
        """Stop both wheel motors immediately by commanding 0.0 velocity."""
        with self._lock:
            self._apply_wheel_velocities(0.0, 0.0)
            self._status.linear_velocity = 0.0
            self._status.angular_velocity = 0.0
            self._status.is_moving = False

    @staticmethod
    def _clean_zero(val: float, threshold: float = 1e-6) -> float:
        """Eliminate IEEE -0.0 and noise below threshold."""
        return 0.0 if abs(val) < threshold else float(val)

    def get_status(self) -> RobotStatus:
        """Query live robot status from CoppeliaSim."""
        with self._lock:
            if self._sim is not None and self._robot_handle is not None:
                try:
                    # Check whether simulation is actively running
                    sim_state = self._sim.getSimulationState()
                    sim_stopped = getattr(self._sim, "simulation_stopped", 0)
                    sim_paused = getattr(self._sim, "simulation_paused", 8)
                    is_sim_active = (sim_state != sim_stopped and sim_state != sim_paused)

                    world_ref = getattr(self._sim, "handle_world", -1)
                    pos = self._sim.getObjectPosition(self._robot_handle, world_ref)
                    euler = self._sim.getObjectOrientation(self._robot_handle, world_ref)

                    heading_deg = math.degrees(euler[2])
                    # Normalize heading to [-180, 180]
                    heading_deg = (heading_deg + 180.0) % 360.0 - 180.0

                    if is_sim_active:
                        vel = self._sim.getObjectVelocity(self._robot_handle)
                        lin_vel = math.hypot(vel[0][0], vel[0][1])
                        ang_vel_deg = math.degrees(vel[1][2])
                    else:
                        lin_vel = 0.0
                        ang_vel_deg = 0.0

                    # Clean values to avoid misleading -0.0 or deadband jitter
                    lin_vel = self._clean_zero(lin_vel, threshold=0.005)
                    ang_vel_deg = self._clean_zero(ang_vel_deg, threshold=0.1)
                    pos_x = self._clean_zero(pos[0])
                    pos_y = self._clean_zero(pos[1])
                    heading_deg = self._clean_zero(heading_deg)

                    is_moving = is_sim_active and (
                        self._status.is_moving or (lin_vel > 0.0 or abs(ang_vel_deg) > 0.0)
                    )

                    return RobotStatus(
                        x=pos_x,
                        y=pos_y,
                        heading_deg=heading_deg,
                        linear_velocity=lin_vel if is_moving else 0.0,
                        angular_velocity=ang_vel_deg if is_moving else 0.0,
                        is_moving=is_moving,
                    )
                except Exception:
                    # Fall back to internal state if remote read fails temporarily
                    pass

            return RobotStatus(
                x=self._clean_zero(self._status.x),
                y=self._clean_zero(self._status.y),
                heading_deg=self._clean_zero(self._status.heading_deg),
                linear_velocity=self._clean_zero(self._status.linear_velocity),
                angular_velocity=self._clean_zero(self._status.angular_velocity),
                is_moving=self._status.is_moving,
            )


# ---------------------------------------------------------------------------
# Backend factory
# ---------------------------------------------------------------------------

def create_robot_backend(
    backend: str | None = None,
    config: CoppeliaSimConfig | None = None,
    client: Any = None,
    sim: Any = None,
) -> RobotInterface:
    """Create a robot backend based on explicit configuration.

    Parameters
    ----------
    backend : str, optional
        "mock" or "coppeliasim". Defaults to os.environ.get("AISR_ROBOT_BACKEND", "mock").
    config : CoppeliaSimConfig, optional
        Configuration settings for CoppeliaSimRobot.
    client : Any, optional
        Pre-existing RemoteAPIClient (useful for testing).
    sim : Any, optional
        Pre-existing sim object (useful for unit tests with a mock sim).

    Returns
    -------
    RobotInterface
        The configured backend instance.

    Raises
    ------
    ValueError
        If backend name is invalid.
    ConnectionError / RuntimeError
        If CoppeliaSim was explicitly requested but could not connect or resolve objects.
        Never silently falls back to MockRobot when CoppeliaSim was requested.
    """
    if backend is None:
        backend = os.environ.get("AISR_ROBOT_BACKEND", "mock")

    backend_clean = backend.strip().lower()
    if backend_clean == "mock":
        return MockRobot()
    elif backend_clean in ("coppeliasim", "coppelia"):
        return CoppeliaSimRobot(config=config, client=client, sim=sim)
    else:
        raise ValueError(
            f"Unknown robot backend: '{backend}'. Valid options are 'mock' or 'coppeliasim'."
        )
