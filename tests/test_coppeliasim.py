"""
test_coppeliasim.py – Unit tests for CoppeliaSim Pioneer P3-DX backend.

Uses a FakeSim in-memory test double to verify object lookup, wheel velocity
kinematics, simulation state handling, zero-velocity stopping, configuration
validation, and backend factory selection without requiring a running CoppeliaSim.

Run with:
    python -m unittest discover -s tests -v
"""

import math
import os
import unittest
from unittest.mock import patch

from src.robot_interface import (
    CoppeliaSimConfig,
    CoppeliaSimRobot,
    MockRobot,
    RobotStatus,
    create_robot_backend,
)
from src.safety import RobotState, SafetyConfig, SafetyController


# ============================================================================
# FakeSim Test Double
# ============================================================================

class FakeSim:
    """Mock implementation of the CoppeliaSim remote API 'sim' object."""

    # Constants
    handle_world = -1
    handle_scene = 0
    handle_all = -1
    simulation_stopped = 0
    simulation_paused = 8
    simulation_advancing_running = 17
    modelproperty_scripts_inactive = 512

    def __init__(self) -> None:
        # Objects database: handle -> info
        self._objects: dict[int, dict] = {
            15: {
                "name": "/PioneerP3DX",
                "alias": "PioneerP3DX",
                "parent": 0,
                "position": [1.5, 2.5, 0.138],
                "orientation": [0.0, 0.0, math.radians(45.0)],
                "velocity": ([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]),
                "model_property": 0,
            },
            20: {
                "name": "/PioneerP3DX/leftMotor",
                "alias": "leftMotor",
                "parent": 15,
                "target_velocity": 0.0,
            },
            17: {
                "name": "/PioneerP3DX/rightMotor",
                "alias": "rightMotor",
                "parent": 15,
                "target_velocity": 0.0,
            },
        }
        self._next_handle = 100
        self._sim_state = self.simulation_advancing_running  # Running by default for motion tests

    # -- object lookup --

    def getObject(self, path: str, options: dict | None = None) -> int:
        proxy = options.get("proxy") if options else None

        # Check relative to proxy
        if proxy is not None and path.startswith("./"):
            rel_name = path[2:]
            for h, info in self._objects.items():
                if info.get("parent") == proxy and (info["alias"] == rel_name or info["name"].endswith("/" + rel_name)):
                    return h
            raise ValueError(f"Object {path} relative to {proxy} not found")

        # Check exact name or alias
        for h, info in self._objects.items():
            if info["name"] == path or info["alias"] == path:
                return h
            if path.startswith("/") and info["name"] == path:
                return h
            if not path.startswith("/") and info["name"] == "/" + path:
                return h

        raise ValueError(f"Object {path} not found")

    def getObjectAlias(self, handle: int) -> str:
        if handle in self._objects:
            return self._objects[handle]["alias"]
        raise ValueError(f"Invalid handle {handle}")

    def getObjectsInTree(self, root: int, obj_type: int, options: int) -> list[int]:
        return list(self._objects.keys())

    # -- properties & states --

    def getModelProperty(self, handle: int) -> int:
        if handle in self._objects:
            return self._objects[handle].get("model_property", 0)
        return 0

    def setModelProperty(self, handle: int, prop: int) -> None:
        if handle in self._objects:
            self._objects[handle]["model_property"] = prop

    def getSimulationState(self) -> int:
        return self._sim_state

    def startSimulation(self) -> None:
        self._sim_state = self.simulation_advancing_running

    def stopSimulation(self) -> None:
        self._sim_state = self.simulation_stopped

    def pauseSimulation(self) -> None:
        self._sim_state = self.simulation_paused

    # -- kinematics & joints --

    def setJointTargetVelocity(self, handle: int, velocity: float) -> None:
        if handle in self._objects:
            self._objects[handle]["target_velocity"] = velocity
        else:
            raise ValueError(f"Invalid joint handle {handle}")

    def getJointTargetVelocity(self, handle: int) -> float:
        if handle in self._objects:
            return self._objects[handle].get("target_velocity", 0.0)
        raise ValueError(f"Invalid joint handle {handle}")

    def getObjectPosition(self, handle: int, rel_to: int) -> list[float]:
        if handle in self._objects and "position" in self._objects[handle]:
            return list(self._objects[handle]["position"])
        return [0.0, 0.0, 0.0]

    def getObjectOrientation(self, handle: int, rel_to: int) -> list[float]:
        if handle in self._objects and "orientation" in self._objects[handle]:
            return list(self._objects[handle]["orientation"])
        return [0.0, 0.0, 0.0]

    def getObjectVelocity(self, handle: int) -> tuple[list[float], list[float]]:
        if handle in self._objects and "velocity" in self._objects[handle]:
            return self._objects[handle]["velocity"]
        return ([0.0, 0.0, 0.0], [0.0, 0.0, 0.0])


# ============================================================================
# 1. Object Resolution Tests
# ============================================================================

class TestCoppeliaSimObjectLookup(unittest.TestCase):
    """Test resolution of robot and joint handles."""

    def setUp(self) -> None:
        self.sim = FakeSim()

    def test_successful_object_resolution(self):
        config = CoppeliaSimConfig(
            robot_name="PioneerP3DX",
            left_motor_name="leftMotor",
            right_motor_name="rightMotor",
        )
        robot = CoppeliaSimRobot(config=config, sim=self.sim)
        self.assertEqual(robot._robot_handle, 15)
        self.assertEqual(robot._left_motor, 20)
        self.assertEqual(robot._right_motor, 17)

    def test_missing_robot_raises_value_error(self):
        config = CoppeliaSimConfig(robot_name="NonExistentRobot")
        with self.assertRaises(ValueError) as ctx:
            CoppeliaSimRobot(config=config, sim=self.sim)
        self.assertIn("NonExistentRobot", str(ctx.exception))

    def test_missing_left_motor_raises_value_error(self):
        config = CoppeliaSimConfig(left_motor_name="missingLeft")
        with self.assertRaises(ValueError) as ctx:
            CoppeliaSimRobot(config=config, sim=self.sim)
        self.assertIn("missingLeft", str(ctx.exception))

    def test_missing_right_motor_raises_value_error(self):
        config = CoppeliaSimConfig(right_motor_name="missingRight")
        with self.assertRaises(ValueError) as ctx:
            CoppeliaSimRobot(config=config, sim=self.sim)
        self.assertIn("missingRight", str(ctx.exception))

    def test_deactivates_child_scripts_on_init(self):
        config = CoppeliaSimConfig(deactivate_child_scripts=True)
        robot = CoppeliaSimRobot(config=config, sim=self.sim)
        prop = self.sim.getModelProperty(robot._robot_handle)
        self.assertTrue(bool(prop & self.sim.modelproperty_scripts_inactive))


# ============================================================================
# 2. Wheel Velocity Kinematics Tests
# ============================================================================

class TestWheelVelocityKinematics(unittest.TestCase):
    """Test differential drive wheel speed calculations."""

    def setUp(self) -> None:
        self.sim = FakeSim()
        # R = 0.0975 m, L = 0.331 m
        self.config = CoppeliaSimConfig(
            wheel_radius=0.0975,
            wheel_separation=0.331,
            max_wheel_speed=20.0,
        )
        self.robot = CoppeliaSimRobot(config=self.config, sim=self.sim)

    def test_pure_forward_motion(self):
        """v = 1.0 m/s, omega = 0 -> both wheels omega = 1.0 / 0.0975 rad/s."""
        w_left, w_right = self.robot.compute_wheel_velocities(1.0, 0.0)
        expected = 1.0 / 0.0975
        self.assertAlmostEqual(w_left, expected, places=5)
        self.assertAlmostEqual(w_right, expected, places=5)
        self.assertGreater(w_left, 0)
        self.assertGreater(w_right, 0)

    def test_pure_backward_motion(self):
        """v = -0.5 m/s, omega = 0 -> both wheels negative."""
        w_left, w_right = self.robot.compute_wheel_velocities(-0.5, 0.0)
        expected = -0.5 / 0.0975
        self.assertAlmostEqual(w_left, expected, places=5)
        self.assertAlmostEqual(w_right, expected, places=5)
        self.assertLess(w_left, 0)
        self.assertLess(w_right, 0)

    def test_turn_left(self):
        """omega = +45 deg/s (CCW) -> left wheel negative, right wheel positive."""
        w_left, w_right = self.robot.compute_wheel_velocities(0.0, 45.0)
        self.assertLess(w_left, 0)
        self.assertGreater(w_right, 0)
        self.assertAlmostEqual(abs(w_left), abs(w_right), places=5)

    def test_turn_right(self):
        """omega = -45 deg/s (CW) -> left wheel positive, right wheel negative."""
        w_left, w_right = self.robot.compute_wheel_velocities(0.0, -45.0)
        self.assertGreater(w_left, 0)
        self.assertLess(w_right, 0)
        self.assertAlmostEqual(abs(w_left), abs(w_right), places=5)

    def test_combined_linear_and_angular(self):
        """v = 1.0 m/s, omega = 30 deg/s -> right wheel faster than left."""
        w_left, w_right = self.robot.compute_wheel_velocities(1.0, 30.0)
        self.assertGreater(w_right, w_left)

    def test_velocity_saturation_clamping(self):
        """Exceeding max_wheel_speed scales velocities proportionally."""
        # High speed that would exceed 20 rad/s
        w_left, w_right = self.robot.compute_wheel_velocities(3.0, 45.0)
        max_speed = max(abs(w_left), abs(w_right))
        self.assertAlmostEqual(max_speed, 20.0, places=5)
        # Ratio of left to right should be preserved
        ang_rad = math.radians(45.0)
        half_l = 0.331 / 2.0
        v_l_ideal = 3.0 - (ang_rad * half_l)
        v_r_ideal = 3.0 + (ang_rad * half_l)
        ideal_ratio = v_l_ideal / v_r_ideal
        self.assertAlmostEqual(w_left / w_right, ideal_ratio, places=5)


# ============================================================================
# 3. Robot Commands & Stop Operation
# ============================================================================

class TestRobotMotionAndStop(unittest.TestCase):
    """Test execution of motion commands on CoppeliaSim joints."""

    def setUp(self) -> None:
        self.sim = FakeSim()
        self.robot = CoppeliaSimRobot(sim=self.sim)

    def test_forward_sets_joint_velocities(self):
        self.robot.forward(1.0)
        w_left = self.sim.getJointTargetVelocity(self.robot._left_motor)
        w_right = self.sim.getJointTargetVelocity(self.robot._right_motor)
        self.assertGreater(w_left, 0)
        self.assertGreater(w_right, 0)
        self.assertTrue(self.robot.get_status().is_moving)

    def test_backward_sets_negative_velocities(self):
        self.robot.backward(0.5)
        w_left = self.sim.getJointTargetVelocity(self.robot._left_motor)
        w_right = self.sim.getJointTargetVelocity(self.robot._right_motor)
        self.assertLess(w_left, 0)
        self.assertLess(w_right, 0)

    def test_turn_left_sets_opposing_velocities(self):
        self.robot.turn_left(30.0)
        w_left = self.sim.getJointTargetVelocity(self.robot._left_motor)
        w_right = self.sim.getJointTargetVelocity(self.robot._right_motor)
        self.assertLess(w_left, 0)
        self.assertGreater(w_right, 0)

    def test_turn_right_sets_opposing_velocities(self):
        self.robot.turn_right(30.0)
        w_left = self.sim.getJointTargetVelocity(self.robot._left_motor)
        w_right = self.sim.getJointTargetVelocity(self.robot._right_motor)
        self.assertGreater(w_left, 0)
        self.assertLess(w_right, 0)

    def test_stop_commands_zero_velocity(self):
        """stop() must command both wheel motors to 0.0 and clear moving flag."""
        self.robot.forward(1.0)
        self.robot.stop()
        w_left = self.sim.getJointTargetVelocity(self.robot._left_motor)
        w_right = self.sim.getJointTargetVelocity(self.robot._right_motor)
        self.assertEqual(w_left, 0.0)
        self.assertEqual(w_right, 0.0)
        status = self.robot.get_status()
        self.assertFalse(status.is_moving)
        self.assertEqual(status.linear_velocity, 0.0)

    def test_stop_does_not_change_simulation_state(self):
        """stop() should stop the wheels, not stop or pause the simulator."""
        initial_state = self.sim.getSimulationState()
        self.robot.forward(1.0)
        self.robot.stop()
        self.assertEqual(self.sim.getSimulationState(), initial_state)

    def test_goto_records_target(self):
        self.robot.goto(5.0, 10.0)
        self.assertEqual(self.robot._target_pos, (5.0, 10.0))
        self.assertTrue(self.robot._status.is_moving)


# ============================================================================
# 4. Simulation State Handling
# ============================================================================

class TestSimulationStateHandling(unittest.TestCase):
    """Test behavior when simulation is stopped, paused, or auto-started."""

    def test_motion_fails_when_simulation_stopped(self):
        sim = FakeSim()
        sim.stopSimulation()
        config = CoppeliaSimConfig(auto_start_simulation=False)
        robot = CoppeliaSimRobot(config=config, sim=sim)

        with self.assertRaises(RuntimeError) as ctx:
            robot.forward(1.0)
        self.assertIn("stopped", str(ctx.exception).lower())

    def test_motion_fails_when_simulation_paused(self):
        sim = FakeSim()
        sim.pauseSimulation()
        config = CoppeliaSimConfig()
        robot = CoppeliaSimRobot(config=config, sim=sim)

        with self.assertRaises(RuntimeError) as ctx:
            robot.forward(1.0)
        self.assertIn("paused", str(ctx.exception).lower())

    def test_auto_start_simulation_when_configured(self):
        sim = FakeSim()
        sim.stopSimulation()
        config = CoppeliaSimConfig(auto_start_simulation=True)
        robot = CoppeliaSimRobot(config=config, sim=sim)

        # Commanding motion should automatically start simulation
        robot.forward(1.0)
        self.assertEqual(sim.getSimulationState(), sim.simulation_advancing_running)
        self.assertGreater(sim.getJointTargetVelocity(robot._left_motor), 0)

    def test_stop_succeeds_even_when_simulation_stopped(self):
        sim = FakeSim()
        sim.stopSimulation()
        robot = CoppeliaSimRobot(sim=sim)
        # stop() should succeed without error
        robot.stop()
        self.assertEqual(sim.getJointTargetVelocity(robot._left_motor), 0.0)


# ============================================================================
# 5. Configuration Validation
# ============================================================================

class TestCoppeliaSimConfigValidation(unittest.TestCase):
    """Test validation of CoppeliaSimConfig parameters."""

    def test_negative_wheel_radius_raises(self):
        with self.assertRaises(ValueError):
            CoppeliaSimConfig(wheel_radius=-0.1)

    def test_zero_wheel_separation_raises(self):
        with self.assertRaises(ValueError):
            CoppeliaSimConfig(wheel_separation=0.0)

    def test_negative_max_wheel_speed_raises(self):
        with self.assertRaises(ValueError):
            CoppeliaSimConfig(max_wheel_speed=-5.0)

    def test_invalid_port_raises(self):
        with self.assertRaises(ValueError):
            CoppeliaSimConfig(port=99999)


# ============================================================================
# 6. Backend Factory Selection
# ============================================================================

class TestBackendFactory(unittest.TestCase):
    """Test create_robot_backend selection and error handling."""

    def test_mock_backend_by_default(self):
        with patch.dict(os.environ, {}, clear=True):
            robot = create_robot_backend()
            self.assertIsInstance(robot, MockRobot)

    def test_explicit_mock_backend(self):
        robot = create_robot_backend("mock")
        self.assertIsInstance(robot, MockRobot)

    def test_case_insensitive_mock(self):
        robot = create_robot_backend("MoCk")
        self.assertIsInstance(robot, MockRobot)

    def test_coppeliasim_backend_with_injected_sim(self):
        sim = FakeSim()
        robot = create_robot_backend("coppeliasim", sim=sim)
        self.assertIsInstance(robot, CoppeliaSimRobot)

    def test_unknown_backend_raises_value_error(self):
        with self.assertRaises(ValueError) as ctx:
            create_robot_backend("gazebo")
        self.assertIn("gazebo", str(ctx.exception))

    def test_coppeliasim_unreachable_never_falls_back_to_mock(self):
        """When CoppeliaSim is explicitly requested but unreachable, fail loudly."""
        # Use an unusable port to guarantee connection refusal
        config = CoppeliaSimConfig(port=23998, connect_timeout=0.1)
        with self.assertRaises((ConnectionError, ImportError)):
            create_robot_backend("coppeliasim", config=config)


# ============================================================================
# 7. Status Reporting
# ============================================================================

class TestStatusReporting(unittest.TestCase):
    """Test get_status() with live simulated pose and velocity."""

    def test_status_returns_simulated_pose(self):
        sim = FakeSim()
        robot = CoppeliaSimRobot(sim=sim)
        status = robot.get_status()
        self.assertIsInstance(status, RobotStatus)
        self.assertAlmostEqual(status.x, 1.5, places=2)
        self.assertAlmostEqual(status.y, 2.5, places=2)
        self.assertAlmostEqual(status.heading_deg, 45.0, places=1)

    def test_status_stopped_simulation_reports_zero_velocity(self):
        sim = FakeSim()
        sim._sim_state = FakeSim.simulation_stopped
        # Even if object had leftover velocity in sim, a stopped sim must report zero
        sim._objects[15]["velocity"] = ([0.5, 0.0, 0.0], [0.0, 0.0, 1.0])
        robot = CoppeliaSimRobot(sim=sim)
        status = robot.get_status()
        self.assertEqual(status.linear_velocity, 0.0)
        self.assertEqual(status.angular_velocity, 0.0)
        self.assertFalse(status.is_moving)

    def test_status_paused_simulation_reports_zero_velocity(self):
        sim = FakeSim()
        sim._sim_state = FakeSim.simulation_paused
        sim._objects[15]["velocity"] = ([0.5, 0.0, 0.0], [0.0, 0.0, 1.0])
        robot = CoppeliaSimRobot(sim=sim)
        status = robot.get_status()
        self.assertEqual(status.linear_velocity, 0.0)
        self.assertEqual(status.angular_velocity, 0.0)
        self.assertFalse(status.is_moving)

    def test_status_eliminates_negative_zero(self):
        sim = FakeSim()
        sim._sim_state = FakeSim.simulation_advancing_running
        # Simulate -0.0 angular velocity and position
        sim._objects[15]["velocity"] = ([0.0, -0.0, 0.0], [0.0, 0.0, -0.0])
        sim._objects[15]["position"] = [-0.0, -0.0, 0.0]
        sim._objects[15]["orientation"] = [0.0, 0.0, -0.0]
        robot = CoppeliaSimRobot(sim=sim)
        status = robot.get_status()

        self.assertEqual(status.linear_velocity, 0.0)
        self.assertEqual(status.angular_velocity, 0.0)
        # Verify it's not IEEE -0.0
        self.assertNotEqual(str(status.angular_velocity), "-0.0")
        self.assertNotEqual(str(status.linear_velocity), "-0.0")
        self.assertNotEqual(str(status.x), "-0.0")
        self.assertNotEqual(str(status.y), "-0.0")
        self.assertNotEqual(str(status.heading_deg), "-0.0")
        self.assertFalse(status.is_moving)

    def test_status_deadband_filters_tiny_jitter(self):
        sim = FakeSim()
        sim._sim_state = FakeSim.simulation_advancing_running
        # Tiny velocities below deadband (0.005 m/s, 0.1 deg/s)
        sim._objects[15]["velocity"] = ([0.002, 0.0, 0.0], [0.0, 0.0, math.radians(0.05)])
        robot = CoppeliaSimRobot(sim=sim)
        status = robot.get_status()
        self.assertEqual(status.linear_velocity, 0.0)
        self.assertEqual(status.angular_velocity, 0.0)
        self.assertFalse(status.is_moving)

    def test_status_reports_physical_velocity_when_moving(self):
        sim = FakeSim()
        sim._sim_state = FakeSim.simulation_advancing_running
        sim._objects[15]["velocity"] = ([0.2, 0.0, 0.0], [0.0, 0.0, math.radians(15.0)])
        robot = CoppeliaSimRobot(sim=sim)
        status = robot.get_status()
        self.assertAlmostEqual(status.linear_velocity, 0.2, places=2)
        self.assertAlmostEqual(status.angular_velocity, 15.0, places=1)
        self.assertTrue(status.is_moving)


# ============================================================================
# 8. Safety Controller Integration with CoppeliaSimRobot
# ============================================================================

class TestSafetyControllerWithCoppeliaSim(unittest.TestCase):
    """Verify that SafetyController commands CoppeliaSimRobot safely."""

    def setUp(self) -> None:
        self.sim = FakeSim()
        self.robot = CoppeliaSimRobot(sim=self.sim)
        self.ctrl = SafetyController(self.robot)

    def test_safety_forward_moves_sim_robot(self):
        resp = self.ctrl.dispatch({"type": "FORWARD", "speed": 1.0})
        self.assertEqual(resp["status"], "ok")
        self.assertEqual(self.ctrl.state, RobotState.MOVING)
        w_left = self.sim.getJointTargetVelocity(self.robot._left_motor)
        self.assertGreater(w_left, 0)

    def test_safety_estop_stops_sim_robot(self):
        self.ctrl.dispatch({"type": "FORWARD", "speed": 1.0})
        resp = self.ctrl.dispatch({"type": "ESTOP"})
        self.assertEqual(resp["type"], "ESTOP_ACK")
        self.assertEqual(self.ctrl.state, RobotState.ESTOP_LATCHED)
        # Motors must be zero
        w_left = self.sim.getJointTargetVelocity(self.robot._left_motor)
        w_right = self.sim.getJointTargetVelocity(self.robot._right_motor)
        self.assertEqual(w_left, 0.0)
        self.assertEqual(w_right, 0.0)

    def test_safety_reset_keeps_motors_zero(self):
        self.ctrl.dispatch({"type": "FORWARD", "speed": 1.0})
        self.ctrl.dispatch({"type": "ESTOP"})
        self.ctrl.dispatch({"type": "RESET"})
        self.assertEqual(self.ctrl.state, RobotState.IDLE)
        w_left = self.sim.getJointTargetVelocity(self.robot._left_motor)
        self.assertEqual(w_left, 0.0)

    def test_safety_disconnect_stops_sim_robot(self):
        self.ctrl.dispatch({"type": "FORWARD", "speed": 1.0})
        self.ctrl.on_client_disconnect()
        self.assertEqual(self.ctrl.state, RobotState.COMM_LOST)
        w_left = self.sim.getJointTargetVelocity(self.robot._left_motor)
        w_right = self.sim.getJointTargetVelocity(self.robot._right_motor)
        self.assertEqual(w_left, 0.0)
        self.assertEqual(w_right, 0.0)


if __name__ == "__main__":
    unittest.main()
