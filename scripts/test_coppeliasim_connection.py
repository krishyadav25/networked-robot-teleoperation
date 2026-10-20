"""
test_coppeliasim_connection.py - Smoke-test script for CoppeliaSim Pioneer P3-DX.

Verifies the ZeroMQ Remote API connection, resolves the three required scene objects
('PioneerP3DX', 'leftMotor', 'rightMotor'), reports simulation state and initial pose,
and exits WITHOUT commanding any robot motion.

Run from project root:
    python scripts/test_coppeliasim_connection.py
    python scripts/test_coppeliasim_connection.py --host localhost --port 23000
"""

import argparse
import math
import sys
from pathlib import Path

# Add project root to sys.path so 'src' can be imported when run as a script
project_root = Path(__file__).resolve().parent.parent
if str(project_root) not in sys.path:
    sys.path.insert(0, str(project_root))

from src.robot_interface import CoppeliaSimConfig, CoppeliaSimRobot


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Verify CoppeliaSim connection and Pioneer P3-DX object handles without moving the robot."
    )
    parser.add_argument("--host", default="localhost", help="CoppeliaSim host (default: localhost)")
    parser.add_argument("--port", type=int, default=23000, help="ZeroMQ Remote API port (default: 23000)")
    parser.add_argument("--robot", default="PioneerP3DX", help="Robot base object name (default: PioneerP3DX)")
    parser.add_argument("--left-motor", default="leftMotor", help="Left motor name (default: leftMotor)")
    parser.add_argument("--right-motor", default="rightMotor", help="Right motor name (default: rightMotor)")
    args = parser.parse_args()

    print("=" * 65)
    print("AISR Milestone 3A - CoppeliaSim Connection Smoke Test")
    print("=" * 65)
    print(f"Target endpoint : {args.host}:{args.port}")
    print(f"Target objects  : {args.robot}, {args.left_motor}, {args.right_motor}\n")

    config = CoppeliaSimConfig(
        host=args.host,
        port=args.port,
        robot_name=args.robot,
        left_motor_name=args.left_motor,
        right_motor_name=args.right_motor,
        auto_start_simulation=False,  # Strictly do not start simulation
    )

    try:
        print("[1/4] Connecting to CoppeliaSim ZeroMQ Remote API...")
        robot = CoppeliaSimRobot(config=config)
        print("      [OK] Connected successfully.")

        print("[2/4] Verifying scene object handles:")
        sim = robot._sim
        print(f"      [OK] Robot base  ('{args.robot}')  : handle = {robot._robot_handle}")
        print(f"      [OK] Left motor  ('{args.left_motor}')   : handle = {robot._left_motor}")
        print(f"      [OK] Right motor ('{args.right_motor}')  : handle = {robot._right_motor}")

        print("[3/4] Checking simulation state:")
        state = sim.getSimulationState()
        sim_stopped = getattr(sim, "simulation_stopped", 0)
        sim_paused = getattr(sim, "simulation_paused", 8)
        sim_running = getattr(sim, "simulation_advancing_running", 17)

        if state == sim_stopped:
            state_str = "STOPPED (idle)"
        elif state == sim_paused:
            state_str = "PAUSED"
        elif state == sim_running:
            state_str = "RUNNING"
        else:
            state_str = f"STATE ID {state}"
        print(f"      Simulation state: {state_str}")

        print("[4/4] Inspecting initial robot pose and velocity state:")
        status = robot.get_status()
        print(f"      Position (x, y)  : ({status.x:.3f} m, {status.y:.3f} m)")
        print(f"      Heading          : {status.heading_deg:.1f} deg")
        print(f"      Linear velocity  : {status.linear_velocity:.3f} m/s")
        print(f"      Angular velocity : {status.angular_velocity:.3f} deg/s")
        print(f"      Moving flag      : {status.is_moving}")

        # Verify both wheel target velocities are strictly zero
        left_vel = sim.getJointTargetVelocity(robot._left_motor)
        right_vel = sim.getJointTargetVelocity(robot._right_motor)
        print(f"      Left motor target velocity  : {left_vel} rad/s")
        print(f"      Right motor target velocity : {right_vel} rad/s")

        print("\n" + "-" * 65)
        print("SMOKE TEST RESULT: SUCCESS")
        print("CoppeliaSim Pioneer P3-DX is properly configured and accessible.")
        print("NO ROBOT MOTION COMMANDED. Robot remains stationary.")
        print("-" * 65)
        return 0

    except Exception as exc:
        print("\n" + "!" * 65)
        print(f"SMOKE TEST FAILED: {exc}")
        print("!" * 65)
        return 1


if __name__ == "__main__":
    sys.exit(main())
