# CoppeliaSim Pioneer P3-DX Integration Guide

This guide describes the integration of the Pioneer P3-DX differential-drive robot in CoppeliaSim with the AISR teleoperation system (Milestone 3A).

---

## 1. Overview & Kinematic Model

The AISR teleoperation system supports two swappable robot backends implementing the common `RobotInterface`:
1. **`MockRobot`**: In-memory test double logging all commands for deterministic unit testing.
2. **`CoppeliaSimRobot`**: Concrete hardware-in-the-loop simulation backend communicating with CoppeliaSim via the ZeroMQ Remote API (`coppeliasim_zmqremoteapi_client`).

### Pioneer P3-DX Differential-Drive Kinematics

The Pioneer P3-DX mobile robot is modeled with two actuated drive wheels and a passive rear caster wheel.

- **Wheel Radius ($R$):** `0.0975 m` ($195\text{ mm}$ diameter).
- **Wheel Separation / Track Width ($L$):** `0.331 m` ($331\text{ mm}$ center-to-center distance between wheels).
- **Max Wheel Angular Velocity:** `20.0 rad/s` ($\approx 1.95\text{ m/s}$ linear surface velocity).

Given an operator command with linear velocity $v$ (m/s) and angular velocity $\omega_{deg}$ (deg/s):

$$\omega_{rad} = \omega_{deg} \times \frac{\pi}{180}$$

The linear velocities for the left and right wheels are:

$$v_{left} = v - \frac{\omega_{rad} \times L}{2}$$
$$v_{right} = v + \frac{\omega_{rad} \times L}{2}$$

The target angular velocities for the left and right drive motors (rad/s) passed to `sim.setJointTargetVelocity`:

$$\omega_{left} = \frac{v_{left}}{R} = \frac{v - \frac{\omega_{rad} \times L}{2}}{R}$$
$$\omega_{right} = \frac{v_{right}}{R} = \frac{v + \frac{\omega_{rad} \times L}{2}}{R}$$

### Direction Signs & Behavior

- **Forward Motion ($v > 0, \omega = 0$):** Both $\omega_{left}$ and $\omega_{right}$ are positive.
- **Backward Motion ($v < 0, \omega = 0$):** Both $\omega_{left}$ and $\omega_{right}$ are negative.
- **Turn Left ($v = 0, \omega > 0$, CCW):** $\omega_{left} < 0$, $\omega_{right} > 0$. Left wheel reverses, right wheel moves forward.
- **Turn Right ($v = 0, \omega < 0$, CW):** $\omega_{left} > 0$, $\omega_{right} < 0$. Left wheel moves forward, right wheel reverses.
- **Velocity Saturation Clamping:** If either wheel exceeds `max_wheel_speed`, both wheel velocities are scaled down proportionally, preserving the turning radius ($\frac{v}{\omega}$).

---

## 2. Object Resolution & Hierarchy

The backend resolves required objects by their exact scene names:

| Object | Default Name | Role |
| :--- | :--- | :--- |
| **Robot Base** | `PioneerP3DX` | Root mobile base, pose and velocity reference |
| **Left Motor** | `leftMotor` | Revolute joint controlling left wheel velocity |
| **Right Motor** | `rightMotor` | Revolute joint controlling right wheel velocity |

### Resolution Protocol
1. Checks the exact path as given (`/PioneerP3DX`, `leftMotor`).
2. Checks with a root slash prefix (`/leftMotor`).
3. Checks relative to the robot base proxy (`./leftMotor` under `PioneerP3DX`).
4. Scans all scene object aliases matching the name.
5. If any object cannot be found, raises an informative `ValueError` specifying the missing object name and hierarchy hints.

### Child Script Deactivation
The standard Pioneer P3-DX model in CoppeliaSim includes a default Braitenberg obstacle avoidance child script. If active, this script overwrites motor velocities during simulation. On initialization, `CoppeliaSimRobot` sets the model property `sim.modelproperty_scripts_inactive` on the Pioneer model, disabling the obstacle avoidance script so motor velocities are exclusively controlled by the teleoperation safety layer.

---

## 3. Simulation State Handling & Safety

- **Simulation Stopped:** Commanding motion while CoppeliaSim simulation is stopped raises a `RuntimeError` by default (`auto_start_simulation=False`), prompting the operator to start the simulation. If `auto_start_simulation=True`, simulation is started automatically with explicit log notification.
- **Simulation Paused:** Raises a `RuntimeError` explaining that simulation is paused.
- **Status Reporting Accuracy:** Querying `get_status()` when the simulation is stopped or paused reports physical linear and angular velocities strictly as `0.0`, with `is_moving = false`. This avoids misreporting physical velocity when the physics engine is not advancing.
- **Negative Zero Normalization:** IEEE 754 floating-point `-0.0` values and velocities below stationary sensor deadband thresholds (`0.005 m/s` linear, `0.1 deg/s` angular) are sanitized to clean `0.0`.
- **Stopping (`stop()`):** Commands both `leftMotor` and `rightMotor` target velocities to `0.0 rad/s`. Crucially, `stop()` does not stop or pause the simulation clock, preserving simulator stability.
- **Emergency Stop, Timers & Watchdog:** E-Stop (`ESTOP`), timed movement expiry, watchdog expiration (>1.5s), and client disconnect all invoke `stop()`, cutting motor power to `0.0 rad/s`.

---

## 4. Setup and Verification

### Step 1: Open CoppeliaSim Scene
Open the provided scene file in CoppeliaSim Edu:
```
D:\AISR-Project\scenes\aisr_teleoperation.ttt
```
CoppeliaSim will automatically start the ZeroMQ Remote API server on port `23000`.

### Step 2: Run Connection Smoke Test
Run the non-moving smoke-test script:
```bash
python scripts/test_coppeliasim_connection.py
```
Expected output:
```
=================================================================
AISR Milestone 3A - CoppeliaSim Connection Smoke Test
=================================================================
Target endpoint : localhost:23000
Target objects  : PioneerP3DX, leftMotor, rightMotor

[1/4] Connecting to CoppeliaSim ZeroMQ Remote API...
      [OK] Connected successfully.
[2/4] Verifying scene object handles:
      [OK] Robot base  ('PioneerP3DX')  : handle = 15
      [OK] Left motor  ('leftMotor')   : handle = 20
      [OK] Right motor ('rightMotor')  : handle = 17
[3/4] Checking simulation state:
      Simulation state: STOPPED (idle)
[4/4] Inspecting initial robot pose and velocity state:
      Position (x, y)  : (-0.125 m, -0.125 m)
      Heading          : 0.0 deg
      Linear velocity  : 0.000 m/s
      Angular velocity : 0.000 deg/s
      Moving flag      : False
      Left motor target velocity  : 0.0 rad/s
      Right motor target velocity : 0.0 rad/s

-----------------------------------------------------------------
SMOKE TEST RESULT: SUCCESS
CoppeliaSim Pioneer P3-DX is properly configured and accessible.
NO ROBOT MOTION COMMANDED. Robot remains stationary.
-----------------------------------------------------------------
```

---

## 5. Running the Teleoperation Server

### Option A: Mock Backend (Default for testing)
```bash
python -m src.tcp_server
# or
python -m src.tcp_server --backend mock
```

### Option B: CoppeliaSim Pioneer P3-DX Backend
Start simulation in CoppeliaSim (press the Play button in the CoppeliaSim UI), then run:
```bash
python -m src.tcp_server --backend coppeliasim
# or via environment variable
set AISR_ROBOT_BACKEND=coppeliasim
python -m src.tcp_server
```

Server output:
```
Initializing robot backend: 'coppeliasim'
AISR server listening on 127.0.0.1:5000 (backend: coppeliasim)
Waiting for the operator client...
```

*Note: If CoppeliaSim is requested but unreachable, the server exits immediately with a fatal connection error and will NEVER silently fall back to the mock backend.*

### Launch Operator Client
In a separate terminal:
```bash
python -m src.tcp_client
```
Interactive commands:
```
AISR> FORWARD 0.5 2.0
AISR> STATUS
AISR> TURN_LEFT 30.0 1.0
AISR> STOP
AISR> ESTOP
AISR> RESET
```

---

## 6. Running Automated Tests

Run all unit and integration tests (both mock and simulated tests run without needing CoppeliaSim open):
```bash
python -m unittest discover -s tests -v
```
All 171 tests pass.
