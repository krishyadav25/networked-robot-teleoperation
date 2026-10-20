# AISR Experimental Benchmark & Latency Testing Guide

This guide describes how to run automated experimental trials on the CoppeliaSim Pioneer P3-DX robot over TCP, inject network latency using the TCP delay proxy, record trial measurements to CSV, and generate analytical graphs.

---

## 1. Experimental Overview

The experiment evaluates teleoperation accuracy, trajectory consistency, and responsiveness under baseline and delayed network conditions:

- **Baseline Motion Command:** `FORWARD` at **0.05 m/s** for **2.0 seconds**.
- **Expected Kinematic Displacement:**
  $$\Delta x_{expected} = v \times t = 0.05\text{ m/s} \times 2.0\text{ s} = 0.100\text{ m (10 cm)}$$
- **Measured Metrics:**
  - Start pose: $(x_1, y_1, \theta_1)$
  - Final pose: $(x_2, y_2, \theta_2)$
  - Actual elapsed wall-clock time
  - Euclidean displacement: $\sqrt{(x_2 - x_1)^2 + (y_2 - y_1)^2}$
  - Signed X displacement: $x_2 - x_1$
  - Signed Y displacement: $y_2 - y_1$
  - Displacement error: $\text{Euclidean} - \Delta x_{expected}$
  - Signed X error: $(x_2 - x_1) - \Delta x_{expected}$
  - Operational state and stopped verification (`is_moving = false`, `linear_velocity = 0.0`)
- **Latency Sweep Conditions:** `0 ms` (baseline direct), `100 ms`, `300 ms`, `500 ms`, and `1000 ms`.

---

## 2. Step-by-Step Procedure

### Step 1: Start CoppeliaSim Simulation

1. Launch **CoppeliaSim Edu**.
2. Open the scene file:
   ```
   D:\AISR-Project\scenes\aisr_teleoperation.ttt
   ```
3. Ensure the Pioneer P3-DX robot has clear forward space along its heading.
4. Start the simulation by pressing the **Play** button in the CoppeliaSim toolbar (or configure `auto_start_simulation=True`).

### Step 2: Start the Teleoperation Server

Open PowerShell and start the server with the CoppeliaSim backend:

```powershell
python -m src.tcp_server --backend coppeliasim
```

Output:
```
Initializing robot backend: 'coppeliasim'
AISR server listening on 127.0.0.1:5000 (backend: coppeliasim)
Waiting for the operator client...
```

---

## 3. Running the Baseline Experiment

To execute the baseline trial without added delay (direct connection to port 5000):

```powershell
python scripts/run_experiments.py --trials 1
```

For multiple repeated baseline trials (e.g. 3 trials):

```powershell
python scripts/run_experiments.py --trials 3 --speed 0.05 --duration 2.0 --tag baseline
```

Console Output Example:
```
--- [Trial 1 | Delay: 0.0 ms | Tag: baseline] ---
Commanded: FORWARD speed=0.050 m/s, duration=2.00 s (expected: 0.100 m)
Initial Pose: x=-0.0365 m, y=-0.1325 m, heading=0.17°, state=IDLE
FORWARD Accepted: speed=0.05 m/s, duration=2.0 s
Final Pose   : x=0.0632 m, y=-0.1324 m, heading=0.18°, state=IDLE
Displacement : Euclidean=0.0997 m, Signed X=0.0997 m
Deviation    : Error=-0.0003 m (Elapsed wall time: 2.12 s)
  [SAVED] Trial 1 logged to results/experiment_results.csv
```

All trials are appended directly to `results/experiment_results.csv` without overwriting historical runs.

---

## 4. Latency Injection & Delay Configuration

The experiment suite provides a dedicated TCP delay proxy (`src/delay_proxy.py`) that transparently buffers and forwards newline-delimited protocol frames with millisecond precision. Network latency is genuinely injected at the transport layer, not by sleeping the client thread before message transmission.

### Delay Modes & Semantics

| Mode | Argument | Semantics |
| :--- | :--- | :--- |
| **`one-way` (Default)** | `--mode one-way` | Adds delay to the client $\to$ server uplink only. Server responses return immediately without artificial delay. |
| **`round-trip`** | `--mode round-trip` | Splits configured latency equally: $\frac{\text{delay}}{2}$ on uplink and $\frac{\text{delay}}{2}$ on downlink. Total round-trip latency added to request-response cycles equals the configured value. |
| **Explicit** | `--uplink-delay-ms <ms>`<br>`--downlink-delay-ms <ms>` | Configures directional delays independently. |

### Option A: Automated Sweep Across All Delays (Recommended)

Run a complete automated sweep across all required conditions (0, 100, 300, 500, 1000 ms) with 3 trials each:

```powershell
python scripts/run_experiments.py --auto-sweep --trials 3
```

This command connects directly for the 0ms baseline, and automatically manages an internal delay proxy on port 5001 for each subsequent latency condition.

### Option B: Standalone Delay Proxy

To test an individual latency condition manually:

**Terminal 1 (Delay Proxy):**
```powershell
python scripts/delay_proxy.py --delay-ms 300 --listen-port 5001 --target-port 5000
```

**Terminal 2 (Experiment Runner):**
```powershell
python scripts/run_experiments.py --port 5001 --delay-ms 300 --trials 3 --tag delay_300ms
```

### Heartbeat Watchdog Interaction Under Latency

The client transmits `{"type": "HEARTBEAT"}` messages every **0.5 seconds**. The server watchdog triggers `COMM_LOST` and cuts motor power if no heartbeat arrives within **1.5 seconds**. Under steady latency (even 1000 ms), the FIFO queue maintains the 0.5s inter-arrival spacing after the initial pipeline delay. If network latency exceeds 1.5s or significant pipeline jitter occurs, the watchdog correctly enters `COMM_LOST`, validating server-side safety under degraded network conditions.

---

## 5. Generating Visual Graphs & Summary Statistics

Generate graphical plots and view summary tables from the recorded CSV data:

```powershell
python scripts/plot_results.py
```

Optional arguments:
```powershell
python scripts/plot_results.py --csv results/experiment_results.csv --outdir results/
```

### Generated Artifacts in `results/`:

1. **`results/displacement_vs_delay.png`**:
   - Total Euclidean displacement across delays with error bars (mean $\pm$ standard deviation).
   - Dashed red benchmark line showing theoretical kinematic displacement ($v \times t = 0.10\text{ m}$).
   - Subplot showing displacement error deviation.
2. **`results/trajectory_xy.png`**:
   - 2D trajectory vector plots from start $(x_1, y_1)$ to end $(x_2, y_2)$ in CoppeliaSim world coordinates.
3. **`results/time_vs_delay.png`**:
   - Measured trial elapsed wall-clock time vs commanded duration (2.0s).

### Console Summary Output Example:
```
================================================================================
EXPERIMENTAL BENCHMARK SUMMARY TABLE
================================================================================
Delay [ms]   | Trials   | Mean Disp [m]   | Mean Error [m]   | Mean Time [s]  | Expected [m]
--------------------------------------------------------------------------------
0.0          | 3        | 0.0998          | -0.0002          | 2.11           | 0.1000
100.0        | 3        | 0.0997          | -0.0003          | 2.21           | 0.1000
300.0        | 3        | 0.0996          | -0.0004          | 2.42           | 0.1000
500.0        | 3        | 0.0995          | -0.0005          | 2.62           | 0.1000
1000.0       | 3        | 0.0992          | -0.0008          | 3.12           | 0.1000
================================================================================
Note: Preliminary baseline measurements. Real-world trajectory depends on physics time-step.
```

---

## 6. Safety & Operational Constraints

1. **Low Speeds and Short Durations:** Always operate with linear speed $\le 0.1\text{ m/s}$ and duration $\le 3.0\text{ s}$ during testing.
2. **Never Auto-Recover:** If any error occurs or `ESTOP` is triggered, the experiment runner never automatically commands movement or clears latches without explicit human control.
3. **Clear Floor Space:** Ensure the Pioneer P3-DX robot is positioned away from walls or barriers before beginning multi-trial runs.
