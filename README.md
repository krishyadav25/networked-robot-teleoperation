# AISR-Project

## Networked Teleoperation of a Robot through Shell Commands

Control a simulated **Pioneer P3-DX robot** in CoppeliaSim using terminal commands sent over TCP. The system includes heartbeat monitoring, an emergency stop, and network-delay experiments.

## Architecture

- **Operator layer:** `src/tcp_client.py` — sends commands from the terminal.
- **Network layer:** TCP communication between the client and server.
- **Safety layer:** `src/tcp_server.py` — command validation, heartbeat watchdog, and emergency-stop handling.
- **Execution layer:** CoppeliaSim backend for controlling the simulated robot.
- **Experiment layer:** Scripts for measuring network delay and robot movement.

## Run

Install the dependencies:

```powershell
python -m pip install -r requirements.txt
```

Open CoppeliaSim and load the project scene:

`scenes/aisr_teleoperation.ttt`

Start the server in Terminal 1:

```powershell
python -m src.tcp_server --backend coppeliasim
```

Start the client in Terminal 2:

```powershell
python -m src.tcp_client
```

Use the available shell commands to control the robot and check its status.

## Safety Features

- Heartbeat monitoring and communication timeout detection.
- Automatic stopping when communication is lost.
- Emergency stop (E-stop) with a latched safety state.
- Reset handling that does not automatically resume movement.

## Testing

Run the automated test suite:

```powershell
python -m unittest discover -s tests -v
```

## Network Delay Experiments

Run the experiment runner:

```powershell
python scripts/run_experiments.py --trials 3
```

Generate plots from recorded experiment data:

```powershell
python scripts/plot_results.py
```

See `docs/experiments_and_benchmarks.md` for detailed experiment instructions.

## Project Repository

GitHub: https://github.com/soumya-jain123/AISR-Project
