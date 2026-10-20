"""
run_experiments.py – Automated experimental testing of CoppeliaSim robot teleoperation.

Conducts controlled robot movement trials over TCP using newline-delimited JSON,
measures initial and final poses, computes Euclidean and signed displacement,
evaluates error against kinematic expectations, and records data to CSV.

Supports both direct connection (0ms baseline) and delayed connections via the
AISR TCP Delay Proxy for 100, 300, 500, and 1000 ms latency conditions.

Run baseline trial:
    python scripts/run_experiments.py --trials 1
    python scripts/run_experiments.py --trials 3 --speed 0.05 --duration 2.0

Run with delay proxy:
    python scripts/run_experiments.py --port 5001 --delay-ms 300 --trials 3

Run automated sweep across [0, 100, 300, 500, 1000] ms:
    python scripts/run_experiments.py --auto-sweep --trials 3
"""

from __future__ import annotations

import argparse
import csv
import datetime
import math
import os
import queue
import socket
import sys
import threading
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Optional

# Add project root to sys.path
project_root = Path(__file__).resolve().parent.parent
if str(project_root) not in sys.path:
    sys.path.insert(0, str(project_root))

from src.delay_proxy import DelayProxy, DelayProxyConfig
from src.protocol import MessageBuffer, encode_message


# ---------------------------------------------------------------------------
# Data Models and Calculations
# ---------------------------------------------------------------------------

@dataclass
class DisplacementMetrics:
    """Displacement and error metrics for a movement trial."""
    euclidean_displacement: float
    signed_x_displacement: float
    signed_y_displacement: float
    displacement_error: float
    x_displacement_error: float


def compute_displacement_metrics(
    start_x: float,
    start_y: float,
    end_x: float,
    end_y: float,
    expected_displacement: float,
) -> DisplacementMetrics:
    """Compute Euclidean and Cartesian displacement and their deviation from expected.

    Parameters
    ----------
    start_x, start_y : float
        Starting coordinate in metres.
    end_x, end_y : float
        Ending coordinate in metres.
    expected_displacement : float
        Theoretical displacement (speed * duration) in metres.
    """
    dx = end_x - start_x
    dy = end_y - start_y
    euclidean = math.sqrt(dx * dx + dy * dy)
    disp_error = euclidean - expected_displacement
    x_error = dx - expected_displacement

    return DisplacementMetrics(
        euclidean_displacement=euclidean,
        signed_x_displacement=dx,
        signed_y_displacement=dy,
        displacement_error=disp_error,
        x_displacement_error=x_error,
    )


@dataclass
class TrialRecord:
    """Complete experimental trial record saved to CSV."""
    timestamp: str
    tag: str
    delay_ms: float
    trial_num: int
    command: str
    commanded_speed: float
    commanded_duration: float
    expected_displacement: float
    start_x: float
    start_y: float
    start_heading: float
    end_x: float
    end_y: float
    end_heading: float
    euclidean_displacement: float
    signed_x_displacement: float
    signed_y_displacement: float
    displacement_error: float
    x_displacement_error: float
    elapsed_time: float
    final_state: str
    is_stopped: bool
    success: bool
    error_message: str


CSV_FIELDNAMES = [
    "timestamp",
    "tag",
    "delay_ms",
    "trial_num",
    "command",
    "commanded_speed",
    "commanded_duration",
    "expected_displacement",
    "start_x",
    "start_y",
    "start_heading",
    "end_x",
    "end_y",
    "end_heading",
    "euclidean_displacement",
    "signed_x_displacement",
    "signed_y_displacement",
    "displacement_error",
    "x_displacement_error",
    "elapsed_time",
    "final_state",
    "is_stopped",
    "success",
    "error_message",
]


def log_trial_to_csv(record: TrialRecord, csv_path: Path | str) -> None:
    """Append a TrialRecord to the specified CSV file without overwriting existing data."""
    csv_file = Path(csv_path)
    csv_file.parent.mkdir(parents=True, exist_ok=True)
    file_exists = csv_file.exists() and csv_file.stat().st_size > 0

    with open(csv_file, mode="a", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=CSV_FIELDNAMES)
        if not file_exists:
            writer.writeheader()
        writer.writerow(asdict(record))


# ---------------------------------------------------------------------------
# Experiment TCP Client with Active Heartbeat Management
# ---------------------------------------------------------------------------

class ExperimentClient:
    """Thread-safe TCP client managing command requests and background heartbeats."""

    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int = 5000,
        heartbeat_interval: float = 0.5,
        connect_timeout: float = 3.0,
    ) -> None:
        self.host = host
        self.port = port
        self.heartbeat_interval = heartbeat_interval
        self.connect_timeout = connect_timeout

        self._sock: socket.socket | None = None
        self._stop_event = threading.Event()
        self._send_lock = threading.Lock()
        self._resp_queue: queue.Queue[dict] = queue.Queue()

        self._reader_thread: threading.Thread | None = None
        self._heartbeat_thread: threading.Thread | None = None

    def connect(self) -> None:
        """Establish TCP connection and start reader and heartbeat background threads."""
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.settimeout(self.connect_timeout)
        self._sock.connect((self.host, self.port))
        self._sock.settimeout(0.2)

        self._stop_event.clear()

        # Reader thread
        self._reader_thread = threading.Thread(
            target=self._reader_loop, daemon=True, name="exp-client-reader"
        )
        self._reader_thread.start()

        # Heartbeat sender thread
        self._heartbeat_thread = threading.Thread(
            target=self._heartbeat_loop, daemon=True, name="exp-client-heartbeat"
        )
        self._heartbeat_thread.start()

    def disconnect(self) -> None:
        """Close connection and stop worker threads cleanly."""
        self._stop_event.set()
        if self._sock:
            try:
                self._sock.close()
            except Exception:
                pass
            self._sock = None

        if self._reader_thread and self._reader_thread.is_alive():
            self._reader_thread.join(timeout=1.0)
        if self._heartbeat_thread and self._heartbeat_thread.is_alive():
            self._heartbeat_thread.join(timeout=1.0)

    def _reader_loop(self) -> None:
        buf = MessageBuffer()
        while not self._stop_event.is_set():
            if not self._sock:
                break
            try:
                data = self._sock.recv(4096)
            except socket.timeout:
                continue
            except (OSError, ConnectionResetError):
                break

            if not data:
                break

            buf.feed(data)
            for msg in buf.get_messages():
                # Filter out heartbeat ACKs so response queue only holds command replies
                if msg.get("type") == "HEARTBEAT_ACK":
                    continue
                self._resp_queue.put(msg)

        self._stop_event.set()

    def _heartbeat_loop(self) -> None:
        hb_data = encode_message({"type": "HEARTBEAT"})
        while not self._stop_event.wait(timeout=self.heartbeat_interval):
            if not self._sock or self._stop_event.is_set():
                break
            try:
                with self._send_lock:
                    self._sock.sendall(hb_data)
            except (OSError, ConnectionResetError):
                break

    def send_command(self, cmd: dict, timeout: float = 5.0) -> dict:
        """Send a protocol command and wait for its corresponding response."""
        if self._stop_event.is_set() or not self._sock:
            raise ConnectionError("Client is not connected.")

        wire_bytes = encode_message(cmd)
        with self._send_lock:
            self._sock.sendall(wire_bytes)

        # Wait for reply from queue
        try:
            resp = self._resp_queue.get(timeout=timeout)
            return resp
        except queue.Empty:
            raise TimeoutError(f"No response received for command '{cmd.get('type')}' within {timeout}s.")


# ---------------------------------------------------------------------------
# Trial Runner
# ---------------------------------------------------------------------------

def run_single_trial(
    client: ExperimentClient,
    trial_num: int,
    speed: float = 0.05,
    duration: float = 2.0,
    delay_ms: float = 0.0,
    tag: str = "baseline",
    timeout_margin: float = 4.0,
) -> TrialRecord:
    """Execute a single timed motion trial and return the structured TrialRecord."""
    expected_disp = speed * duration
    ts = datetime.datetime.now().isoformat()

    print(f"\n--- [Trial {trial_num} | Delay: {delay_ms} ms | Tag: {tag}] ---")
    print(f"Commanded: FORWARD speed={speed:.3f} m/s, duration={duration:.2f} s (expected: {expected_disp:.3f} m)")

    # 1. Query initial pose
    try:
        init_resp = client.send_command({"type": "STATUS"}, timeout=4.0)
    except Exception as exc:
        err = f"Failed to query initial STATUS: {exc}"
        print(f"  [ERROR] {err}")
        return TrialRecord(
            timestamp=ts, tag=tag, delay_ms=delay_ms, trial_num=trial_num,
            command="FORWARD", commanded_speed=speed, commanded_duration=duration,
            expected_displacement=expected_disp, start_x=0.0, start_y=0.0, start_heading=0.0,
            end_x=0.0, end_y=0.0, end_heading=0.0, euclidean_displacement=0.0,
            signed_x_displacement=0.0, signed_y_displacement=0.0, displacement_error=0.0,
            x_displacement_error=0.0, elapsed_time=0.0, final_state="ERROR",
            is_stopped=True, success=False, error_message=err,
        )

    if init_resp.get("status") != "ok":
        err = f"Initial STATUS returned error: {init_resp}"
        print(f"  [ERROR] {err}")
        return TrialRecord(
            timestamp=ts, tag=tag, delay_ms=delay_ms, trial_num=trial_num,
            command="FORWARD", commanded_speed=speed, commanded_duration=duration,
            expected_displacement=expected_disp, start_x=0.0, start_y=0.0, start_heading=0.0,
            end_x=0.0, end_y=0.0, end_heading=0.0, euclidean_displacement=0.0,
            signed_x_displacement=0.0, signed_y_displacement=0.0, displacement_error=0.0,
            x_displacement_error=0.0, elapsed_time=0.0, final_state=str(init_resp.get("state")),
            is_stopped=True, success=False, error_message=err,
        )

    start_x = float(init_resp.get("x", 0.0))
    start_y = float(init_resp.get("y", 0.0))
    start_heading = float(init_resp.get("heading_deg", 0.0))
    start_state = str(init_resp.get("state", "UNKNOWN"))

    print(f"Initial Pose: x={start_x:.4f} m, y={start_y:.4f} m, heading={start_heading:.2f}°, state={start_state}")

    if start_state in ("ESTOP_LATCHED", "COMM_LOST"):
        err = f"Robot is in non-movable state '{start_state}'. Aborting trial without moving."
        print(f"  [ERROR] {err}")
        return TrialRecord(
            timestamp=ts, tag=tag, delay_ms=delay_ms, trial_num=trial_num,
            command="FORWARD", commanded_speed=speed, commanded_duration=duration,
            expected_displacement=expected_disp, start_x=start_x, start_y=start_y,
            start_heading=start_heading, end_x=start_x, end_y=start_y, end_heading=start_heading,
            euclidean_displacement=0.0, signed_x_displacement=0.0, signed_y_displacement=0.0,
            displacement_error=-expected_disp, x_displacement_error=-expected_disp,
            elapsed_time=0.0, final_state=start_state, is_stopped=True, success=False, error_message=err,
        )

    # 2. Dispatch FORWARD command
    wall_start = time.time()
    try:
        fwd_resp = client.send_command(
            {"type": "FORWARD", "speed": speed, "duration": duration},
            timeout=5.0 + (delay_ms / 1000.0),
        )
    except Exception as exc:
        err = f"Failed to send FORWARD command: {exc}"
        print(f"  [ERROR] {err}")
        return TrialRecord(
            timestamp=ts, tag=tag, delay_ms=delay_ms, trial_num=trial_num,
            command="FORWARD", commanded_speed=speed, commanded_duration=duration,
            expected_displacement=expected_disp, start_x=start_x, start_y=start_y,
            start_heading=start_heading, end_x=start_x, end_y=start_y, end_heading=start_heading,
            euclidean_displacement=0.0, signed_x_displacement=0.0, signed_y_displacement=0.0,
            displacement_error=-expected_disp, x_displacement_error=-expected_disp,
            elapsed_time=0.0, final_state=start_state, is_stopped=True, success=False, error_message=err,
        )

    if fwd_resp.get("status") != "ok":
        err = f"FORWARD command rejected by server: {fwd_resp.get('message', fwd_resp)}"
        print(f"  [ERROR] {err}")
        return TrialRecord(
            timestamp=ts, tag=tag, delay_ms=delay_ms, trial_num=trial_num,
            command="FORWARD", commanded_speed=speed, commanded_duration=duration,
            expected_displacement=expected_disp, start_x=start_x, start_y=start_y,
            start_heading=start_heading, end_x=start_x, end_y=start_y, end_heading=start_heading,
            euclidean_displacement=0.0, signed_x_displacement=0.0, signed_y_displacement=0.0,
            displacement_error=-expected_disp, x_displacement_error=-expected_disp,
            elapsed_time=0.0, final_state=str(fwd_resp.get("state", "REJECTED")),
            is_stopped=True, success=False, error_message=err,
        )

    print(f"FORWARD Accepted: speed={fwd_resp.get('speed')} m/s, duration={fwd_resp.get('duration')} s")

    # 3. Wait for timed motion completion
    # Motion duration is 'duration' seconds. We wait 'duration' seconds, then poll STATUS until IDLE
    wait_until = wall_start + duration + (delay_ms / 1000.0)
    sleep_first = max(0.0, (wait_until - time.time()) * 0.9)
    if sleep_first > 0:
        time.sleep(sleep_first)

    # Poll status until state returns to IDLE (or safety timeout expires)
    deadline = wall_start + duration + timeout_margin + (delay_ms / 1000.0) * 2.0
    final_status = {}
    is_stopped = False

    while time.time() < deadline:
        try:
            status_poll = client.send_command({"type": "STATUS"}, timeout=2.0)
            if status_poll.get("status") == "ok":
                final_status = status_poll
                curr_state = status_poll.get("state")
                is_moving = status_poll.get("is_moving", True)
                lin_vel = status_poll.get("linear_velocity", 0.0)
                if curr_state == "IDLE" and not is_moving and abs(lin_vel) < 0.005:
                    is_stopped = True
                    break
        except Exception:
            pass
        time.sleep(0.15)

    wall_end = time.time()
    elapsed_time = wall_end - wall_start

    end_x = float(final_status.get("x", start_x))
    end_y = float(final_status.get("y", start_y))
    end_heading = float(final_status.get("heading_deg", start_heading))
    final_state = str(final_status.get("state", "UNKNOWN"))

    metrics = compute_displacement_metrics(
        start_x=start_x,
        start_y=start_y,
        end_x=end_x,
        end_y=end_y,
        expected_displacement=expected_disp,
    )

    print(f"Final Pose   : x={end_x:.4f} m, y={end_y:.4f} m, heading={end_heading:.2f}°, state={final_state}")
    print(f"Displacement : Euclidean={metrics.euclidean_displacement:.4f} m, Signed X={metrics.signed_x_displacement:.4f} m")
    print(f"Deviation    : Error={metrics.displacement_error:.4f} m (Elapsed wall time: {elapsed_time:.2f} s)")

    return TrialRecord(
        timestamp=ts,
        tag=tag,
        delay_ms=delay_ms,
        trial_num=trial_num,
        command="FORWARD",
        commanded_speed=speed,
        commanded_duration=duration,
        expected_displacement=expected_disp,
        start_x=start_x,
        start_y=start_y,
        start_heading=start_heading,
        end_x=end_x,
        end_y=end_y,
        end_heading=end_heading,
        euclidean_displacement=metrics.euclidean_displacement,
        signed_x_displacement=metrics.signed_x_displacement,
        signed_y_displacement=metrics.signed_y_displacement,
        displacement_error=metrics.displacement_error,
        x_displacement_error=metrics.x_displacement_error,
        elapsed_time=elapsed_time,
        final_state=final_state,
        is_stopped=is_stopped,
        success=True,
        error_message="",
    )


# ---------------------------------------------------------------------------
# CLI & Execution Orchestration
# ---------------------------------------------------------------------------

def run_experiment_suite(
    host: str,
    port: int,
    trials: int,
    speed: float,
    duration: float,
    delay_ms: float,
    tag: str,
    output_csv: Path | str,
) -> list[TrialRecord]:
    """Execute a series of trials for a given delay condition."""
    print("=" * 65)
    print(f"Starting Experiment: {trials} trial(s) at {speed} m/s for {duration} s (Delay: {delay_ms} ms)")
    print(f"Connecting to AISR server at {host}:{port}...")
    print("=" * 65)

    client = ExperimentClient(host=host, port=port)
    try:
        client.connect()
    except Exception as exc:
        print(f"[FATAL] Could not connect to AISR server at {host}:{port}: {exc}", file=sys.stderr)
        print("Please verify that the AISR server is running: python -m src.tcp_server --backend coppeliasim", file=sys.stderr)
        return []

    records: list[TrialRecord] = []
    try:
        for t in range(1, trials + 1):
            record = run_single_trial(
                client=client,
                trial_num=t,
                speed=speed,
                duration=duration,
                delay_ms=delay_ms,
                tag=tag,
            )
            records.append(record)
            log_trial_to_csv(record, output_csv)
            print(f"  [SAVED] Trial {t} logged to {output_csv}")
            if t < trials:
                time.sleep(1.0)
    finally:
        client.disconnect()
        print("\nDisconnected from AISR server.")

    return records


def run_automated_delay_sweep(
    server_host: str,
    server_port: int,
    proxy_port: int,
    trials_per_condition: int,
    speed: float,
    duration: float,
    delays: list[float],
    output_csv: Path | str,
) -> None:
    """Execute an automated delay sweep across specified delay values using in-process proxy."""
    print("=" * 65)
    print("AISR Automated Network Latency Experiment Sweep")
    print("=" * 65)
    print(f"Target Server   : {server_host}:{server_port}")
    print(f"Proxy Port      : {proxy_port}")
    print(f"Latency Sweeps  : {delays} ms")
    print(f"Trials per delay: {trials_per_condition}")
    print(f"CSV Output Path : {output_csv}")
    print("=" * 65)

    for delay in delays:
        print(f"\n>>> Running condition: Added Delay = {delay} ms <<<")
        if delay == 0.0:
            # Baseline: Connect directly to server
            run_experiment_suite(
                host=server_host,
                port=server_port,
                trials=trials_per_condition,
                speed=speed,
                duration=duration,
                delay_ms=0.0,
                tag="baseline_0ms",
                output_csv=output_csv,
            )
        else:
            # Latency condition: Launch DelayProxy
            proxy_cfg = DelayProxyConfig(
                listen_host="127.0.0.1",
                listen_port=proxy_port,
                target_host=server_host,
                target_port=server_port,
                delay_ms=delay,
                mode="one-way",  # One-way uplink latency
            )
            proxy = DelayProxy(proxy_cfg)
            proxy.start()
            time.sleep(0.3)  # Brief warm up

            try:
                run_experiment_suite(
                    host="127.0.0.1",
                    port=proxy_port,
                    trials=trials_per_condition,
                    speed=speed,
                    duration=duration,
                    delay_ms=delay,
                    tag=f"delayed_{int(delay)}ms",
                    output_csv=output_csv,
                )
            finally:
                proxy.stop()
                time.sleep(0.5)

    print("\n" + "=" * 65)
    print("ALL EXPERIMENTAL CONDITIONS COMPLETED.")
    print(f"Results saved to: {output_csv}")
    print("To plot graphs: python scripts/plot_results.py")
    print("=" * 65)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Automated experimental testing of CoppeliaSim Pioneer P3-DX teleoperation."
    )
    parser.add_argument("--host", default="127.0.0.1", help="Target server or proxy host (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=5000, help="Target server or proxy port (default: 5000)")
    parser.add_argument("--trials", type=int, default=1, help="Number of trials to execute (default: 1)")
    parser.add_argument("--speed", type=float, default=0.05, help="Forward speed in m/s (default: 0.05)")
    parser.add_argument("--duration", type=float, default=2.0, help="Forward duration in seconds (default: 2.0)")
    parser.add_argument("--delay-ms", type=float, default=0.0, help="Metadata delay in ms when using external proxy (default: 0.0)")
    parser.add_argument("--tag", default="baseline", help="Experiment condition tag/label (default: baseline)")
    parser.add_argument("--output", default="results/experiment_results.csv", help="CSV output filepath")
    parser.add_argument(
        "--auto-sweep",
        action="store_true",
        help="Run full automated latency sweep across [0, 100, 300, 500, 1000] ms with internal delay proxy",
    )
    parser.add_argument(
        "--sweep-delays",
        type=float,
        nargs="+",
        default=[0.0, 100.0, 300.0, 500.0, 1000.0],
        help="Delay values for --auto-sweep in ms (default: 0 100 300 500 1000)",
    )
    parser.add_argument("--proxy-port", type=int, default=5001, help="Port for internal delay proxy during --auto-sweep (default: 5001)")

    args = parser.parse_args()

    if args.auto_sweep:
        run_automated_delay_sweep(
            server_host=args.host,
            server_port=args.port,
            proxy_port=args.proxy_port,
            trials_per_condition=args.trials,
            speed=args.speed,
            duration=args.duration,
            delays=args.sweep_delays,
            output_csv=args.output,
        )
    else:
        run_experiment_suite(
            host=args.host,
            port=args.port,
            trials=args.trials,
            speed=args.speed,
            duration=args.duration,
            delay_ms=args.delay_ms,
            tag=args.tag,
            output_csv=args.output,
        )


if __name__ == "__main__":
    main()
