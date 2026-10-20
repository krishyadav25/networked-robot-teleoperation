"""
test_experiments.py – Automated unit tests for experiment runner, displacement calculations,
CSV logging, delay proxy configuration, and error handling.

Does not require a running simulator or live server.
"""

from __future__ import annotations

import csv
import math
import socket
import tempfile
import threading
import time
import unittest
from pathlib import Path

from scripts.plot_results import generate_plots, load_results
from scripts.run_experiments import (
    CSV_FIELDNAMES,
    DisplacementMetrics,
    ExperimentClient,
    TrialRecord,
    compute_displacement_metrics,
    log_trial_to_csv,
    run_single_trial,
)
from src.delay_proxy import DelayProxy, DelayProxyConfig
from src.protocol import MessageBuffer, encode_message


# ============================================================================
# 1. Displacement Calculations
# ============================================================================

class TestDisplacementCalculations(unittest.TestCase):
    """Verify Euclidean, signed X/Y, and displacement error calculations."""

    def test_positive_x_forward(self):
        metrics = compute_displacement_metrics(
            start_x=0.0,
            start_y=0.0,
            end_x=0.10,
            end_y=0.0,
            expected_displacement=0.10,
        )
        self.assertAlmostEqual(metrics.euclidean_displacement, 0.10, places=4)
        self.assertAlmostEqual(metrics.signed_x_displacement, 0.10, places=4)
        self.assertAlmostEqual(metrics.signed_y_displacement, 0.0, places=4)
        self.assertAlmostEqual(metrics.displacement_error, 0.0, places=4)
        self.assertAlmostEqual(metrics.x_displacement_error, 0.0, places=4)

    def test_diagonal_displacement(self):
        # 3-4-5 triangle: dx=0.03, dy=0.04 -> hypot=0.05
        metrics = compute_displacement_metrics(
            start_x=1.00,
            start_y=2.00,
            end_x=1.03,
            end_y=2.04,
            expected_displacement=0.05,
        )
        self.assertAlmostEqual(metrics.euclidean_displacement, 0.05, places=4)
        self.assertAlmostEqual(metrics.signed_x_displacement, 0.03, places=4)
        self.assertAlmostEqual(metrics.signed_y_displacement, 0.04, places=4)
        self.assertAlmostEqual(metrics.displacement_error, 0.0, places=4)

    def test_under_and_over_displacement(self):
        # Under-displacement: expected 0.10, actual 0.08 -> error = -0.02
        under = compute_displacement_metrics(0.0, 0.0, 0.08, 0.0, 0.10)
        self.assertAlmostEqual(under.displacement_error, -0.02, places=4)

        # Over-displacement: expected 0.10, actual 0.12 -> error = +0.02
        over = compute_displacement_metrics(0.0, 0.0, 0.12, 0.0, 0.10)
        self.assertAlmostEqual(over.displacement_error, 0.02, places=4)

    def test_stationary_zero_displacement(self):
        metrics = compute_displacement_metrics(0.5, 0.5, 0.5, 0.5, 0.10)
        self.assertEqual(metrics.euclidean_displacement, 0.0)
        self.assertEqual(metrics.signed_x_displacement, 0.0)
        self.assertAlmostEqual(metrics.displacement_error, -0.10, places=4)


# ============================================================================
# 2. CSV Logging
# ============================================================================

class TestCSVLogging(unittest.TestCase):
    """Verify non-destructive appending, formatting, and error logging to CSV."""

    def setUp(self):
        self.temp_dir = tempfile.TemporaryDirectory()
        self.csv_path = Path(self.temp_dir.name) / "test_results.csv"

    def tearDown(self):
        self.temp_dir.cleanup()

    def test_create_file_with_header(self):
        record = TrialRecord(
            timestamp="2026-10-21T01:00:00",
            tag="baseline",
            delay_ms=0.0,
            trial_num=1,
            command="FORWARD",
            commanded_speed=0.05,
            commanded_duration=2.0,
            expected_displacement=0.10,
            start_x=0.0,
            start_y=0.0,
            start_heading=0.0,
            end_x=0.098,
            end_y=0.001,
            end_heading=0.2,
            euclidean_displacement=0.098,
            signed_x_displacement=0.098,
            signed_y_displacement=0.001,
            displacement_error=-0.002,
            x_displacement_error=-0.002,
            elapsed_time=2.05,
            final_state="IDLE",
            is_stopped=True,
            success=True,
            error_message="",
        )
        log_trial_to_csv(record, self.csv_path)

        self.assertTrue(self.csv_path.exists())
        with open(self.csv_path, mode="r", encoding="utf-8") as f:
            reader = list(csv.reader(f))
            self.assertEqual(reader[0], CSV_FIELDNAMES)
            self.assertEqual(len(reader), 2)  # header + 1 record
            self.assertEqual(reader[1][1], "baseline")
            self.assertEqual(reader[1][4], "FORWARD")

    def test_append_preserves_previous_records(self):
        r1 = TrialRecord(
            timestamp="2026-10-21T01:00:00", tag="t1", delay_ms=0.0, trial_num=1,
            command="FORWARD", commanded_speed=0.05, commanded_duration=2.0,
            expected_displacement=0.10, start_x=0.0, start_y=0.0, start_heading=0.0,
            end_x=0.10, end_y=0.0, end_heading=0.0, euclidean_displacement=0.10,
            signed_x_displacement=0.10, signed_y_displacement=0.0, displacement_error=0.0,
            x_displacement_error=0.0, elapsed_time=2.0, final_state="IDLE",
            is_stopped=True, success=True, error_message="",
        )
        r2 = TrialRecord(
            timestamp="2026-10-21T01:01:00", tag="t2", delay_ms=300.0, trial_num=2,
            command="FORWARD", commanded_speed=0.05, commanded_duration=2.0,
            expected_displacement=0.10, start_x=0.10, start_y=0.0, start_heading=0.0,
            end_x=0.20, end_y=0.0, end_heading=0.0, euclidean_displacement=0.10,
            signed_x_displacement=0.10, signed_y_displacement=0.0, displacement_error=0.0,
            x_displacement_error=0.0, elapsed_time=2.3, final_state="IDLE",
            is_stopped=True, success=True, error_message="",
        )

        log_trial_to_csv(r1, self.csv_path)
        log_trial_to_csv(r2, self.csv_path)

        with open(self.csv_path, mode="r", encoding="utf-8") as f:
            lines = [line.strip() for line in f if line.strip()]
            self.assertEqual(len(lines), 3)  # 1 header + 2 data rows

    def test_error_trial_logging(self):
        err_record = TrialRecord(
            timestamp="2026-10-21T01:02:00", tag="err", delay_ms=1000.0, trial_num=3,
            command="FORWARD", commanded_speed=0.05, commanded_duration=2.0,
            expected_displacement=0.10, start_x=0.0, start_y=0.0, start_heading=0.0,
            end_x=0.0, end_y=0.0, end_heading=0.0, euclidean_displacement=0.0,
            signed_x_displacement=0.0, signed_y_displacement=0.0, displacement_error=-0.10,
            x_displacement_error=-0.10, elapsed_time=0.0, final_state="ERROR",
            is_stopped=True, success=False, error_message="Watchdog timeout",
        )
        log_trial_to_csv(err_record, self.csv_path)

        results = load_results(self.csv_path)
        self.assertEqual(len(results), 1)
        self.assertFalse(results[0]["success"])
        self.assertEqual(results[0]["error_message"], "Watchdog timeout")


# ============================================================================
# 3. Delay Proxy Configuration and Semantics
# ============================================================================

class TestDelayProxyConfiguration(unittest.TestCase):
    """Verify DelayProxyConfig calculation of uplink, downlink, and round-trip delay."""

    def test_default_config(self):
        cfg = DelayProxyConfig()
        self.assertEqual(cfg.delay_ms, 0.0)
        self.assertEqual(cfg.effective_uplink_delay_sec, 0.0)
        self.assertEqual(cfg.effective_downlink_delay_sec, 0.0)
        self.assertEqual(cfg.total_round_trip_delay_ms, 0.0)

    def test_one_way_mode(self):
        cfg = DelayProxyConfig(delay_ms=300.0, mode="one-way")
        self.assertAlmostEqual(cfg.effective_uplink_delay_sec, 0.300, places=4)
        self.assertAlmostEqual(cfg.effective_downlink_delay_sec, 0.0, places=4)
        self.assertAlmostEqual(cfg.total_round_trip_delay_ms, 300.0, places=1)

    def test_round_trip_mode(self):
        cfg = DelayProxyConfig(delay_ms=500.0, mode="round-trip")
        self.assertAlmostEqual(cfg.effective_uplink_delay_sec, 0.250, places=4)
        self.assertAlmostEqual(cfg.effective_downlink_delay_sec, 0.250, places=4)
        self.assertAlmostEqual(cfg.total_round_trip_delay_ms, 500.0, places=1)

    def test_explicit_overrides(self):
        cfg = DelayProxyConfig(uplink_delay_ms=100.0, downlink_delay_ms=200.0)
        self.assertAlmostEqual(cfg.effective_uplink_delay_sec, 0.100, places=4)
        self.assertAlmostEqual(cfg.effective_downlink_delay_sec, 0.200, places=4)
        self.assertAlmostEqual(cfg.total_round_trip_delay_ms, 300.0, places=1)

    def test_invalid_mode_raises(self):
        cfg = DelayProxyConfig(mode="unknown")
        with self.assertRaises(ValueError):
            _ = cfg.effective_uplink_delay_sec


# ============================================================================
# 4. In-Memory Mock Server Helpers
# ============================================================================

class MockEchoServer:
    """Lightweight test loopback server for testing proxy forwarding and client."""

    def __init__(self, response_delay: float = 0.0) -> None:
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("127.0.0.1", 0))  # OS chooses free port
        self.port = self.sock.getsockname()[1]
        self.sock.listen(1)
        self.response_delay = response_delay
        self.stop_event = threading.Event()
        self.received_messages: list[dict] = []
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self) -> None:
        self.sock.settimeout(0.5)
        try:
            conn, _ = self.sock.accept()
        except (socket.timeout, OSError):
            return

        buf = MessageBuffer()
        conn.settimeout(0.2)
        while not self.stop_event.is_set():
            try:
                data = conn.recv(4096)
            except socket.timeout:
                continue
            except OSError:
                break
            if not data:
                break

            buf.feed(data)
            for msg in buf.get_messages():
                self.received_messages.append(msg)
                msg_type = msg.get("type")

                if self.response_delay > 0:
                    time.sleep(self.response_delay)

                if msg_type == "HEARTBEAT":
                    conn.sendall(encode_message({"type": "HEARTBEAT_ACK", "status": "ok", "state": "IDLE"}))
                elif msg_type == "STATUS":
                    conn.sendall(encode_message({
                        "type": "STATUS_RESPONSE", "status": "ok", "state": "IDLE",
                        "x": 0.0, "y": 0.0, "heading_deg": 0.0, "linear_velocity": 0.0,
                        "angular_velocity": 0.0, "is_moving": False,
                    }))
                elif msg_type == "FORWARD":
                    conn.sendall(encode_message({
                        "type": "FORWARD_ACK", "status": "ok", "state": "MOVING",
                        "speed": msg.get("speed", 0.05), "duration": msg.get("duration", 2.0),
                    }))
                elif msg_type == "PING":
                    conn.sendall(encode_message({"type": "ACK", "received_type": "PING", "status": "ok"}))

        try:
            conn.close()
        except Exception:
            pass

    def stop(self) -> None:
        self.stop_event.set()
        try:
            self.sock.close()
        except Exception:
            pass
        self._thread.join(timeout=1.0)


# ============================================================================
# 5. Delay Proxy Transmission Testing
# ============================================================================

class TestDelayProxyTransmission(unittest.TestCase):
    """Verify that DelayProxy accurately delays message arrival while preserving framing."""

    def test_proxy_delays_message_transmission(self):
        mock_server = MockEchoServer()
        try:
            # Configure proxy with 60ms delay
            proxy_cfg = DelayProxyConfig(
                listen_host="127.0.0.1",
                listen_port=0,  # OS chosen port
                target_host="127.0.0.1",
                target_port=mock_server.port,
                delay_ms=60.0,
                mode="one-way",
            )
            # Find free port for proxy
            probe = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            probe.bind(("127.0.0.1", 0))
            proxy_port = probe.getsockname()[1]
            probe.close()
            proxy_cfg.listen_port = proxy_port

            proxy = DelayProxy(proxy_cfg)
            proxy.start()
            self.assertTrue(proxy.is_running)

            # Connect client to proxy
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client:
                client.connect(("127.0.0.1", proxy_port))
                t0 = time.monotonic()
                client.sendall(encode_message({"type": "PING"}))

                buf = MessageBuffer()
                client.settimeout(2.0)
                while True:
                    data = client.recv(4096)
                    buf.feed(data)
                    msgs = buf.get_messages()
                    if msgs:
                        break

                rtt = time.monotonic() - t0
                resp = msgs[0]
                self.assertEqual(resp.get("type"), "ACK")
                # RTT must be at least the 60ms delay
                self.assertGreaterEqual(rtt, 0.050)

            proxy.stop()
            self.assertFalse(proxy.is_running)
        finally:
            mock_server.stop()


# ============================================================================
# 6. Experiment Client and Error Handling
# ============================================================================

class TestExperimentClient(unittest.TestCase):
    """Verify ExperimentClient connection, timeout, and trial execution error handling."""

    def test_client_connect_and_heartbeat(self):
        server = MockEchoServer()
        try:
            client = ExperimentClient(host="127.0.0.1", port=server.port, heartbeat_interval=0.1)
            client.connect()

            # Send ping
            resp = client.send_command({"type": "PING"}, timeout=2.0)
            self.assertEqual(resp.get("type"), "ACK")

            # Verify server received heartbeats
            time.sleep(0.3)
            hb_msgs = [m for m in server.received_messages if m.get("type") == "HEARTBEAT"]
            self.assertGreaterEqual(len(hb_msgs), 1)

            client.disconnect()
        finally:
            server.stop()

    def test_client_send_timeout(self):
        # Server that delays responding for 5 seconds
        server = MockEchoServer(response_delay=5.0)
        try:
            client = ExperimentClient(host="127.0.0.1", port=server.port)
            client.connect()
            with self.assertRaises(TimeoutError):
                client.send_command({"type": "STATUS"}, timeout=0.3)
            client.disconnect()
        finally:
            server.stop()

    def test_client_disconnected_send_raises(self):
        client = ExperimentClient(host="127.0.0.1", port=9999)
        with self.assertRaises(ConnectionError):
            client.send_command({"type": "PING"})

    def test_run_single_trial_handles_server_rejection(self):
        # Mock server that rejects FORWARD
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.bind(("127.0.0.1", 0))
        p = sock.getsockname()[1]
        sock.listen(1)

        def _handler():
            sock.settimeout(1.0)
            try:
                conn, _ = sock.accept()
            except OSError:
                return
            buf = MessageBuffer()
            while True:
                data = conn.recv(1024)
                if not data:
                    break
                buf.feed(data)
                for m in buf.get_messages():
                    if m.get("type") == "STATUS":
                        conn.sendall(encode_message({"type": "STATUS_RESPONSE", "status": "ok", "state": "IDLE", "x": 0, "y": 0, "heading_deg": 0, "linear_velocity": 0, "angular_velocity": 0, "is_moving": False}))
                    elif m.get("type") == "FORWARD":
                        conn.sendall(encode_message({"type": "ERROR", "status": "rejected", "message": "Emergency stop is active"}))
            conn.close()

        t = threading.Thread(target=_handler, daemon=True)
        t.start()

        client = ExperimentClient(host="127.0.0.1", port=p)
        client.connect()
        try:
            record = run_single_trial(client, trial_num=1, speed=0.05, duration=2.0)
            self.assertFalse(record.success)
            self.assertIn("rejected", record.error_message)
        finally:
            client.disconnect()
            sock.close()
            t.join(timeout=1.0)


# ============================================================================
# 7. Plotting and Statistical Summary
# ============================================================================

class TestPlotResults(unittest.TestCase):
    """Verify plot generation and statistics aggregation."""

    def test_load_non_existent_csv(self):
        results = load_results("results/does_not_exist_xyz.csv")
        self.assertEqual(results, [])

    def test_generate_plots_with_records(self):
        records = [
            {
                "timestamp": "2026-10-21T01:00:00", "tag": "baseline", "delay_ms": 0.0,
                "trial_num": 1, "command": "FORWARD", "commanded_speed": 0.05,
                "commanded_duration": 2.0, "expected_displacement": 0.10,
                "start_x": 0.0, "start_y": 0.0, "start_heading": 0.0,
                "end_x": 0.098, "end_y": 0.0, "end_heading": 0.0,
                "euclidean_displacement": 0.098, "signed_x_displacement": 0.098,
                "signed_y_displacement": 0.0, "displacement_error": -0.002,
                "x_displacement_error": -0.002, "elapsed_time": 2.05,
                "final_state": "IDLE", "is_stopped": True, "success": True, "error_message": "",
            },
            {
                "timestamp": "2026-10-21T01:05:00", "tag": "delayed_300ms", "delay_ms": 300.0,
                "trial_num": 2, "command": "FORWARD", "commanded_speed": 0.05,
                "commanded_duration": 2.0, "expected_displacement": 0.10,
                "start_x": 0.0, "start_y": 0.0, "start_heading": 0.0,
                "end_x": 0.099, "end_y": 0.0, "end_heading": 0.0,
                "euclidean_displacement": 0.099, "signed_x_displacement": 0.099,
                "signed_y_displacement": 0.0, "displacement_error": -0.001,
                "x_displacement_error": -0.001, "elapsed_time": 2.35,
                "final_state": "IDLE", "is_stopped": True, "success": True, "error_message": "",
            },
        ]
        with tempfile.TemporaryDirectory() as temp_dir:
            plots = generate_plots(records, temp_dir)
            self.assertEqual(len(plots), 3)
            for p in plots:
                self.assertTrue(Path(p).exists())
                self.assertGreater(Path(p).stat().st_size, 1000)


if __name__ == "__main__":
    unittest.main()
