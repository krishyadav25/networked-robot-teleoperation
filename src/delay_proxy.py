"""
delay_proxy.py – Configurable TCP proxy for injecting network latency.

Forwards newline-delimited JSON protocol messages between an experiment client
and the AISR robot server, delaying messages by a configurable latency.

Preserves exact message framing, FIFO ordering, and newline termination.
Supports both one-way (uplink only) and round-trip (uplink + downlink) delay modes.
"""

from __future__ import annotations

import argparse
import queue
import socket
import sys
import threading
import time
from dataclasses import dataclass
from typing import Optional

from src.protocol import MessageBuffer, encode_message


@dataclass
class DelayProxyConfig:
    """Configuration for the delay proxy."""
    listen_host: str = "127.0.0.1"
    listen_port: int = 5001
    target_host: str = "127.0.0.1"
    target_port: int = 5000
    delay_ms: float = 0.0
    mode: str = "one-way"  # "one-way" (uplink only) or "round-trip" (split equally)
    uplink_delay_ms: Optional[float] = None
    downlink_delay_ms: Optional[float] = None

    @property
    def effective_uplink_delay_sec(self) -> float:
        """Effective delay in seconds for client -> server messages."""
        if self.uplink_delay_ms is not None:
            return max(0.0, self.uplink_delay_ms / 1000.0)
        if self.mode == "one-way":
            return max(0.0, self.delay_ms / 1000.0)
        if self.mode == "round-trip":
            return max(0.0, (self.delay_ms / 2.0) / 1000.0)
        raise ValueError(f"Unknown mode: {self.mode}")

    @property
    def effective_downlink_delay_sec(self) -> float:
        """Effective delay in seconds for server -> client responses."""
        if self.downlink_delay_ms is not None:
            return max(0.0, self.downlink_delay_ms / 1000.0)
        if self.mode == "one-way":
            return 0.0
        if self.mode == "round-trip":
            return max(0.0, (self.delay_ms / 2.0) / 1000.0)
        raise ValueError(f"Unknown mode: {self.mode}")

    @property
    def total_round_trip_delay_ms(self) -> float:
        """Total round-trip delay added to a request-response cycle in ms."""
        return (self.effective_uplink_delay_sec + self.effective_downlink_delay_sec) * 1000.0


class DelayedForwarder:
    """Forwards messages from a source socket to a dest socket with added delay."""

    def __init__(
        self,
        source_sock: socket.socket,
        dest_sock: socket.socket,
        delay_sec: float,
        channel_name: str = "forwarder",
    ) -> None:
        self.source_sock = source_sock
        self.dest_sock = dest_sock
        self.delay_sec = delay_sec
        self.channel_name = channel_name
        self.stop_event = threading.Event()
        self.queue: queue.Queue[tuple[float, bytes] | None] = queue.Queue()
        self.reader_thread: threading.Thread | None = None
        self.sender_thread: threading.Thread | None = None

    def start(self) -> None:
        self.stop_event.clear()
        self.reader_thread = threading.Thread(
            target=self._read_loop,
            daemon=True,
            name=f"{self.channel_name}-reader",
        )
        self.sender_thread = threading.Thread(
            target=self._send_loop,
            daemon=True,
            name=f"{self.channel_name}-sender",
        )
        self.reader_thread.start()
        self.sender_thread.start()

    def stop(self) -> None:
        self.stop_event.set()
        self.queue.put(None)
        if self.reader_thread and self.reader_thread.is_alive():
            self.reader_thread.join(timeout=1.0)
        if self.sender_thread and self.sender_thread.is_alive():
            self.sender_thread.join(timeout=1.0)

    def _read_loop(self) -> None:
        buf = MessageBuffer()
        self.source_sock.settimeout(0.2)
        while not self.stop_event.is_set():
            try:
                data = self.source_sock.recv(4096)
            except socket.timeout:
                continue
            except (OSError, ConnectionResetError):
                break

            if not data:
                # Source socket closed EOF
                break

            buf.feed(data)
            # Retrieve complete messages and enqueue them with schedule timestamp
            for msg in buf.get_messages():
                wire_bytes = encode_message(msg)
                forward_time = time.monotonic() + self.delay_sec
                self.queue.put((forward_time, wire_bytes))

        self.stop_event.set()
        self.queue.put(None)

    def _send_loop(self) -> None:
        while not self.stop_event.is_set():
            try:
                item = self.queue.get(timeout=0.2)
            except queue.Empty:
                continue

            if item is None:
                break

            forward_time, wire_bytes = item
            now = time.monotonic()
            wait = forward_time - now
            if wait > 0:
                time.sleep(wait)

            try:
                self.dest_sock.sendall(wire_bytes)
            except (OSError, ConnectionResetError):
                break
            finally:
                self.queue.task_done()


class DelayProxy:
    """Thread-safe TCP proxy injecting configurable transmission latency."""

    def __init__(self, config: DelayProxyConfig | None = None) -> None:
        self.config = config or DelayProxyConfig()
        self._server_sock: socket.socket | None = None
        self._running = False
        self._listener_thread: threading.Thread | None = None
        self._active_forwarders: list[DelayedForwarder] = []
        self._lock = threading.Lock()

    @property
    def is_running(self) -> bool:
        return self._running

    def start(self) -> None:
        """Start listening for client connections on the configured listen port."""
        if self._running:
            return

        self._server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server_sock.bind((self.config.listen_host, self.config.listen_port))
        self._server_sock.listen(5)
        self._server_sock.settimeout(0.5)

        self._running = True
        self._listener_thread = threading.Thread(
            target=self._listen_loop,
            daemon=True,
            name="delay-proxy-listener",
        )
        self._listener_thread.start()

    def stop(self) -> None:
        """Stop proxy and disconnect all active forwarders."""
        self._running = False
        if self._server_sock:
            try:
                self._server_sock.close()
            except Exception:
                pass
            self._server_sock = None

        if self._listener_thread and self._listener_thread.is_alive():
            self._listener_thread.join(timeout=2.0)

        with self._lock:
            for f in self._active_forwarders:
                f.stop()
            self._active_forwarders.clear()

    def _listen_loop(self) -> None:
        while self._running:
            try:
                assert self._server_sock is not None
                client_sock, client_addr = self._server_sock.accept()
            except (socket.timeout, OSError):
                continue

            # Connect to target server
            try:
                target_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                target_sock.connect((self.config.target_host, self.config.target_port))
            except Exception as exc:
                print(f"[DelayProxy] Failed to connect to target {self.config.target_host}:{self.config.target_port}: {exc}", file=sys.stderr)
                client_sock.close()
                continue

            # Launch forwarder pair: client -> target (uplink), target -> client (downlink)
            up_f = DelayedForwarder(
                source_sock=client_sock,
                dest_sock=target_sock,
                delay_sec=self.config.effective_uplink_delay_sec,
                channel_name=f"up-{client_addr[1]}",
            )
            down_f = DelayedForwarder(
                source_sock=target_sock,
                dest_sock=client_sock,
                delay_sec=self.config.effective_downlink_delay_sec,
                channel_name=f"down-{client_addr[1]}",
            )

            with self._lock:
                self._active_forwarders.extend([up_f, down_f])

            up_f.start()
            down_f.start()

            # Supervisor thread for this connection pair
            def _supervisor(up: DelayedForwarder, down: DelayedForwarder, c_s: socket.socket, t_s: socket.socket) -> None:
                while self._running and not (up.stop_event.is_set() or down.stop_event.is_set()):
                    time.sleep(0.1)
                up.stop()
                down.stop()
                try:
                    c_s.close()
                except Exception:
                    pass
                try:
                    t_s.close()
                except Exception:
                    pass
                with self._lock:
                    if up in self._active_forwarders:
                        self._active_forwarders.remove(up)
                    if down in self._active_forwarders:
                        self._active_forwarders.remove(down)

            t = threading.Thread(
                target=_supervisor,
                args=(up_f, down_f, client_sock, target_sock),
                daemon=True,
                name=f"supervisor-{client_addr[1]}",
            )
            t.start()


def main() -> None:
    parser = argparse.ArgumentParser(description="AISR TCP Network Latency Injection Proxy")
    parser.add_argument("--listen-port", type=int, default=5001, help="Port to listen for client connections (default: 5001)")
    parser.add_argument("--listen-host", default="127.0.0.1", help="Host interface to bind (default: 127.0.0.1)")
    parser.add_argument("--target-port", type=int, default=5000, help="Target AISR server port (default: 5000)")
    parser.add_argument("--target-host", default="127.0.0.1", help="Target AISR server host (default: 127.0.0.1)")
    parser.add_argument("--delay-ms", type=float, default=0.0, help="Added latency in milliseconds (default: 0.0)")
    parser.add_argument(
        "--mode",
        choices=["one-way", "round-trip"],
        default="one-way",
        help="Delay mode: 'one-way' adds delay to uplink only; 'round-trip' splits delay equally between uplink and downlink.",
    )
    parser.add_argument("--uplink-delay-ms", type=float, default=None, help="Explicit uplink delay in ms (overrides --mode)")
    parser.add_argument("--downlink-delay-ms", type=float, default=None, help="Explicit downlink delay in ms (overrides --mode)")

    args = parser.parse_args()

    config = DelayProxyConfig(
        listen_host=args.listen_host,
        listen_port=args.listen_port,
        target_host=args.target_host,
        target_port=args.target_port,
        delay_ms=args.delay_ms,
        mode=args.mode,
        uplink_delay_ms=args.uplink_delay_ms,
        downlink_delay_ms=args.downlink_delay_ms,
    )

    proxy = DelayProxy(config)
    print("=" * 65)
    print("AISR TCP Delay Proxy")
    print("=" * 65)
    print(f"Listening on       : {config.listen_host}:{config.listen_port}")
    print(f"Forwarding to      : {config.target_host}:{config.target_port}")
    print(f"Configured Mode    : {config.mode}")
    print(f"Uplink Delay       : {config.effective_uplink_delay_sec * 1000.0:.1f} ms")
    print(f"Downlink Delay     : {config.effective_downlink_delay_sec * 1000.0:.1f} ms")
    print(f"Total Round-Trip   : {config.total_round_trip_delay_ms:.1f} ms")
    print("=" * 65)
    print("Press Ctrl+C to terminate proxy.")

    proxy.start()
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\nShutting down delay proxy...")
    finally:
        proxy.stop()
        print("Proxy exited.")


if __name__ == "__main__":
    main()
