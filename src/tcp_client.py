"""
tcp_client.py – AISR robot teleoperation client.

Connects to the AISR server, sends automatic heartbeats every 0.5s in
a background thread, and lets the operator type interactive commands.

Run from the project root:
    python -m src.tcp_client
"""

import json
import socket
import sys
import threading
import time

from src.protocol import MessageBuffer, encode_message

HOST = "127.0.0.1"
PORT = 5000
RECV_SIZE = 4096
HEARTBEAT_INTERVAL = 0.5  # seconds


# ---------------------------------------------------------------------------
# Heartbeat sender (background thread)
# ---------------------------------------------------------------------------

def _heartbeat_loop(
    sock: socket.socket,
    stop_event: threading.Event,
    lock: threading.Lock,
) -> None:
    """Send ``{"type": "HEARTBEAT"}`` every ``HEARTBEAT_INTERVAL`` seconds.

    Runs in a daemon thread.  Stops when *stop_event* is set or the
    socket becomes unusable.
    """
    heartbeat_msg = encode_message({"type": "HEARTBEAT"})
    while not stop_event.wait(timeout=HEARTBEAT_INTERVAL):
        try:
            with lock:
                sock.sendall(heartbeat_msg)
        except OSError:
            # Socket closed or broken — exit silently.
            break


# ---------------------------------------------------------------------------
# Response reader (background thread)
# ---------------------------------------------------------------------------

def _reader_loop(
    sock: socket.socket,
    stop_event: threading.Event,
    buffer: MessageBuffer,
) -> None:
    """Continuously read and display server responses.

    Runs in a daemon thread so the main thread can block on ``input()``
    without missing heartbeat ACKs or async responses.
    """
    sock.settimeout(0.5)
    while not stop_event.is_set():
        try:
            data = sock.recv(RECV_SIZE)
        except socket.timeout:
            continue
        except OSError:
            break

        if not data:
            print("\nServer closed the connection.")
            stop_event.set()
            break

        buffer.feed(data)
        for response in buffer.get_messages():
            # Suppress noisy heartbeat ACKs from the display.
            if response.get("type") == "HEARTBEAT_ACK":
                continue
            print(f"\n  ← Server: {json.dumps(response)}")


# ---------------------------------------------------------------------------
# Command helpers
# ---------------------------------------------------------------------------

_HELP_TEXT = """
Available commands:
  FORWARD [speed]       – Move forward (default 0.5 m/s)
  BACKWARD [speed]      – Move backward
  TURN_LEFT [speed]     – Turn left (default 45 deg/s)
  TURN_RIGHT [speed]    – Turn right
  MOVE <linear> <angular> – Set velocities directly
  GOTO <x> <y>          – Navigate to coordinates
  STOP                  – Stop the robot
  STATUS                – Query robot state
  ESTOP                 – Emergency stop (latched)
  RESET                 – Clear emergency stop
  PING                  – Basic ping
  help                  – Show this help
  quit                  – Disconnect
"""


def _parse_user_command(user_input: str) -> dict | None:
    """Turn a user's text input into a protocol message dict.

    Returns ``None`` if the input couldn't be parsed.
    """
    parts = user_input.strip().split()
    if not parts:
        return None

    cmd = parts[0].upper()

    # Commands with no arguments
    if cmd in ("STOP", "STATUS", "ESTOP", "RESET", "PING", "HEARTBEAT"):
        return {"type": cmd}

    # FORWARD / BACKWARD [speed]
    if cmd in ("FORWARD", "BACKWARD"):
        msg: dict = {"type": cmd}
        if len(parts) >= 2:
            try:
                msg["speed"] = float(parts[1])
            except ValueError:
                print(f"  Invalid speed: {parts[1]}")
                return None
        return msg

    # TURN_LEFT / TURN_RIGHT [speed]
    if cmd in ("TURN_LEFT", "TURN_RIGHT"):
        msg = {"type": cmd}
        if len(parts) >= 2:
            try:
                msg["speed"] = float(parts[1])
            except ValueError:
                print(f"  Invalid speed: {parts[1]}")
                return None
        return msg

    # MOVE <linear> <angular>
    if cmd == "MOVE":
        if len(parts) < 3:
            print("  Usage: MOVE <linear> <angular>")
            return None
        try:
            return {
                "type": "MOVE",
                "linear": float(parts[1]),
                "angular": float(parts[2]),
            }
        except ValueError:
            print("  Invalid velocity values.")
            return None

    # GOTO <x> <y>
    if cmd == "GOTO":
        if len(parts) < 3:
            print("  Usage: GOTO <x> <y>")
            return None
        try:
            return {"type": "GOTO", "x": float(parts[1]), "y": float(parts[2])}
        except ValueError:
            print("  Invalid coordinates.")
            return None

    # Anything else — send as-is
    return {"type": cmd}


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    """Connect to the AISR server and send protocol messages."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client:
        try:
            client.connect((HOST, PORT))
        except ConnectionRefusedError:
            print(f"Could not connect to {HOST}:{PORT}. Is the server running?")
            sys.exit(1)

        print(f"Connected to AISR server at {HOST}:{PORT}")
        print("Type 'help' for available commands.\n")

        stop_event = threading.Event()
        send_lock = threading.Lock()
        buffer = MessageBuffer()

        # Start background threads.
        hb_thread = threading.Thread(
            target=_heartbeat_loop,
            args=(client, stop_event, send_lock),
            daemon=True,
            name="heartbeat-sender",
        )
        reader_thread = threading.Thread(
            target=_reader_loop,
            args=(client, stop_event, buffer),
            daemon=True,
            name="response-reader",
        )
        hb_thread.start()
        reader_thread.start()

        try:
            while not stop_event.is_set():
                try:
                    user_input = input("AISR> ")
                except EOFError:
                    break

                if user_input.strip().lower() == "quit":
                    print("Disconnecting...")
                    break

                if user_input.strip().lower() == "help":
                    print(_HELP_TEXT)
                    continue

                message = _parse_user_command(user_input)
                if message is None:
                    continue

                try:
                    encoded = encode_message(message)
                except ValueError as exc:
                    print(f"  Invalid message: {exc}")
                    continue

                try:
                    with send_lock:
                        client.sendall(encoded)
                    print(f"  → Sent: {json.dumps(message)}")
                except OSError:
                    print("  Connection lost.")
                    break

        except KeyboardInterrupt:
            print("\nInterrupted.")

        finally:
            stop_event.set()
            # Give threads a moment to finish.
            hb_thread.join(timeout=2)
            reader_thread.join(timeout=2)

    print("Connection closed.")


if __name__ == "__main__":
    main()
