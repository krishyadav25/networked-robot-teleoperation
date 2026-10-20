"""
tcp_server.py – AISR robot teleoperation server.

Listens on a TCP socket, processes newline-delimited JSON messages via
the shared protocol, and dispatches commands through the safety
controller to the robot backend.

Run from the project root:
    python -m src.tcp_server
"""

import socket
import sys
import threading

from src.protocol import MessageBuffer, encode_message
from src.robot_interface import MockRobot
from src.safety import SafetyConfig, SafetyController

HOST = "127.0.0.1"
PORT = 5000
RECV_SIZE = 4096
SOCKET_TIMEOUT = 0.5  # short timeout so watchdog isn't blocked by recv()


def handle_message(message: dict) -> dict:
    """Legacy simple handler kept for backward-compatible tests.

    The real server uses ``SafetyController.dispatch()`` instead.
    """
    msg_type: str = message["type"]
    return {
        "type": "ACK",
        "received_type": msg_type,
        "status": "ok",
    }


def serve_client(
    conn: socket.socket,
    address: tuple,
    controller: SafetyController,
) -> None:
    """Read protocol messages from *conn* and send responses.

    Uses :class:`MessageBuffer` to handle TCP framing correctly.
    The short socket timeout allows the heartbeat watchdog to keep
    running even when no data arrives.

    Parameters
    ----------
    conn : socket.socket
        The accepted client connection.
    address : tuple
        ``(host, port)`` of the remote client.
    controller : SafetyController
        The safety controller managing state and robot commands.
    """
    print(f"Connected to client: {address}")
    conn.settimeout(SOCKET_TIMEOUT)
    buffer = MessageBuffer()

    try:
        while True:
            try:
                data: bytes = conn.recv(RECV_SIZE)
            except socket.timeout:
                # No data this cycle — watchdog keeps running in its
                # own thread, so we just loop back to recv().
                continue
            except OSError:
                # Socket closed or broken.
                break

            if not data:
                # Client closed the connection gracefully.
                break

            buffer.feed(data)

            for message in buffer.get_messages():
                msg_type = message.get("type", "")
                if msg_type.upper() != "HEARTBEAT":
                    print(f"Received: {message}")

                response: dict = controller.dispatch(message)
                try:
                    conn.sendall(encode_message(response))
                except OSError:
                    # Client disconnected while we were responding.
                    return

    except ConnectionResetError:
        print("Client connection reset.")
    finally:
        controller.on_client_disconnect()
        conn.close()
        print(f"Client {address} disconnected. Robot stopped.")


def main() -> None:
    """Start the TCP server and accept a single client."""
    robot = MockRobot()
    controller = SafetyController(robot)
    controller.start_watchdog()

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((HOST, PORT))
        server.listen(1)

        print(f"AISR server listening on {HOST}:{PORT}")
        print("Waiting for the operator client...")

        try:
            conn, address = server.accept()
        except KeyboardInterrupt:
            print("\nServer shut down.")
            controller.stop_watchdog()
            sys.exit(0)

        with conn:
            serve_client(conn, address, controller)

    controller.stop_watchdog()
    print("Server exited.")


if __name__ == "__main__":
    main()
