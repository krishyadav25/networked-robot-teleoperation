"""
tcp_server.py – AISR robot teleoperation server.

Listens on a TCP socket and processes newline-delimited JSON messages
using the shared protocol from src.protocol.

Run from the project root:
    python -m src.tcp_server
"""

import socket
import sys

from src.protocol import MessageBuffer, encode_message

HOST = "127.0.0.1"
PORT = 5000
RECV_SIZE = 4096
SOCKET_TIMEOUT = 120  # seconds – drop idle clients after 2 minutes


def handle_message(message: dict) -> dict:
    """Build a response for a single validated message.

    Parameters
    ----------
    message : dict
        A decoded protocol message (guaranteed to have a ``type`` key).

    Returns
    -------
    dict
        A JSON-serialisable response dictionary.
    """
    msg_type: str = message["type"]

    return {
        "type": "ACK",
        "received_type": msg_type,
        "status": "ok",
    }


def serve_client(conn: socket.socket, address: tuple) -> None:
    """Read protocol messages from *conn* and send responses.

    Uses :class:`MessageBuffer` to handle TCP framing correctly:
    partial reads, or multiple messages arriving in one ``recv()``.

    Parameters
    ----------
    conn : socket.socket
        The accepted client connection.
    address : tuple
        ``(host, port)`` of the remote client.
    """
    print(f"Connected to client: {address}")
    conn.settimeout(SOCKET_TIMEOUT)
    buffer = MessageBuffer()

    try:
        while True:
            try:
                data: bytes = conn.recv(RECV_SIZE)
            except socket.timeout:
                print("Client timed out.")
                break

            if not data:
                # Client closed the connection gracefully.
                break

            buffer.feed(data)

            for message in buffer.get_messages():
                print(f"Received: {message}")
                response: dict = handle_message(message)
                conn.sendall(encode_message(response))

    except ConnectionResetError:
        print("Client connection reset.")
    finally:
        conn.close()
        print(f"Client {address} disconnected.")


def main() -> None:
    """Start the TCP server and accept a single client."""
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
            sys.exit(0)

        with conn:
            serve_client(conn, address)


if __name__ == "__main__":
    main()
