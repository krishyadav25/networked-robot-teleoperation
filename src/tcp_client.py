"""
tcp_client.py – AISR robot teleoperation client.

Connects to the AISR server and sends newline-delimited JSON messages
using the shared protocol from src.protocol.

Run from the project root:
    python -m src.tcp_client
"""

import json
import socket
import sys

from src.protocol import MessageBuffer, encode_message

HOST = "127.0.0.1"
PORT = 5000
RECV_SIZE = 4096
SOCKET_TIMEOUT = 30  # seconds – how long to wait for a server response


def main() -> None:
    """Connect to the AISR server and send protocol messages."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client:
        try:
            client.connect((HOST, PORT))
        except ConnectionRefusedError:
            print(f"Could not connect to {HOST}:{PORT}. Is the server running?")
            sys.exit(1)

        client.settimeout(SOCKET_TIMEOUT)
        print(f"Connected to AISR server at {HOST}:{PORT}")

        buffer = MessageBuffer()

        while True:
            user_input: str = input(
                "\nEnter message type (e.g. PING, STATUS) or 'quit' to exit: "
            )

            if user_input.strip().lower() == "quit":
                print("Disconnecting...")
                break

            # Build a structured message from user input.
            message: dict = {"type": user_input.strip()}

            try:
                encoded: bytes = encode_message(message)
            except ValueError as exc:
                print(f"Invalid message: {exc}")
                continue

            client.sendall(encoded)
            print(f"Sent: {json.dumps(message)}")

            # Read the response.
            try:
                data: bytes = client.recv(RECV_SIZE)
            except socket.timeout:
                print("Server did not respond in time.")
                continue

            if not data:
                print("Server closed the connection.")
                break

            buffer.feed(data)

            for response in buffer.get_messages():
                print(f"Server replied: {json.dumps(response)}")

    print("Connection closed.")


if __name__ == "__main__":
    main()
