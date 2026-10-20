
import socket

HOST = "127.0.0.1"
PORT = 5000


def main():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((HOST, PORT))
        server.listen(1)

        print(f"TCP server listening on {HOST}:{PORT}")
        print("Waiting for the operator client...")

        conn, address = server.accept()

        with conn:
            print(f"Connected to client: {address}")

            while True:
                data = conn.recv(1024)

                if not data:
                    break

                message = data.decode("utf-8")
                print(f"Client says: {message}")

                response = f"ACK: {message}"
                conn.sendall(response.encode("utf-8"))

        print("Client disconnected.")


if __name__ == "__main__":
    main()
