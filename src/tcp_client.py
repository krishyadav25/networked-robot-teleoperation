
import socket

HOST = "127.0.0.1"
PORT = 5000


def main():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client:
        client.connect((HOST, PORT))
        print("Connected to the robot server test.")

        while True:
            message = input("Enter a message (or 'quit' to exit): ")

            if message.lower() == "quit":
                break

            client.sendall(message.encode("utf-8"))

            response = client.recv(1024)
            print(f"Server replied: {response.decode('utf-8')}")


if __name__ == "__main__":
    main()
