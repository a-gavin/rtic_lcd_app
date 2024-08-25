import socket
from time import sleep

server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_sock.bind(("169.254.112.189", 14000))
server_sock.listen(1)

(conn, addr) = server_sock.accept()
with conn:
    print(f"{addr} connected")
    while True:
        conn.send("Hey".encode())
        print("Hey")
        sleep(5)
        conn.send("bestie".encode())
        print("bestie")
        sleep(5)