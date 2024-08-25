import socket

server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_sock.bind(("169.254.112.189", 14000))
server_sock.listen(1)

(conn, addr) = server_sock.accept()
with conn:
    print(f"{addr} connected")
    while True:
        data = conn.recv(1024)
        print(data)