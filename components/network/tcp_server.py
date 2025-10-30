import socket
import json
import time

TCP_IP = "0.0.0.0"     # Escuta todas as interfaces
TCP_PORT = 6010        # Mesma porta usada no ESP32

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.bind((TCP_IP, TCP_PORT))
server.listen(1)
print(f"Servidor TCP aguardando conexão em {TCP_IP}:{TCP_PORT}...")

while True:
    conn, addr = server.accept()
    print(f"Cliente conectado: {addr}")

    with conn:
        while True:
            data = conn.recv(1024)
            if not data:
                print("Cliente desconectado.")
                break

            try:
                msg = json.loads(data.decode())
                print("Recebido JSON:", msg)
            except json.JSONDecodeError:
                print("Recebido não-JSON:", data)
                continue

            ack = {
                "ack": True,
                "srv_time_us": int(time.time() * 1_000_000),  # timestamp em microssegundos
                "recv_seq": msg.get("seq", None),
                "recv_proto": msg.get("protocol", "unknown")
            }

            conn.sendall(json.dumps(ack).encode())