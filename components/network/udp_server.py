"""
udp_server.py
Exemplo para disciplina de Sistemas em Tempo Real
Prof. Felipe Viel
Curso de Engenharia de Computação da Univali
Exemplo de Servidor utilizando protocolo de transporte UDP.
A ESP32 deverá se conectar nele usando o IP do computador usando o servidor.
    - Use ipconfig no Windows e ifconfig no Linux para saber o IP do seu computador (com a placa de rede adequada)
o IPv4 0.0.0.0 indica que o servidor irá escutar qualquer IP que enviar pacote para para na porta indicada.
A porta, que nesse exemplo é 6010, pode ser alterada para qualquer número dentro do intervalo de 2^16.
"""
import socket
import json
import time

ADDR = ("0.0.0.0", 3333)  # mesma porta usada no ESP32 (REPORT_UDP_PORT)
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind(ADDR)

print("Servidor UDP aguardando em", ADDR)

while True:
    data, src = s.recvfrom(2048)
    msg = data.decode(errors="ignore")

    print(f"De {src} => {msg}")

    ack = {
        "ack": True,
        "srv_time_us": int(time.time() * 1_000_000),
    }
    s.sendto(json.dumps(ack).encode(), src)
    print(f"ACK enviado para {src}")
