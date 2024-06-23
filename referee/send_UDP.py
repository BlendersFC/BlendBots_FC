import socket
UDP_IP = "192.168.56.1"

UDP_PORT_answer = 3939

sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
sock.bind((UDP_IP,UDP_PORT_answer))

while True:
    sock.sendto(b"RGrt",("192.168.56.1", UDP_PORT_answer))
    print("funciona")
