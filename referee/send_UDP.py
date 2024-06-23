import socket
import struct
UDP_IP = "127.0.0.1"

UDP_PORT_answer = 3939


header ="RGrt"
protocol_v = 12
team = 5
player = 1
message = 3

data_packed = struct.pack('4s4B', header.encode(),protocol_v,team,player,message)

sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
sock.bind((UDP_IP,UDP_PORT_answer))

while True:
    sock.sendto(data_packed,(UDP_IP, UDP_PORT_answer))
    print(data_packed)
