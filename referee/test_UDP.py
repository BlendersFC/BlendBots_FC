UDP_IP = "0.0.0.0"

UDP_PORT = 3838

sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
sock.bind((UDP_IP,UDP_PORT))

while True:
    data, addr = sock.recvfrom(1024)
    mensaje=data.decode('ISO-8859-1')
    #print(f"primeros 10 bytes en hexadecimal: {data.hex()}")
    #print(f"Mensaje recibido: {mensaje} desde {addr}")
    # Convertir los primeros 10 bytes a binario
    for i in range(24): #24
        byte_hex = data[9]
        byte_bin = bin(byte_hex)[2:].zfill(8)  # Convierte a binario y rellena con ceros a la izquierda
        print(byte_hex)