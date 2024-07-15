import socket
import struct
UDP_IP = "127.0.0.1"

UDP_PORT_answer = 3939

# Definición de los datos según la estructura
header = b'RGrt'    # Header "RGrt"
version = 2         # Versión de la estructura de datos
team = 1            # Número de equipo
player = 5          # Número de jugador
message = 1         # Mensaje (0: GAMECONTROLLER_RETURN_MSG_ALIVE, 1: GAMECONTROLLER_RETURN_MSG_MAN_PENALISE, 2: GAMECONTROLLER_RETURN_MSG_MAN_UNPENALISE)
#time = 12345        # Tiempo en segundos desde el comienzo del medio

# Empaquetar los datos en formato binario utilizando struct
packed_data = struct.pack('<4sBBBB', header, version, team, player, message)

# Crear un socket UDP
udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Dirección y puerto del receptor
server_address = ('127.0.0.1', 3939) #hay que cambiar la IP a la del árbitro de verdad

while True:
    # Enviar los datos empaquetados
    udp_socket.sendto(packed_data, server_address)
    print("Datos enviados")

