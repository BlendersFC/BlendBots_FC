#!/usr/bin/env python

import rospy
from soccer_pkg.msg import referee
import socket
import struct 


def main():
    #dictionary for penalty
    def get_pen(byte):
        pen = {
            255:"unknown",
            0:"none",
            14:"substitute",
            30:"pushing",
            31:"ball_manipulation",
            34:"pickup/incapable",
            35:"service"
        }
        return pen.get(byte)
    #dictionary for submode
    def get_submode(byte):
    	submode = {
    	0: "still", # 0 = Robots should stay still, referee places the ball on the ground
    	1: "prepare", #1 = Robots can place themselves toward the ball
    	2: "still" #2 = Robots should stay still and referees ask to remove illegally positioned robots
    	}
        return submode.get(byte)

    def get_game_state(byte):
        state = {
        -1: "quieto",
        0: "quieto",
        1: "Ready",
        2: "quieto",
        3: "Playing",
        4: "quieto"
        }
        return state.get(byte, "Impossible")

    #dictionary for game type
    def get_game_type(byte):
        gtype = {
        -1: "Unknown",
        0: "round robin",
        1: "playoff",
        2: "drop-in"
        }
        return gtype.get(byte, "Unknown")


    #secondary state library
    def get_s_state(byte):
        s_state = {
        0: "Normal",
        1: "Penalty Shoot",
        2: "Overtime",
        3: "Timeout",
        4: "Direct Free Kick",
        5: "Indirect Free Kick",
        6: "Penalty Kick",
        7: "Corner Kick",
        8: "Goal Kick",
        9: "Throw-In"
        }
        return s_state.get(byte, "Normal")


    # Node
    rospy.init_node('game_control', anonymous=True)
    # publisher
    pub = rospy.Publisher('r_data', referee, queue_size=1)
    rate = rospy.Rate(10)  # (10 Hz)
    
    
    UDP_IP = "192.168.0.2" #hearing referee IP
    UDP_PORT = 3838 #communication port listen
    UDP_PORTR = 3939 #return data port 

    # Datos de cada jugador
    header = b'RGrt'    # Header "RGrt"
    version = 2         # Versión de la estructura de datos
    team = 1            # Número de equipo
    player = 5          # Número de jugador
    message = 3      # Mensaje (0: GAMECONTROLLER_RETURN_MSG_ALIVE, 1: GAMECONTROLLER_RETURN_MSG_MAN_PENALISE, 2: GAMECONTROLLER_RETURN_MSG_MAN_UNPENALISE)


    # Empaquetar los datos en formato binario utilizando struct
    packed_data = struct.pack('<4sBBBB', header, version, team, player, message)

    sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)  #new datagram socket IPv4 
    sock_return = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    

    sock.bind((UDP_IP,UDP_PORT)) #associate socket with IP and port
   # rospy.loginfo(f"escuchando IP{UDP_IP} al puerto {UDP_PORT}")
    r_var= referee() #initialize referee variable
    r_var.I=0
    r_var.important = "quieto"
    rospy.loginfo("Mensaje creado")


    n=0 #variable para recorrer por cada jugador 0 es player1, 1 es player2...


    while not rospy.is_shutdown():
        
        data, addr = sock.recvfrom(1024) #1024 bytes
        teaminfo = data[24:29] 
        playerinfo = data[290+n*6:296+n*6]
        rospy.loginfo("si funciona")
        #first part
        protocol1, protocol2, r_var.packet_number, r_var.players_per_team, g_type, state, r_var.first_half, r_var.kick_off_team, s_state, r_var.team_p, submode = struct.unpack('11B',data[4:15])
        #second part
        r_var.drop_in_team, di_time1, di_time2, time1, time2, stime1, stime2 = struct.unpack('7B',data[17:24])

        if(r_var.first_half==1): #los bytes cambian en el cambio de cancha
            teaminfo = data[24:29]
            playerinfo = data[290+n*6:296+n*6]
        else:
            teaminfo = data[356:361]
            playerinfo = data[622+n*6:628+n*6]

        #team info 356 to 361 for b team
        r_var.team_n, r_var.team_c, r_var.score, r_var.p_shoot, r_var.coach_s = struct.unpack('5B',teaminfo)
        #player info 622+n*6:628 for b team 
        penalty, r_var.time_p_1, r_var.warnings_1, r_var.ycards_1, r_var.rcards_1, r_var.gkeeper_1 = struct.unpack('6B',playerinfo)


        #16 bits values and dictionaries 
        r_var.protocol_version = protocol1 | (protocol2<<8)
        r_var.game_type = get_game_type(g_type)
        r_var.state = get_game_state(state)
        r_var.secondary_state=get_s_state(s_state)
        r_var.submode = get_submode(submode)
        r_var.drop_in_time = di_time1 | (di_time2<<8)
        r_var.secs_remaining = time1|(time2<<8)
        r_var.secondary_time = stime1|(stime2<<8)
        r_var.penalty_1=get_pen(penalty)
        if (r_var.state == "quieto") | (r_var.rcards_1 == 1) | (r_var.time_p_1 != 0) | (r_var.secondary_state == "Timeout") | ((r_var.submode == "still") & (r_var.secondary_state !="Normal")):
             r_var.important = "quieto"
             r_var.I = 0
        if (r_var.state == "Ready"):
            r_var.important = "acomodate"
            r_var.I = 1
        if (r_var.state == "Playing") & (r_var.secondary_state == "Normal") & (r_var.rcards_1 == 0) & (r_var.time_p_1 == 0) :
            r_var.important = "playing"
            r_var.I = 2
        if (r_var.secondary_state != "Normal") & (r_var.team_p == 10) & (r_var.submode != "still"):
            r_var.important = "acercate"
            r_var.I = 3
        if (r_var.secondary_state != "Normal") & (r_var.team_p != 10) & (r_var.submode != "still"):
            r_var.important = "alejate"
            r_var.I = 4

        sock_return.sendto(packed_data, (UDP_IP,UDP_PORTR))
        pub.publish(r_var)
        rospy.loginfo(r_var.secs_remaining)





if __name__ == '__main__':
    main()