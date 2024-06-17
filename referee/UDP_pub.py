#!/usr/bin/env python3
import rospy
from referee_pkg.msg import referee
import socket 


def main():
    #dictionary for submode
    def get_submode(byte):
    	submode = {
    	0: "still",
    	1: "prepare",
    	2: "still_2",
    	}
    	return submode.get(byte)
    	"""
         *                0 = Robots should stay still, referee places the ball on the ground
     *                1 = Robots can place themselves toward the ball
     *                2 = Robots should stay still and referees ask to remove illegally positioned robots
    """
    #dictionary for state	
    def get_game_state(byte):
        state = {
        -1: "Impossible",
        0: "Initial",
        1: "Ready",
        2: "Set",
        3: "Playing",
        4: "Finished"
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
    pub = rospy.Publisher('r_data', referee, queue_size=10)
    rate = rospy.Rate(10)  # (10 Hz)
    
    
    UDP_IP = "0.0.0.0" #hearing all interfaces
    UDP_PORT = 3838 #communication port

    sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)  #new datagram socket IPv4 
    sock.bind((UDP_IP,UDP_PORT)) #associate socket with IP and port
    rospy.loginfo(f"escuchando IP{UDP_IP} al puerto {UDP_PORT}")
    referee_var= referee() #initialize referee variable
    while not rospy.is_shutdown():
        
        data, addr = sock.recvfrom(1024) #1024 bytes
	# pending Header
        referee_var.protocol_version = data[4] | (data[5]<<8)
        referee_var.packet_number = data[6]
        referee_var.players_per_team = data[7]
        referee_var.game_type = get_game_type(data[8])
        referee_var.state = get_game_state(data[9])
        referee_var.first_half = data[10]
        referee_var.kick_off_team = data[11] 
        referee_var.secondary_state=get_s_state(data[12])
        referee_var.team_p = data[13] #team performing 
        referee_var.submode = get_submode(data[14])
        referee_var.drop_in_team = data[17] 
        referee_var.drop_in_time = data[18] | (data[19]<<8)
        referee_var.secs_remaining = data[20]|(data[21]<<8)
        referee_var.secondary_time = data[22]|(data[23]<<8)


        pub.publish(referee_var)





if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
