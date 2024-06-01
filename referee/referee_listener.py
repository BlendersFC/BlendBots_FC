#!/usr/bin/env python3
#coding=utf-8
import rospy
from std_msgs.msg import String
import socket 


def main():
    state="Impossible"

    # Inicializa el nodo
    rospy.init_node('referee_listener', anonymous=True)
    # Crea un publicador para el tópico
    pub = rospy.Publisher('Game_status', String, queue_size=10)
    rate = rospy.Rate(10)  # Frecuencia de publicación (10 Hz)
    UDP_IP = "0.0.0.0"

    UDP_PORT = 3838

    sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)    
    sock.bind((UDP_IP,UDP_PORT))
    rospy.loginfo(f"escuchando IP{UDP_IP} al puerto {UDP_PORT}")

    while not rospy.is_shutdown():
        data, addr = sock.recvfrom(1024)
        mensaje=data.decode('ISO-8859-1')
        state_hex = data[9]
        rospy.loginfo("funciona?")
        if state_hex==0:
            state="Initial"
        elif state_hex==1:
            state="Ready"
        elif state_hex==2:
            state="Set"
        elif state_hex==3:
            state="Playing"
        elif state_hex==4:
            state="Finished"
        pub.publish(state)



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
