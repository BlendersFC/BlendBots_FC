#include <google/protobuf/timestamp.pb.h>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <ros/ros.h>


#include <cstring>
#include <arpa/inet.h>
#include <unistd.h>

#include "robocup.pb.h"
#include <referee_pkg/referee.h>
#include <referee_pkg/OP3_player.h>
#include "utils.hpp"


int test;
int test2;
referee_pkg::OP3_player dato;

// Create a new message which contains extensions
robocup::humanoid::Message msg;

void receive()
{
    // Crear un socket UDP
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock == -1)
    {
        std::cerr << "Error al crear el socket" << std::endl;
        return;
    }

    // Configurar la dirección del servidor
    sockaddr_in serverAddr;
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_addr.s_addr = INADDR_ANY;
    serverAddr.sin_port = htons(9000); // Puerto del servidor

    // Enlazar el socket a la dirección
    if (bind(sock, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) == -1)
    {
        std::cerr << "Error al enlazar el socket" << std::endl;
        close(sock);
        return;
    }

    std::string r_file = "/home/student/Downloads/r2file.pb"; // Ruta del archivo de salida

    // Recibir los datos
    char buffer[1024];
    ssize_t receivedBytes = recv(sock, buffer, sizeof(buffer), 0);
    if (receivedBytes == -1)
    {
        std::cerr << "Error al recibir los datos" << std::endl;
    }
    else
    {
        // Guardar los datos en el archivo
        std::ofstream outFile(r_file, std::ios::binary);
        outFile.write(buffer, receivedBytes);
        std::cout << "Datos recibidos y guardados en " << r_file << std::endl;
    }

    // Cerrar el socket
    close(sock);
}
void sendpb()
{
    // Crear un socket UDP
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock == -1)
    {
        std::cerr << "Error al crear el socket" << std::endl;
        return;
    }

    // Configurar la dirección del destinatario
    sockaddr_in destAddr;
    destAddr.sin_family = AF_INET;
    destAddr.sin_addr.s_addr = inet_addr("127.0.0.1"); // Dirección IP del host receptor
    destAddr.sin_port = htons(9000); // Puerto del host receptor

    // Leer el archivo .pb (reemplaza 'test.pb' con el nombre real)
    std::ifstream file("/home/student/catkin_ws/prueba1.pb", std::ios::binary);
    if (!file)
    {
        std::cerr << "Error al abrir el archivo .pb" << std::endl;
        close(sock);
        return;
    }

    // Obtener el tamaño del archivo
    file.seekg(0, std::ios::end);
    size_t fileSize = file.tellg();
    file.seekg(0, std::ios::beg);

    // Leer los datos del archivo
    std::string data(fileSize, '\0');
    file.read(&data[0], fileSize);

    // Enviar los datos
    ssize_t sentBytes = sendto(sock, data.c_str(), data.size(), 0, (struct sockaddr*)&destAddr, sizeof(destAddr));
    if (sentBytes == -1)
    {
        std::cerr << "Error al enviar los datos" << std::endl;
    }
    else
    {
        std::cout << "Archivo .pb enviado correctamente" << std::endl;
    }

    // Cerrar el socket
    close(sock);

}


void encode(const std::string& msg_file){


    // Set the transmission timestamp
    auto d       = std::chrono::system_clock::now().time_since_epoch();
    auto seconds = std::chrono::duration_cast<std::chrono::seconds>(d);
    auto nanos   = std::chrono::duration_cast<std::chrono::nanoseconds>(d - seconds);
    msg.mutable_timestamp()->set_seconds(seconds.count());
    msg.mutable_timestamp()->set_nanos(nanos.count());

    // Set player details
    msg.mutable_current_pose()->set_player_id(15);
    msg.mutable_current_pose()->set_team(robocup::humanoid::Team::BLUE);
    msg.mutable_current_pose()->mutable_position()->set_x(1.0f);
    msg.mutable_current_pose()->mutable_position()->set_y(3.0f);
    msg.mutable_current_pose()->mutable_position()->set_z(M_PI / 3);
    msg.mutable_current_pose()->mutable_covariance()->mutable_x()->set_x(1.0f);
    msg.mutable_current_pose()->mutable_covariance()->mutable_x()->set_y(0.0f);
    msg.mutable_current_pose()->mutable_covariance()->mutable_x()->set_z(0.0f);
    msg.mutable_current_pose()->mutable_covariance()->mutable_y()->set_x(0.0f);
    msg.mutable_current_pose()->mutable_covariance()->mutable_y()->set_y(1.0f);
    msg.mutable_current_pose()->mutable_covariance()->mutable_y()->set_z(0.0f);
    msg.mutable_current_pose()->mutable_covariance()->mutable_z()->set_x(0.0f);
    msg.mutable_current_pose()->mutable_covariance()->mutable_z()->set_y(0.0f);
    msg.mutable_current_pose()->mutable_covariance()->mutable_z()->set_z(1.0f);

    // Set ball details
    msg.mutable_ball()->mutable_position()->set_x(2.0f);
    msg.mutable_ball()->mutable_position()->set_y(3.0f);
    msg.mutable_ball()->mutable_position()->set_z(1.0f);
    msg.mutable_ball()->mutable_velocity()->set_x(0.5f);
    msg.mutable_ball()->mutable_velocity()->set_y(0.5f);
    msg.mutable_ball()->mutable_velocity()->set_z(-0.5f);
    msg.mutable_ball()->mutable_covariance()->mutable_x()->set_x(1.0f);
    msg.mutable_ball()->mutable_covariance()->mutable_x()->set_y(0.0f);
    msg.mutable_ball()->mutable_covariance()->mutable_x()->set_z(0.0f);
    msg.mutable_ball()->mutable_covariance()->mutable_y()->set_x(0.0f);
    msg.mutable_ball()->mutable_covariance()->mutable_y()->set_y(1.0f);
    msg.mutable_ball()->mutable_covariance()->mutable_y()->set_z(0.0f);
    msg.mutable_ball()->mutable_covariance()->mutable_z()->set_x(0.0f);
    msg.mutable_ball()->mutable_covariance()->mutable_z()->set_y(0.0f);
    msg.mutable_ball()->mutable_covariance()->mutable_z()->set_z(1.0f);

    // Set walk command
    msg.mutable_walk_command()->set_x(0.15f);
    msg.mutable_walk_command()->set_y(0.15f);
    msg.mutable_walk_command()->set_z(-0.05f);

    // Set target pose
    msg.mutable_target_pose()->set_player_id(3);
    msg.mutable_target_pose()->set_team(robocup::humanoid::Team::BLUE);
    msg.mutable_target_pose()->mutable_position()->set_x(2.0f);
    msg.mutable_target_pose()->mutable_position()->set_y(3.0f);
    msg.mutable_target_pose()->mutable_position()->set_z(-M_PI / 3);

    // Set kick target
    msg.mutable_kick_target()->set_x(4.5f);
    msg.mutable_kick_target()->set_y(0.0f);

    // Set state
    msg.set_state(robocup::humanoid::State::UNPENALISED);

    // Set some others details
    robocup::humanoid::Robot* other = msg.add_others();
    other->set_player_id(20);
    other->set_team(robocup::humanoid::Team::BLUE);
    other->mutable_position()->set_x(1.0f);
    other->mutable_position()->set_y(-3.0f);
    other->mutable_position()->set_z(M_PI / 6);
    other->mutable_covariance()->mutable_x()->set_x(1.0f);
    other->mutable_covariance()->mutable_x()->set_y(0.0f);
    other->mutable_covariance()->mutable_x()->set_z(0.0f);
    other->mutable_covariance()->mutable_y()->set_x(0.0f);
    other->mutable_covariance()->mutable_y()->set_y(1.0f);
    other->mutable_covariance()->mutable_y()->set_z(0.0f);
    other->mutable_covariance()->mutable_z()->set_x(0.0f);
    other->mutable_covariance()->mutable_z()->set_y(0.0f);
    other->mutable_covariance()->mutable_z()->set_z(2.0f);

    // ******************************
    // * Official message ends here *
    // ******************************

    // Dump serialised message to file
    std::ofstream ofs(msg_file, std::ofstream::binary);
    std::string string_msg;
    msg.SerializeToString(&string_msg);
    ofs.write(string_msg.data(), string_msg.size());
    ofs.close();

}







void decode(const std::string& msg_file) {

    // Open the extended message and parse it
    std::ifstream ifs(msg_file, std::ifstream::binary);
    robocup::humanoid::Message r_msg;
    robocup::humanoid::Robot* arabe = r_msg.add_others();
    if (ifs.is_open()) {
        // "Send" the message over the network and decode it
        std::stringstream stream;
        stream << ifs.rdbuf();
        //parse_message<robocup::humanoid::Message>(stream.str());
            if(r_msg.ParseFromString(stream.str()))
	   {
	    test=r_msg.mutable_current_pose()->player_id();
	    test2 = arabe->player_id();
	    std::cout<<test<<std::endl;
	    std::cout<<test2<<std::endl;
	    }
	    }
    else {
        std::cerr << "Error al abrir el archivo: " << msg_file << std::endl;
    }
    dato.id_teammate = test2;
}


void myinfoCB(const referee_pkg::referee::ConstPtr& mymsg){
    asm("NOP");
    //int test = mymsg->secs_remaining;
    
}
int main(int argc, char **argv) {
    
    //ROS
    ros::init(argc, argv, "referee_node");
    ros::NodeHandle nh;
    ros::Subscriber sub_op3 = nh.subscribe("/r_data", 1, myinfoCB); //suscriptor a sì mismo para enviar a sus compañeros
    ros:: Rate loop_rate(1);
    
    ros::Publisher pub_op3;
    pub_op3 = nh.advertise<referee_pkg::OP3_player>("his_data",1); //publicar a si mismo lo que recibe de los demàs 
    encode("prueba1.pb");
    //encode("/home/student/pb_msg/r2file.pb");
    //sendpb();
    decode("prueba1.pb");
    decode("/home/student/pb_msg/r2file.pb");
    //receive();
    //ROS
    while(ros::ok()){
    pub_op3.publish(dato);
    //ros::spin();
    }
    }
