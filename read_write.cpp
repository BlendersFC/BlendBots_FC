/* Authors: Pedro Deniz
            Marlene Cobian */

#include <std_srvs/Empty.h>
#include <chrono>
#include <thread>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Point.h>
#include <iostream>
#include <fstream>
#include <eigen3/Eigen/Eigen>

#include "robotis_controller_msgs/SetModule.h"
#include "robotis_controller_msgs/SyncWriteItem.h"
#include "robotis_math/robotis_linear_algebra.h"
#include "op3_action_module_msgs/IsRunning.h"

bool init_gazebo_engine(void);
void buttonHandlerCallback(const std_msgs::String::ConstPtr& msg);
void jointstatesCallback(const sensor_msgs::JointState::ConstPtr& msg);
void readyToDemo();
void setModule(const std::string& module_name);
void goInitPose();
double pos_r_sho_pitch(double actual, double fin);
bool isActionRunning();
void goAction(int page);

bool checkManagerRunning(std::string& manager_name);
void torqueOnAll();

std::vector<double> motores;
void CallBack(const sensor_msgs::JointState& posicion);
void callbackError(const geometry_msgs::Point& msg);
void callbackPosition(const geometry_msgs::Point& msg);
void findballCallBack(const std_msgs::Bool& ball);
void callbackImu(const sensor_msgs::Imu::ConstPtr& msg);

double r_hip_pitch;
double l_hip_pitch;
double r_knee;
double l_knee;
double r_ank_pitch;
double l_ank_pitch;

double limite = 0.000005;

int ult_pos;
double rest_inc = 0.2181;
//rest_inc =0.2618 15°
//double rest_inc_giro = 0.08726;
double rest_inc_giro = 0.2181;

double t_ref_ang;
double t_ref;
double act_val = 0;

bool a = true;

double errorx;
double errory;
double ang;

double positionx;
double positiony;
double area;

double head_pan;
double head_tilt;

double giro = 0;

bool patear = false;

double alpha = 0.4;
double pitch;

double rpy_orientation;
const double FALL_FORWARD_LIMIT = 55;
const double FALL_BACK_LIMIT = -55;
double present_pitch_;
int page;
int state;

const int row = 5700;
  int col = 14;
  float posiciones[row][col];
  
  const int row2 = 40;
  int col2 = 6;
  float posiciones2[row2][col2];

enum ControlModule
{
  None = 0,
  DirectControlModule = 1,
  Framework = 2,
};

const int SPIN_RATE = 30;
const bool DEBUG_PRINT = false;

ros::Publisher init_pose_pub;
ros::Publisher dxl_torque_pub;
ros::Publisher write_joint_pub;
ros::Publisher vision_case_pub;
ros::Publisher action_pose_pub;
ros::Subscriber read_joint_sub;
ros::Subscriber imu_sub;

ros::ServiceClient set_joint_module_client;
ros::ServiceClient is_running_client;

std_msgs::Bool vision_case;

int control_module = None;
bool demo_ready = false;

bool find_ball = true;

//node main
int main(int argc, char **argv)
{
  
  //init ros
  ros::init(argc, argv, "read_write");
  ros::NodeHandle nh(ros::this_node::getName());

  ros::Subscriber error_sub = nh.subscribe("/error", 5, callbackError);
  ros::Subscriber position_sub = nh.subscribe("/position", 5, callbackPosition);
  ros::Subscriber joint_error_sub = nh.subscribe("/robotis/present_joint_states", 5, CallBack);
  ros::Subscriber find_ball_sub = nh.subscribe("/find_ball", 5, findballCallBack);
  ros::Subscriber imu_sub = nh.subscribe("/robotis/open_cr/imu", 1, callbackImu);
  
  std::string line;
  /*std::ifstream myfile ("/home/robotis/nobios/src/op3_leo/data/FLDeg.txt");
  if (myfile.is_open()){
	std::cout << "El archivo se abriò";
	
		for (int idx = 0; idx < row; idx++){
			for (int idy = 0; idy < col; idy++){
				myfile >> posiciones[idx][idy];
			}	
		}
		myfile.close();
  }else{
  	std::cout << "no abriò";
  }*/
  
  std::ifstream myfile ("/home/robotis/nayarit_ws/src/op3_leo/data/Pararse.txt");
  if (myfile.is_open()){
	std::cout << "El archivo se abriò";
	
		for (int idx2 = 0; idx2 < row2; idx2++){
			for (int idy2 = 0; idy2 < col2; idy2++){
				myfile >> posiciones2[idx2][idy2];
			}
			
		}
		/*for (int idx = 0; idx < row; idx++){
			for (int idy = 0; idy < col; idy++){
				std::cout  << posiciones[idx][idy] << std::endl;
			}
			
		}*/
		myfile.close();
  }else{
  	std::cout << "no abriò";
  }
  

  init_pose_pub = nh.advertise<std_msgs::String>("/robotis/base/ini_pose", 0);
  dxl_torque_pub = nh.advertise<std_msgs::String>("/robotis/dxl_torque", 0);
  write_joint_pub = nh.advertise<sensor_msgs::JointState>("/robotis/set_joint_states", 0);
  vision_case_pub = nh.advertise<std_msgs::Bool>("/vision_case", 1000);
  action_pose_pub = nh.advertise<std_msgs::Int32>("/robotis/action/page_num", 0);
  
  //Subscriber
  ros::Subscriber lectura = nh.subscribe("/robotis/present_joint_states",1,CallBack);
  

  // service
  set_joint_module_client = nh.serviceClient<robotis_controller_msgs::SetModule>("/robotis/set_present_ctrl_modules");
  is_running_client = nh.serviceClient<op3_action_module_msgs::IsRunning>("/robotis/action/is_running");

  ros::start();

  //set node loop rate
  ros::Rate loop_rate(SPIN_RATE);

  // wait for starting of op3_manager
  std::string manager_name = "/op3_manager";
  while (ros::ok())
  {
    ros::Duration(1.0).sleep();

    if (checkManagerRunning(manager_name) == true)
    {
      break;
      ROS_INFO_COND(DEBUG_PRINT, "Succeed to connect");
    }
    ROS_WARN("Waiting for op3 manager");
  }

  readyToDemo();

  //node loop
  sensor_msgs::JointState write_msg;
  write_msg.header.stamp = ros::Time::now();
  
  //Pararse en posiciòn para caminar
  ros::Duration(1).sleep();
  ros::Rate loop_rate_pararse(60);
  
  for (int fila2=0; fila2<row2; fila2++){
    write_msg.name.push_back("r_ank_pitch");
    write_msg.position.push_back(posiciones2[fila2][0]);
    write_msg.name.push_back("r_knee");
    write_msg.position.push_back(posiciones2[fila2][1]);
    write_msg.name.push_back("r_hip_pitch");
    write_msg.position.push_back(posiciones2[fila2][2] + rest_inc);
    write_msg.name.push_back("l_ank_pitch");
    write_msg.position.push_back(posiciones2[fila2][3]);
    write_msg.name.push_back("l_knee");
    write_msg.position.push_back(posiciones2[fila2][4]);
    write_msg.name.push_back("l_hip_pitch");
    write_msg.position.push_back(posiciones2[fila2][5] - rest_inc);
    write_joint_pub.publish(write_msg);
      
    loop_rate_pararse.sleep();
    ult_pos = fila2;
  
  }
  
 //////////////////////////////////////////////// Acomodo de pies ////////////////////////////////////////////////
  ros::Duration(1).sleep();
  write_msg.name.push_back("l_hip_yaw");
  write_msg.position.push_back(-0.0873);
  write_msg.name.push_back("r_hip_yaw");
  write_msg.position.push_back(0.0873);
  write_joint_pub.publish(write_msg);
    
  ros::Duration(1).sleep();
  write_msg.name.push_back("l_hip_roll");
  write_msg.position.push_back(-0.0873);
  write_msg.name.push_back("r_hip_roll");
  write_msg.position.push_back(0.0873);
  write_msg.name.push_back("l_ank_roll");
  write_msg.position.push_back(-0.0873);
  write_msg.name.push_back("r_ank_roll");
  write_msg.position.push_back(0.0873);
  write_joint_pub.publish(write_msg);

 //////////////////////////////////////////////// Marcha en su lugar con giro ////////////////////////////////////////////////

  while (ros::ok()){
    ros::spinOnce();
    ros::Rate loop_rate(SPIN_RATE);

    write_msg.name.push_back("head_pan");
    write_msg.position.push_back(positionx);
    write_msg.name.push_back("head_tilt");
    write_msg.position.push_back(positiony);
    write_joint_pub.publish(write_msg);

    if (area > 35000 && head_pan > 0 && head_tilt < -1.05){

      std::cout  << "PATEAA SIUUUUUUU" << std::endl;

      ///////////////////// Patada larga //////////////////////////
      //Detenerse
      write_msg.name.push_back("r_ank_pitch");
      write_msg.position.push_back(posiciones2[ult_pos][0]);
      write_msg.name.push_back("r_knee");
      write_msg.position.push_back(posiciones2[ult_pos][1]);
      write_msg.name.push_back("r_hip_pitch");
      write_msg.position.push_back(posiciones2[ult_pos][2] + rest_inc);
      write_msg.name.push_back("l_ank_pitch");
      write_msg.position.push_back(posiciones2[ult_pos][3]);
      write_msg.name.push_back("l_knee");
      write_msg.position.push_back(posiciones2[ult_pos][4]);
      write_msg.name.push_back("l_hip_pitch");
      write_msg.position.push_back(posiciones2[ult_pos][5] - rest_inc);
      write_joint_pub.publish(write_msg);
      
      //Inclinarse para alinear el centro de masa
      ros::Duration(1).sleep();
          write_msg.name.push_back("l_hip_roll");
      write_msg.position.push_back(0.17);
          write_msg.name.push_back("r_hip_roll");
      write_msg.position.push_back(0.17);
      write_msg.name.push_back("r_ank_roll");
      write_msg.position.push_back(-0.15);
      write_msg.name.push_back("l_ank_roll");
      write_msg.position.push_back(-0.45);
      write_joint_pub.publish(write_msg);
      
      //Posiciòn de seguridad
      ros::Duration(0.1).sleep();
      write_msg.name.push_back("l_ank_pitch");
      write_msg.position.push_back(0.7091);
      write_msg.name.push_back("l_knee");
      write_msg.position.push_back(1.5287);
      write_msg.name.push_back("l_hip_pitch");
      write_msg.position.push_back(-0.9474 - rest_inc);
      write_msg.name.push_back("l_ank_roll");
      write_msg.position.push_back(0);
      write_joint_pub.publish(write_msg);
      
      //Patada
      ros::Duration(0.1).sleep();
      write_msg.name.push_back("l_ank_pitch");
      write_msg.position.push_back(0.0046);
      write_msg.name.push_back("l_knee");
      write_msg.position.push_back(0.7420);
      write_msg.name.push_back("l_hip_pitch");
      write_msg.position.push_back(-1.2287 - rest_inc);
      write_joint_pub.publish(write_msg);

      ros::Duration(0.05).sleep();
      write_msg.name.push_back("l_knee");
      write_msg.position.push_back(0);
      write_msg.name.push_back("l_hip_pitch");
      write_msg.position.push_back(-1.5 - rest_inc);
      write_joint_pub.publish(write_msg);
      
      //Posiciòn de seguridad
      ros::Duration(0.1).sleep();
      write_msg.name.push_back("l_ank_pitch");
      write_msg.position.push_back(0.7091);
      write_msg.name.push_back("l_knee");
      write_msg.position.push_back(1.8);
      write_msg.name.push_back("l_hip_pitch");
      write_msg.position.push_back(-1.4 - rest_inc);
      write_msg.name.push_back("l_ank_roll");
      write_msg.position.push_back(0);
      write_joint_pub.publish(write_msg);
      
      //Regreso
      ros::Duration(0.2).sleep();
      write_msg.name.push_back("l_hip_roll");
      write_msg.position.push_back(-0.0873);
      write_msg.name.push_back("r_hip_roll");
      write_msg.position.push_back(0.0873);
      write_msg.name.push_back("l_ank_roll");
      write_msg.position.push_back(-0.0873);
      write_msg.name.push_back("r_ank_roll");
      write_msg.position.push_back(0.0873);
      
      write_msg.name.push_back("l_ank_pitch");
      write_msg.position.push_back(posiciones2[ult_pos][3]);
      write_msg.name.push_back("l_knee");
      write_msg.position.push_back(posiciones2[ult_pos][4]);
      write_msg.name.push_back("l_hip_pitch");
      write_msg.position.push_back(posiciones2[ult_pos][5] - rest_inc);
      write_joint_pub.publish(write_msg);

      ros::Duration(1).sleep();
    
    }else if (area > 35000 && head_pan < 0 && head_tilt < -1.05){

      std::cout  << "PATEAA SIUUUUUUU" << std::endl;

      ///////////////////// Patada larga //////////////////////////
      //Detenerse
      write_msg.name.push_back("r_ank_pitch");
      write_msg.position.push_back(posiciones2[ult_pos][0]);
      write_msg.name.push_back("r_knee");
      write_msg.position.push_back(posiciones2[ult_pos][1]);
      write_msg.name.push_back("r_hip_pitch");
      write_msg.position.push_back(posiciones2[ult_pos][2] + rest_inc);
      write_msg.name.push_back("l_ank_pitch");
      write_msg.position.push_back(posiciones2[ult_pos][3]);
      write_msg.name.push_back("l_knee");
      write_msg.position.push_back(posiciones2[ult_pos][4]);
      write_msg.name.push_back("l_hip_pitch");
      write_msg.position.push_back(posiciones2[ult_pos][5] - rest_inc);
      write_joint_pub.publish(write_msg);
      
      //Inclinarse para alinear el centro de masa
      ros::Duration(1).sleep();
          write_msg.name.push_back("l_hip_roll");
      write_msg.position.push_back(-0.17);
          write_msg.name.push_back("r_hip_roll");
      write_msg.position.push_back(-0.17);
      write_msg.name.push_back("l_ank_roll");
      write_msg.position.push_back(0.15);
      write_msg.name.push_back("r_ank_roll");
      write_msg.position.push_back(0.45);
      write_joint_pub.publish(write_msg);
      
      //Posiciòn de seguridad
      ros::Duration(0.1).sleep();
      write_msg.name.push_back("r_ank_pitch");
      write_msg.position.push_back(-0.7091);
      write_msg.name.push_back("r_knee");
      write_msg.position.push_back(-1.5287);
      write_msg.name.push_back("r_hip_pitch");
      write_msg.position.push_back(0.9474 + rest_inc);
      write_msg.name.push_back("r_ank_roll");
      write_msg.position.push_back(0);
      write_joint_pub.publish(write_msg);
      
      //Patada
      ros::Duration(0.1).sleep();
      
      write_msg.name.push_back("r_ank_pitch");
      write_msg.position.push_back(-0.0046);
      write_msg.name.push_back("r_knee");
      write_msg.position.push_back(-0.7420);
      write_msg.name.push_back("r_hip_pitch");
      write_msg.position.push_back(1.2287 + rest_inc);
      write_joint_pub.publish(write_msg);

      ros::Duration(0.05).sleep();
      write_msg.name.push_back("r_knee");
      write_msg.position.push_back(0);
      write_msg.name.push_back("r_hip_pitch");
      write_msg.position.push_back(1.5 + rest_inc);
      write_joint_pub.publish(write_msg);
      
      //Posiciòn de seguridad
      ros::Duration(0.1).sleep();
      write_msg.name.push_back("r_ank_pitch");
      write_msg.position.push_back(-0.7091);
      write_msg.name.push_back("r_knee");
      write_msg.position.push_back(-1.8);
      write_msg.name.push_back("r_hip_pitch");
      write_msg.position.push_back(1.4 + rest_inc);
      write_msg.name.push_back("r_ank_roll");
      write_msg.position.push_back(0);
      write_joint_pub.publish(write_msg);
      
      //Regreso
      ros::Duration(0.2).sleep();
      write_msg.name.push_back("l_hip_roll");
      write_msg.position.push_back(-0.0873);
      write_msg.name.push_back("r_hip_roll");
      write_msg.position.push_back(0.0873);
      write_msg.name.push_back("l_ank_roll");
      write_msg.position.push_back(-0.0873);
      write_msg.name.push_back("r_ank_roll");
      write_msg.position.push_back(0.0873);
      
      write_msg.name.push_back("r_ank_pitch");
      write_msg.position.push_back(posiciones2[ult_pos][0]);
      write_msg.name.push_back("r_knee");
      write_msg.position.push_back(posiciones2[ult_pos][1]);
      write_msg.name.push_back("r_hip_pitch");
      write_msg.position.push_back(posiciones2[ult_pos][2] + rest_inc);
      write_joint_pub.publish(write_msg);

      ros::Duration(1).sleep();
    
    }else if(head_pan > -0.4 && head_pan < 0.4 && (errorx > -40 && errorx < 40) && (errory > -40 && errory < 40)){
          
      std::cout  << "Caminando ando :p" << std::endl;
      //Pie izquierdo
      ros::Duration(0.1).sleep();
      write_msg.name.push_back("l_ank_pitch");
      write_msg.position.push_back(0.7520);
      write_msg.name.push_back("l_knee");
      write_msg.position.push_back(1.5317);
      write_msg.name.push_back("l_hip_pitch");
      write_msg.position.push_back(-0.8143-rest_inc);
      write_joint_pub.publish(write_msg);
      
      ros::Duration(0.1).sleep();
      write_msg.name.push_back("l_ank_pitch");
      write_msg.position.push_back(0.5486);
      write_msg.name.push_back("l_knee");
      write_msg.position.push_back(1.1446);
      write_msg.name.push_back("l_hip_pitch");
      write_msg.position.push_back(-0.6618-rest_inc);
      write_msg.name.push_back("r_ank_pitch");	//Pie derecho se acomoda para que centro de masa quede en medio de ambos pies
      write_msg.position.push_back(-0.5845);
      write_msg.name.push_back("r_knee");
      write_msg.position.push_back(-1.1453);
      write_msg.name.push_back("r_hip_pitch");
      write_msg.position.push_back(0.5233+rest_inc);
      write_joint_pub.publish(write_msg);
      
      //Pie derecho
      ros::Duration(0.1).sleep();
      write_msg.name.push_back("r_ank_pitch");
      write_msg.position.push_back(-0.7520);
      write_msg.name.push_back("r_knee");
      write_msg.position.push_back(-1.5317);
      write_msg.name.push_back("r_hip_pitch");
      write_msg.position.push_back(0.8143+rest_inc);
      write_joint_pub.publish(write_msg);
      
      ros::Duration(0.1).sleep();
      write_msg.name.push_back("r_ank_pitch");	
      write_msg.position.push_back(-0.5486);		//-0.5486  <- valor matlab
      write_msg.name.push_back("r_knee");
      write_msg.position.push_back(-1.1446);
      write_msg.name.push_back("r_hip_pitch");
      write_msg.position.push_back(0.6618+rest_inc);
      write_msg.name.push_back("l_ank_pitch");	//Pie izquierdo se acomoda para que centro de masa quede en medio de ambos pies
      write_msg.position.push_back(0.5845);
      write_msg.name.push_back("l_knee");
      write_msg.position.push_back(1.1453);
      write_msg.name.push_back("l_hip_pitch");
      write_msg.position.push_back(-0.5233-rest_inc);
      
      write_joint_pub.publish(write_msg);
        
    }else if (errorx > -5 && errorx < 5 && errory > -25 && errory < 25 && (head_pan < -0.1745 || head_pan > 0.1745)){

      //Detenerse
      write_msg.name.push_back("r_ank_pitch");
      write_msg.position.push_back(posiciones2[ult_pos][0]);
      write_msg.name.push_back("r_knee");
      write_msg.position.push_back(posiciones2[ult_pos][1]);
      write_msg.name.push_back("r_hip_pitch");
      write_msg.position.push_back(posiciones2[ult_pos][2] + rest_inc);
      write_msg.name.push_back("l_ank_pitch");
      write_msg.position.push_back(posiciones2[ult_pos][3]);
      write_msg.name.push_back("l_knee");
      write_msg.position.push_back(posiciones2[ult_pos][4]);
      write_msg.name.push_back("l_hip_pitch");
      write_msg.position.push_back(posiciones2[ult_pos][5] - rest_inc);
      write_joint_pub.publish(write_msg);

      ros::Duration(0.5).sleep();

      if (head_pan > 0){

        std::cout  << "Izquierda :0" << std::endl;
        //Levantar pie izquierdo
        ros::Duration(0.1).sleep();
        write_msg.name.push_back("l_ank_pitch");
        write_msg.position.push_back(0.7520);
        write_msg.name.push_back("l_knee");
        write_msg.position.push_back(1.5317);
        write_msg.name.push_back("l_hip_pitch");
        write_msg.position.push_back(-0.8143 - rest_inc_giro);
        write_msg.name.push_back("r_hip_yaw");
        write_msg.position.push_back(0.1746*1.5);
        write_msg.name.push_back("l_hip_yaw");
        write_msg.position.push_back(-0.1746*1.5);

        write_msg.name.push_back("l_hip_roll");
        write_msg.position.push_back(-0.0873);
        write_msg.name.push_back("r_hip_roll");
        write_msg.position.push_back(0.0873);
        write_msg.name.push_back("l_ank_roll");
        write_msg.position.push_back(-0.0873);
        write_msg.name.push_back("r_ank_roll");
        write_msg.position.push_back(0.0873);
        write_joint_pub.publish(write_msg);
        

        //Bajar pie izquierdo
        ros::Duration(0.1).sleep();
        write_msg.name.push_back("l_ank_pitch");
        write_msg.position.push_back(posiciones2[ult_pos][3]);
        write_msg.name.push_back("l_knee");
        write_msg.position.push_back(posiciones2[ult_pos][4]);
        write_msg.name.push_back("l_hip_pitch");
        write_msg.position.push_back(posiciones2[ult_pos][5]);
        write_joint_pub.publish(write_msg);
        
        //Levantar pie derecho
        ros::Duration(0.1).sleep();
        write_msg.name.push_back("r_ank_pitch");
        write_msg.position.push_back(-0.7520);
        write_msg.name.push_back("r_knee");
        write_msg.position.push_back(-1.5317);
        write_msg.name.push_back("r_hip_pitch");
        write_msg.position.push_back(0.8143 + rest_inc_giro);
        write_msg.name.push_back("l_hip_yaw");
        write_msg.position.push_back(0);
        write_msg.name.push_back("r_hip_yaw");
        write_msg.position.push_back(0);
        write_joint_pub.publish(write_msg);
        
        //Bajar pie izquierdo
        ros::Duration(0.1).sleep();
        write_msg.name.push_back("r_ank_pitch");
        write_msg.position.push_back(posiciones2[ult_pos][0]);
        write_msg.name.push_back("r_knee");
        write_msg.position.push_back(posiciones2[ult_pos][1]);
        write_msg.name.push_back("r_hip_pitch");
        write_msg.position.push_back(posiciones2[ult_pos][2] + rest_inc_giro);
        write_joint_pub.publish(write_msg);

      }else{

        std::cout  << "Derechaaaa D:" << std::endl;
        //Levantar pie derecho 
        ros::Duration(0.1).sleep();
        write_msg.name.push_back("r_ank_pitch");
        write_msg.position.push_back(-0.7091);
        write_msg.name.push_back("r_knee");
        write_msg.position.push_back(-1.4131);
        write_msg.name.push_back("r_hip_pitch");
        write_msg.position.push_back(0.7091 + rest_inc_giro);
        write_msg.name.push_back("r_hip_yaw");
        write_msg.position.push_back(0.1746*1.5);
        write_msg.name.push_back("l_hip_yaw");
        write_msg.position.push_back(-0.1746*1.5);

        write_msg.name.push_back("l_hip_roll");
        write_msg.position.push_back(-0.0873);
        write_msg.name.push_back("r_hip_roll");
        write_msg.position.push_back(0.0873);
        write_msg.name.push_back("l_ank_roll");
        write_msg.position.push_back(-0.0873);
        write_msg.name.push_back("r_ank_roll");
        write_msg.position.push_back(0.0873);
        write_joint_pub.publish(write_msg);

        //Bajar pie derecho
        ros::Duration(0.1).sleep();
        write_msg.name.push_back("r_ank_pitch");
        write_msg.position.push_back(posiciones2[ult_pos][0]);
        write_msg.name.push_back("r_knee");
        write_msg.position.push_back(posiciones2[ult_pos][1]);
        write_msg.name.push_back("r_hip_pitch");
        write_msg.position.push_back(posiciones2[ult_pos][2] + rest_inc_giro);
        write_joint_pub.publish(write_msg);
        
        //Levantar pie izquierdo
        ros::Duration(0.1).sleep();
        write_msg.name.push_back("l_ank_pitch");
        write_msg.position.push_back(0.7091);
        write_msg.name.push_back("l_knee");
        write_msg.position.push_back(1.4131);
        write_msg.name.push_back("l_hip_pitch");
        write_msg.position.push_back(-0.7091 - rest_inc_giro);
        write_msg.name.push_back("r_hip_yaw");
        write_msg.position.push_back(0);
        write_msg.name.push_back("l_hip_yaw");
        write_msg.position.push_back(0);
        write_joint_pub.publish(write_msg);
        
        //Bajar pie izquierdo
        ros::Duration(0.1).sleep();
        write_msg.name.push_back("l_ank_pitch");
        write_msg.position.push_back(posiciones2[ult_pos][3]);
        write_msg.name.push_back("l_knee");
        write_msg.position.push_back(posiciones2[ult_pos][4]);
        write_msg.name.push_back("l_hip_pitch");
        write_msg.position.push_back(posiciones2[ult_pos][5] - rest_inc_giro);
        write_joint_pub.publish(write_msg);
      
      }

    }else{
      std::cout  << "Quieto -_-" << std::endl;
      //Detenerse
      ros::Duration(0.1).sleep();
      write_msg.name.push_back("r_ank_pitch");
      write_msg.position.push_back(posiciones2[ult_pos][0]);
      write_msg.name.push_back("r_knee");
      write_msg.position.push_back(posiciones2[ult_pos][1]);
      write_msg.name.push_back("r_hip_pitch");
      write_msg.position.push_back(posiciones2[ult_pos][2] + rest_inc);
      write_msg.name.push_back("l_ank_pitch");
      write_msg.position.push_back(posiciones2[ult_pos][3]);
      write_msg.name.push_back("l_knee");
      write_msg.position.push_back(posiciones2[ult_pos][4]);
      write_msg.name.push_back("l_hip_pitch");
      write_msg.position.push_back(posiciones2[ult_pos][5] - rest_inc);
      write_joint_pub.publish(write_msg);
      ros::Duration(0.1).sleep();

      for (int i=0; i<4; i++){
        std::cout  << "Busco Derechaaa:" << std::endl;
        //Levantar pie derecho 
        ros::Duration(0.1).sleep();
        write_msg.name.push_back("r_ank_pitch");
        write_msg.position.push_back(-0.7091);
        write_msg.name.push_back("r_knee");
        write_msg.position.push_back(-1.4131);
        write_msg.name.push_back("r_hip_pitch");
        write_msg.position.push_back(0.7091 + rest_inc_giro);
        write_msg.name.push_back("r_hip_yaw");
        write_msg.position.push_back(0.1746*1.5);
        write_msg.name.push_back("l_hip_yaw");
        write_msg.position.push_back(-0.1746*1.5);

        write_msg.name.push_back("l_hip_roll");
        write_msg.position.push_back(-0.0873);
        write_msg.name.push_back("r_hip_roll");
        write_msg.position.push_back(0.0873);
        write_msg.name.push_back("l_ank_roll");
        write_msg.position.push_back(-0.0873);
        write_msg.name.push_back("r_ank_roll");
        write_msg.position.push_back(0.0873);
        write_joint_pub.publish(write_msg);

        //Bajar pie derecho
        ros::Duration(0.1).sleep();
        write_msg.name.push_back("r_ank_pitch");
        write_msg.position.push_back(posiciones2[ult_pos][0]);
        write_msg.name.push_back("r_knee");
        write_msg.position.push_back(posiciones2[ult_pos][1]);
        write_msg.name.push_back("r_hip_pitch");
        write_msg.position.push_back(posiciones2[ult_pos][2] + rest_inc_giro);
        write_joint_pub.publish(write_msg);
        
        //Levantar pie izquierdo
        ros::Duration(0.1).sleep();
        write_msg.name.push_back("l_ank_pitch");
        write_msg.position.push_back(0.7091);
        write_msg.name.push_back("l_knee");
        write_msg.position.push_back(1.4131);
        write_msg.name.push_back("l_hip_pitch");
        write_msg.position.push_back(-0.7091 - rest_inc_giro);
        write_msg.name.push_back("r_hip_yaw");
        write_msg.position.push_back(0);
        write_msg.name.push_back("l_hip_yaw");
        write_msg.position.push_back(0);
        write_joint_pub.publish(write_msg);
        
        //Bajar pie izquierdo
        ros::Duration(0.1).sleep();
        write_msg.name.push_back("l_ank_pitch");
        write_msg.position.push_back(posiciones2[ult_pos][3]);
        write_msg.name.push_back("l_knee");
        write_msg.position.push_back(posiciones2[ult_pos][4]);
        write_msg.name.push_back("l_hip_pitch");
        write_msg.position.push_back(posiciones2[ult_pos][5] - rest_inc_giro);
        write_joint_pub.publish(write_msg); 
    }

    std::cout  << "Quieto -_-" << std::endl;
      //Detenerse
      ros::Duration(0.1).sleep();
      write_msg.name.push_back("r_ank_pitch");
      write_msg.position.push_back(posiciones2[ult_pos][0]);
      write_msg.name.push_back("r_knee");
      write_msg.position.push_back(posiciones2[ult_pos][1]);
      write_msg.name.push_back("r_hip_pitch");
      write_msg.position.push_back(posiciones2[ult_pos][2] + rest_inc);
      write_msg.name.push_back("l_ank_pitch");
      write_msg.position.push_back(posiciones2[ult_pos][3]);
      write_msg.name.push_back("l_knee");
      write_msg.position.push_back(posiciones2[ult_pos][4]);
      write_msg.name.push_back("l_hip_pitch");
      write_msg.position.push_back(posiciones2[ult_pos][5] - rest_inc);
      write_joint_pub.publish(write_msg);
      ros::Duration(0.1).sleep();

  }
  return 0;
}

void CallBack(const sensor_msgs::JointState& posicion){
  head_pan = posicion.position[0];
  head_tilt = posicion.position[1];
}

void readyToDemo()
{
  ROS_INFO("Start Read-Write Demo");
  torqueOnAll();
  ROS_INFO("Torque on All joints");

  // send message for going init posture
  goInitPose();
  ROS_INFO("Go Init pose");

  // wait while ROBOTIS-OP3 goes to the init posture.
  ros::Duration(4.0).sleep();

  setModule("none");
}

void goInitPose()
{
  std_msgs::String init_msg;
  init_msg.data = "ini_pose";
  init_pose_pub.publish(init_msg);
}

void goAction(int page) {
  setModule("action_module");
  ROS_INFO("Action pose");

  std_msgs::Int32 action_msg;
  action_msg.data = page;
  action_pose_pub.publish(action_msg);
}

bool checkManagerRunning(std::string& manager_name)
{
  std::vector<std::string> node_list;
  ros::master::getNodes(node_list);

  for (unsigned int node_list_idx = 0; node_list_idx < node_list.size(); node_list_idx++)
  {
    if (node_list[node_list_idx] == manager_name)
      return true;
  }
  ROS_ERROR("Can't find op3_manager");
  return false;
}

void setModule(const std::string& module_name)
{
  robotis_controller_msgs::SetModule set_module_srv;
  set_module_srv.request.module_name = module_name;

  if (set_joint_module_client.call(set_module_srv) == false)
  {
    ROS_ERROR("Failed to set module");
    return;
  }
  return ;
}

void torqueOnAll()
{
  std_msgs::String check_msg;
  check_msg.data = "check";
  dxl_torque_pub.publish(check_msg);
}

bool isActionRunning() {
  op3_action_module_msgs::IsRunning is_running_srv;

  if (is_running_client.call(is_running_srv) == false) {
    ROS_ERROR("Failed to start action module");
    return true;
  } else {
    if (is_running_srv.response.is_running == true) {
      return true;
    }
  }
  return false;
}

void callbackError(const geometry_msgs::Point& msg)
{
  errorx = msg.x;
  errory = msg.y;
}

void callbackPosition(const geometry_msgs::Point& msg)
{
  positionx = msg.x;
  positiony = msg.y;
  area = msg.z;
}

void findballCallBack(const std_msgs::Bool& ball){
	find_ball = ball.data;
}

void callbackImu(const sensor_msgs::Imu::ConstPtr& msg) {
  Eigen::Quaterniond orientation(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
  Eigen::MatrixXd rpy_orientation = robotis_framework::convertQuaternionToRPY(orientation);
  rpy_orientation *= (180 / 3.141516);

  double pitch = rpy_orientation.coeff(1, 0);

  double alpha = 0.4;
  if (present_pitch_ == 0) 
    present_pitch_ = pitch;
  else
    present_pitch_ = present_pitch_ * (1 - alpha) + pitch * alpha;

  if (present_pitch_ > FALL_FORWARD_LIMIT) {
    goAction(122);
    setModule("none");
  } else if (present_pitch_ < FALL_BACK_LIMIT) {
    goAction(123);
    setModule("none");
  } else {
    state = 0;
  }
}