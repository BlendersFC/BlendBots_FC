/* Author: Marlene Cobian 
 Maintainer: Pedro Deniz */

#include <std_srvs/Empty.h>
#include <chrono>
#include <thread>

#include <ros/ros.h>
#include <cmath>
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
#include "op3_walking_module_msgs/GetWalkingParam.h"
#include "op3_walking_module_msgs/WalkingParam.h"
//#include "op3_online_walkin_module_msgs/GetJointPose.h"

void readyToDemo();
void setModule(const std::string& module_name);
bool checkManagerRunning(std::string& manager_name);
void torqueOnAll();

void goInitPose();
void goAction(int page);
void turn2search();
void goWalk(std::string& command);

bool isActionRunning();
bool getWalkingParam();
bool getJointPose();
void waitFollowing();

void calcFootstep(double target_distance, double target_angle, double delta_time, double& fb_move, double& rl_angle);
void setWalkingParam(double x_move, double y_move, double rotation_angle, bool balance = true);
bool processFollowing();
void waitFollowing();

void callbackImu(const sensor_msgs::Imu::ConstPtr& msg);
void callbackPosition(const geometry_msgs::Point& msg);
void callbackJointStates(const sensor_msgs::JointState& msg);
void callbackError(const geometry_msgs::Point& msg);
void callbackTurn(const std_msgs::Bool& msg);
void callbackfindBall(const std_msgs::Bool& msg);
void callbackSearchBall(const std_msgs::Bool& msg);
void callbackBall(const geometry_msgs::Point& msg);
void callbackSlope(const std_msgs::Float64& msg);

void buttonHandlerCallback(const std_msgs::String::ConstPtr& msg);

double rest_inc = 0.2181;
//rest_inc =0.2618 15°
double rest_inc_giro = 0.08726;

double alpha = 0.4;
double pitch;

double rpy_orientation;
const double FALL_FORWARD_LIMIT = 55;
const double FALL_BACK_LIMIT = -55;
double present_pitch_;
double head_pan;
double head_tilt;
double positionx;
double positiony;
double errorx;
double errory;
double ball_area;
double ball_slope;
double distance_to_ball = 0.0;
int page;
int state;
double ball_trigger_area;
bool right_kick;
bool left_kick;

int contr = 0;
int contl = 0;

std_msgs::String scan;

const int row = 5700;
const int col = 14;
float posiciones[row][col];

const int row2 = 40;
const int col2 = 6;
float posiciones2[row2][col2];

const double SPOT_FB_OFFSET = 0.0;
const double SPOT_RL_OFFSET = 0.0;
const double SPOT_ANGLE_OFFSET = 0.0;
double current_x_move_ = 0.005;
double current_r_angle_ = 0.0;
const double IN_PLACE_FB_STEP = -0.003;
const int NOT_FOUND_THRESHOLD = 0.5;
int count_not_found_ = 0;
int count_to_kick_ = 0;
const double CAMERA_HEIGHT = 0.46;
const double FOV_WIDTH = 35.2 * M_PI / 180;
const double FOV_HEIGHT = 21.6 * M_PI / 180;
const double hip_pitch_offset_ = 0.12217305;  //7°
const double UNIT_FB_STEP = 0.002;
const double UNIT_RL_TURN = 0.00872665;  //0.5°
const double MAX_FB_STEP = 0.007;
const double MAX_RL_TURN =  0.26179939;  //15°
const double MIN_FB_STEP = 0.003;
const double MIN_RL_TURN = 0.08726646;  //5°
double accum_period_time = 0.0;
double current_period_time = 0.6;

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
ros::Publisher write_head_joint_pub;
ros::Publisher write_joint_pub;
ros::Publisher vision_case_pub;
ros::Publisher action_pose_pub;
ros::Publisher walk_command_pub;
ros::Publisher set_walking_param_pub;
ros::Publisher reset_body_pub;
ros::Publisher scan_command;

ros::Subscriber read_joint_sub;
ros::Subscriber ball_sub;
ros::Subscriber slope_sub;
ros::Subscriber imu_sub;
ros::Subscriber position_sub;
ros::Subscriber error_sub;
ros::Subscriber search_ball_sub;
ros::Subscriber find_ball_sub;
ros::Subscriber button_sub;
ros::Subscriber turnNsearch_sub;

ros::ServiceClient set_joint_module_client;
ros::ServiceClient is_running_client;
ros::ServiceClient get_param_client;

op3_walking_module_msgs::WalkingParam current_walking_param;

int control_module = None;
bool demo_ready = false;

bool ball = false;
bool turnNsearch = false;
bool search_ball = false;
bool setup_kick_pos = false;
bool start_button_flag = false;

//node main
int main(int argc, char **argv)
{
  
  //init ros
  ros::init(argc, argv, "read_write");
  ros::NodeHandle nh(ros::this_node::getName());

  int robot_id;
  nh.param<int>("robot_id", robot_id, 0);

  //subscribers
  read_joint_sub = nh.subscribe("/robotis_" + std::to_string(robot_id) + "/present_joint_states",1, callbackJointStates);
  position_sub = nh.subscribe("/robotis_" + std::to_string(robot_id) + "/position", 5, callbackPosition);
  error_sub = nh.subscribe("/robotis_" + std::to_string(robot_id) + "/error", 5, callbackError);
  search_ball_sub = nh.subscribe("/search_ball", 5, callbackSearchBall);
  ball_sub = nh.subscribe("/robotis_" + std::to_string(robot_id) + "/BallCenter", 1, callbackBall);
  slope_sub = nh.subscribe("/robotis_" + std::to_string(robot_id) + "/Slope", 1, callbackSlope);
  imu_sub = nh.subscribe("/robotis_" + std::to_string(robot_id) + "/open_cr/imu", 1, callbackImu);
  find_ball_sub = nh.subscribe("/robotis_" + std::to_string(robot_id) + "/find_ball", 5, callbackfindBall);
  button_sub = nh.subscribe("/robotis_" + std::to_string(robot_id) + "/open_cr/button", 1, buttonHandlerCallback);
  turnNsearch_sub = nh.subscribe("/robotis_" + std::to_string(robot_id) + "/turnNsearch", 5, callbackTurn);
  
  std::string command;
  std::ifstream myfile ("/home/robotis/blenders_ws/src/soccer_pkg/data/Pararse.txt");
  if (myfile.is_open()) {
    std::cout << "El archivo se abrió";

    for (int idx2 = 0; idx2 < row2; idx2++){
      for (int idy2 = 0; idy2 < col2; idy2++){
        myfile >> posiciones2[idx2][idy2];
      }
      
    }
    myfile.close();
  } else {
    std::cout << "El archivo no abrió";
  }

  //publishers
  init_pose_pub = nh.advertise<std_msgs::String>("/robotis_" + std::to_string(robot_id) + "/base/ini_pose", 0);
  dxl_torque_pub = nh.advertise<std_msgs::String>("/robotis_" + std::to_string(robot_id) + "/dxl_torque", 0);
  write_head_joint_pub = nh.advertise<sensor_msgs::JointState>("/robotis_" + std::to_string(robot_id) + "/head_control/set_joint_states", 0);
  write_joint_pub = nh.advertise<sensor_msgs::JointState>("/robotis_" + std::to_string(robot_id) + "/set_joint_states", 0);
  action_pose_pub = nh.advertise<std_msgs::Int32>("/robotis_" + std::to_string(robot_id) + "/action/page_num", 0);
  walk_command_pub = nh.advertise<std_msgs::String>("/robotis_" + std::to_string(robot_id) + "/walking/command", 0);
  set_walking_param_pub = nh.advertise<op3_walking_module_msgs::WalkingParam>("/robotis_" + std::to_string(robot_id) + "/walking/set_params", 0);
  reset_body_pub = nh.advertise<std_msgs::Bool>("/robotis_" + std::to_string(robot_id) + "/online_walking/reset_body", 0);
  scan_command = nh.advertise<std_msgs::String>("/robotis_" + std::to_string(robot_id) + "/head_control/scan_command", 0);

  //services
  set_joint_module_client = nh.serviceClient<robotis_controller_msgs::SetModule>("/robotis_" + std::to_string(robot_id) + "/set_present_ctrl_modules");
  is_running_client = nh.serviceClient<op3_action_module_msgs::IsRunning>("/robotis_" + std::to_string(robot_id) + "/action/is_running");
  get_param_client = nh.serviceClient<op3_walking_module_msgs::GetWalkingParam>("/robotis_" + std::to_string(robot_id) + "/walking/get_params");

  ros::start();

  //set node loop rate
  ros::Rate loop_rate(SPIN_RATE);

  //wait for starting of op3_manager
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
  
  //pararse en posición para caminar
  ros::Duration(5).sleep();
  goAction(9);
  ros::Rate loop_rate_pararse(60);
  ros::Duration(5.0).sleep();
  ros::Time prev_time_ = ros::Time::now();
  
  
 
  while (ros::ok())
  {
    ros::Rate loop_rate(SPIN_RATE);
    ros::spinOnce();
    ros::Time prev_time_ = ros::Time::now();
    std::cout  << "ball area: " << ball_area << std::endl;

    if (start_button_flag == 1){
      asm("NOP");
      std::cout  << "MUEVETE" << std::endl; 
    }
    else {
      std::cout  << "nada" << std::endl; 
      continue;
    }
    
    
    if(!ball){
        setModule("head_control_module");
        
        if (!search_ball){
            write_msg.name.push_back("head_pan");
            write_msg.position.push_back(head_pan + positionx);
            write_msg.name.push_back("head_tilt");
            write_msg.position.push_back(head_tilt + positiony);
            write_head_joint_pub.publish(write_msg);
        }else{
            if (distance_to_ball < 0.43 && distance_to_ball != 0.0){
                positiony -= 0.17;
            }
            write_msg.name.push_back("head_pan");
            write_msg.position.push_back(positionx);
            write_msg.name.push_back("head_tilt");
            write_msg.position.push_back(positiony);
            write_head_joint_pub.publish(write_msg);
        }

        if (setup_kick_pos == 0){
          
        setModule("none");
                          //////////////////////////////////////////////// Acomodo de pies ////////////////////////////////////////////////
	        ros::Duration(0.5).sleep();
	        write_msg.name.push_back("l_hip_yaw");
	        write_msg.position.push_back(-0.0873);
	        write_msg.name.push_back("r_hip_yaw");
	        write_msg.position.push_back(0.0873);
     	    write_joint_pub.publish(write_msg);
     	
	        ros::Duration(0.1).sleep();
	        write_msg.name.push_back("l_hip_roll");
        	write_msg.position.push_back(-0.0873);
	        write_msg.name.push_back("r_hip_roll");
        	write_msg.position.push_back(0.0873);
        	write_msg.name.push_back("l_ank_roll");
        	write_msg.position.push_back(-0.0873);
        	write_msg.name.push_back("r_ank_roll");
        	write_msg.position.push_back(0.0873);
            write_joint_pub.publish(write_msg);


        }

        //goAction(9);
        //ros::Time prev_time_ = ros::Time::now();   
        //ros::spinOnce();
        //ros::Rate loop_rate(SPIN_RATE);
        
        //setme::now();
        setup_kick_pos = 1;

    }

    ball_trigger_area = 13500;

    if((ball_area > ball_trigger_area)){ //&& (ball)
    //std::cout  << "PELOTAA" << std::endl;
    contl+= 1;
      /*if (contl>25){
        contl = 0;
      }*/
      if (contl>0){
        right_kick = true;
        contl = 0;
      }
      else {
        right_kick = false;
        left_kick = false;
      }
        /*if (ball_slope >= 5){  //30°
          contr += 1;
          if ((contr >= 12)&&(contr > (contl + 3))){
            std::cout  << "SALTAR DERECHA" << std::endl;
            right_jump = true;
            left_jump = false;
            contr = 0;
            contl = 0;
          }
        }else if (ball_slope <= -5){
          contl += 1;
          if ((contl >= 12)&&(contl > (contr + 2))){
            std::cout  << "SALTAR IZQUIERDA" << std::endl;
            right_jump = false;
            left_jump = true;
            contr = 0;
            contl = 0;
          }
          
        }
      }*/
    }else{
      std::cout  << "No patees" << std::endl;
      right_kick = false;
      left_kick = false;
    }
      
    /*if (30<yaw<-30){
      goAction(60);
      ros::Duration(10).sleep();
    }else{
      while(yaw>30 or yaw<-30){
        turn2search();
      }
    }*/

    if(right_kick) {
      right_kick = false;
      left_kick = false;    
      //setModule("none");
            std::cout  << "PATADAAAAAAAAA" << std::endl; 
            //ros::Duration(0.1).sleep();
                        //Inclinarse para alinear el centro de masa
            /*goAction(1);
            ros::Duration(5.0).sleep();            
            	ros::Duration(0.5).sleep();*/
            write_msg.name.push_back("l_hip_roll");
            write_msg.position.push_back(-0.17);
            write_msg.name.push_back("r_hip_roll");
            write_msg.position.push_back(-0.17);
            write_msg.name.push_back("l_ank_roll");
            write_msg.position.push_back(0.15);
            write_msg.name.push_back("r_ank_roll");
            write_msg.position.push_back(0.15);
            write_joint_pub.publish(write_msg);
            ros::Duration(0.1).sleep();
            
            //Posiciòn de seguridad
            write_msg.name.push_back("r_ank_pitch");
            write_msg.position.push_back(-0.7091);
            write_msg.name.push_back("r_knee");
            write_msg.position.push_back(-1.5287);
            write_msg.name.push_back("r_hip_pitch");
            write_msg.position.push_back(0.9474 + rest_inc);
            write_msg.name.push_back("r_ank_roll");
            write_msg.position.push_back(0);
            write_joint_pub.publish(write_msg);
            ros::Time prev_time_ = ros::Time::now();

            //Patada
            ros::Duration(0.1).sleep();
            write_msg.name.push_back("r_ank_pitch");
            write_msg.position.push_back(-0.0046);
            write_msg.name.push_back("r_knee");
            write_msg.position.push_back(-0.7420);
            write_msg.name.push_back("r_hip_pitch");
            write_msg.position.push_back(1.2287 + rest_inc);
            write_joint_pub.publish(write_msg);

            ros::Duration(0.1).sleep();
            write_msg.name.push_back("r_knee");
            write_msg.position.push_back(0);
            write_msg.name.push_back("r_hip_pitch");
            write_msg.position.push_back(1.5 + rest_inc);
           
            //write_msg.name.push_back("r_hip_pitch");
            //write_msg.position.push_back(posiciones2[row2-1][2] + rest_inc);
            write_joint_pub.publish(write_msg);
            ros::Duration(.1).sleep();
            
            //Posiciòn de seguridad
            // cambio hip roll
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
            ros::Duration(0.1).sleep();
	        //write_msg.name.push_back("l_hip_yaw");
	        //write_msg.position.push_back(-0.0873);
	        //write_msg.name.push_back("r_hip_yaw");
	        //write_msg.position.push_back(0.0873);
     	    //write_joint_pub.publish(write_msg);

            //Regreso tuneado
            //ros::Duration(0.1).sleep();
            write_msg.name.push_back("l_hip_roll");
            write_msg.position.push_back(0.027625142989741086);
            write_msg.name.push_back("r_hip_roll");
            write_msg.position.push_back(0.027625142989741086);
            ros::Duration(0.1).sleep();
            //write_msg.name.push_back("r_ank_pitch");
            //write_msg.position.push_back(-0.7091);
            //write_msg.name.push_back("r_knee");
            //write_msg.position.push_back(-1.8);
            //write_msg.name.push_back("r_hip_pitch");
            //write_msg.position.push_back(1.4 + rest_inc);
            //write_msg.name.push_back("r_ank_roll");
            //write_msg.position.push_back(0);
            //write_joint_pub.publish(write_msg);
            /*write_msg.name.push_back("r_knee");
            write_msg.position.push_back(-1.2007185887207031);
            ros::Duration(0.1).sleep();
            write_msg.name.push_back("r_ank_pitch");
            write_msg.position.push_back(-0.404078533203126);
            ros::Duration(0.2).sleep();*/           

             //Regreso
            
            write_msg.name.push_back("l_ank_roll");
            write_msg.position.push_back(-0.03528155808105469);
            write_msg.name.push_back("r_ank_roll");
            write_msg.position.push_back(0.03683352398632145);
            write_msg.name.push_back("r_knee");
            write_msg.position.push_back(-1.0707185887207031);
            ros::Duration(0.1).sleep();
            write_msg.name.push_back("r_hip_pitch");
            write_msg.position.push_back(0.850968129628725);
            write_msg.name.push_back("r_ank_pitch");
            write_msg.position.push_back(-0.6504078533203126);
            write_msg.name.push_back("r_knee");
            write_msg.position.push_back(-1.5707185887207031);
            ros::Duration(0.1).sleep();
            write_msg.name.push_back("r_knee");
            write_msg.position.push_back(-1.5707185887207031);
            ros::Duration(0.1).sleep();
            write_msg.name.push_back("r_hip_pitch");
            write_msg.position.push_back(0.550968129628725);
            
            
            write_joint_pub.publish(write_msg);

            ros::Duration(0.1).sleep();
            write_msg.name.push_back("r_knee");
            write_msg.position.push_back(-1.0707185887207031);
            write_joint_pub.publish(write_msg);
      ros::Duration(4.0).sleep();
      setModule("none");
      setup_kick_pos = 0;

    }else if(left_kick){
      right_kick = false;
      left_kick = false;
      goAction(84);
      ros::Duration(5.0).sleep();
      setModule("none");      
    }
  }
	return 0;
}


void buttonHandlerCallback(const std_msgs::String::ConstPtr& msg)
{

  // in the middle of playing demo
  if (msg->data == "mode")
    {
      // go to mode selection status
      start_button_flag = 1;
    }
  else if (msg->data == "start"){
    start_button_flag = 0;
  }
}



void readyToDemo()
{
  ROS_INFO("Start read-write demo");
  torqueOnAll();
  ROS_INFO("Torque on all joints");

  //send message for going init posture
  goInitPose();
  ROS_INFO("Go init pose");

  //wait while ROBOTIS-OP3 goes to the init posture.
  ros::Duration(4.0).sleep();

  setModule("none");
}

void goInitPose()
{
  std_msgs::String init_msg;
  init_msg.data = "ini_pose";
  init_pose_pub.publish(init_msg);
}

void turn2search(){
  //node loop
  sensor_msgs::JointState write_msg;
  write_msg.header.stamp = ros::Time::now();
  
  for (int i = 1; i <= 3; i++)
  {
    setModule("none");
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
    write_msg.position.push_back(posiciones2[row2-1][0]);
    write_msg.name.push_back("r_knee");
    write_msg.position.push_back(posiciones2[row2-1][1]);
    write_msg.name.push_back("r_hip_pitch");
    write_msg.position.push_back(posiciones2[row2-1][2] + rest_inc_giro);
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
    write_msg.position.push_back(posiciones2[row2-1][3]);
    write_msg.name.push_back("l_knee");
    write_msg.position.push_back(posiciones2[row2-1][4]);
    write_msg.name.push_back("l_hip_pitch");
    write_msg.position.push_back(posiciones2[row2-1][5] - rest_inc_giro);
    write_joint_pub.publish(write_msg);
  }
}

void goAction(int page) 
{
  setModule("action_module");
  ROS_INFO("Action pose");

  std_msgs::Int32 action_msg;
  action_msg.data = page;
  action_pose_pub.publish(action_msg);
}

void goWalk(std::string& command) 
{
  setModule("walking_module");
  if (command == "start") {
    getWalkingParam();
    setWalkingParam(IN_PLACE_FB_STEP, 0, 0, true);
  }

  std_msgs::String command_msg;
  command_msg.data = command;
  walk_command_pub.publish(command_msg);
}

void waitFollowing() 
{
  count_not_found_++;

  if (count_not_found_ > NOT_FOUND_THRESHOLD * 0.5)
    setWalkingParam(0.0, 0.0, 0.0);
}

void calcFootstep(double target_distance, double target_angle, double delta_time, double& fb_move, double& rl_angle) 
{
  double next_movement = current_x_move_;
  if (target_distance < 0)
    target_distance = 0.0;

  double fb_goal = fmin(target_distance * 0.1, MAX_FB_STEP);
  accum_period_time += delta_time;
  if (accum_period_time > (current_period_time  / 4))
  {
    accum_period_time = 0.0;
    if ((target_distance * 0.1 / 2) < current_x_move_)
      next_movement -= UNIT_FB_STEP;
    else
      next_movement += UNIT_FB_STEP;
  }
  fb_goal = fmin(next_movement, fb_goal);
  fb_move = fmax(fb_goal, MIN_FB_STEP);

  double rl_goal = 0.0;
  if (fabs(target_angle) * 180 / M_PI > 5.0)
  {
    double rl_offset = fabs(target_angle) * 0.2;
    rl_goal = fmin(rl_offset, MAX_RL_TURN);
    rl_goal = fmax(rl_goal, MIN_RL_TURN);
    rl_angle = fmin(fabs(current_r_angle_) + UNIT_RL_TURN, rl_goal);

    if (target_angle < 0)
      rl_angle *= (-1);
  }
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

bool isActionRunning() 
{
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

void setWalkingParam(double x_move, double y_move, double rotation_angle, bool balance)
{
  current_walking_param.balance_enable = balance;
  current_walking_param.x_move_amplitude = x_move + SPOT_FB_OFFSET;
  current_walking_param.y_move_amplitude = y_move + SPOT_RL_OFFSET;
  current_walking_param.angle_move_amplitude = rotation_angle + SPOT_ANGLE_OFFSET;

  set_walking_param_pub.publish(current_walking_param);

  current_x_move_ = x_move;
  current_r_angle_ = rotation_angle;
}

bool getWalkingParam() 
{
  
  op3_walking_module_msgs::GetWalkingParam walking_param_msg;

  if (get_param_client.call(walking_param_msg))
  {
    current_walking_param = walking_param_msg.response.parameters;

    // update ui
    ROS_INFO_COND(DEBUG_PRINT, "Get walking parameters");

    return true;
  }
  else
  {
    ROS_ERROR("Fail to get walking parameters.");

    return false;
  }
}

void callbackJointStates(const sensor_msgs::JointState& msg)
{ 
  head_pan = msg.position[0];
  head_tilt = msg.position[1];
  return;
}

void callbackPosition(const geometry_msgs::Point& msg)
{
  positionx = msg.x;
  positiony = msg.y;
}

void callbackError(const geometry_msgs::Point& msg)
{
  errorx = msg.x;
  errory = msg.y;
}


void callbackBall(const geometry_msgs::Point& msg){
  ball_area = msg.z;
}

void callbackSlope(const std_msgs::Float64& msg){
  ball_slope = msg.data;
}

void callbackImu(const sensor_msgs::Imu::ConstPtr& msg) 
{
  Eigen::Quaterniond orientation(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
  Eigen::MatrixXd rpy_orientation = robotis_framework::convertQuaternionToRPY(orientation);
  rpy_orientation *= (180 / 3.141516);
  double w = msg->orientation.w;
  double x = msg->orientation.x;
  double y = msg->orientation.y;
  double z = msg->orientation.z;
  double pitch = rpy_orientation.coeff(1, 0);

  double t3 = 2.0 * (w * z + x * y);
  double t4 = 1.0 - 2.0 * (y * y + z * z);
  double yaw_z = atan2(2.0*(y*z + w*x), w*w - x*x - y*y + z*z); //atan2(t3, t4);
  yaw_z *= (180 / 3.141516);
  //std::cout << yaw_z << std::endl;

  double alpha = 0.4;
  if (present_pitch_ == 0) 
    present_pitch_ = pitch;
  else
    present_pitch_ = present_pitch_ * (1 - alpha) + pitch * alpha;

  if (present_pitch_ > FALL_FORWARD_LIMIT) {
    goAction(122);
    setModule("none");
  } else if (present_pitch_ < FALL_BACK_LIMIT) {
    goAction(1);
    setModule("none");
    ros::Duration(1).sleep();
    goAction(82);
    setModule("none");
  } else {
    state = 0;
  }
}

void callbackfindBall(const std_msgs::Bool& msg)
{
  ball = msg.data;
}

void callbackTurn(const std_msgs::Bool& msg)
{
  turnNsearch = msg.data;
}

void callbackSearchBall(const std_msgs::Bool& msg)
{
  search_ball = msg.data;
}
