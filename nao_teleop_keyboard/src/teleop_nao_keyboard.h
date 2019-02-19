/*
 * Nao Keyboard teleoperation
 */

#ifndef TELEOP_NAO_KEYBOARD_H
#define TELEOP_NAO_KEYBOARD_H

#include <ros/ros.h>
#include <stdio.h>
#include <iostream>
#include <algorithm>
#include <signal.h>
#include <termios.h>

#include <geometry_msgs/Twist.h>

#include <naoqi_bridge_msgs/JointAnglesWithSpeed.h>


//BASE
#define KEY_w 0x77 // move forward
#define KEY_W 0x57

#define KEY_a 0x61 // move left
#define KEY_A 0x41

#define KEY_d 0x64 // move right
#define KEY_D 0x44

#define KEY_s 0x73 // stop
#define KEY_S 0x53

#define KEY_q 0x71 // rotate right -> left
#define KEY_Q 0x51

#define KEY_e 0x65 // rotate left -> right
#define KEY_E 0x45


//HEAD
#define KEY_t 0x74 // move up
#define KEY_T 0x54

#define KEY_f 0x66 // move left
#define KEY_F 0x46

#define KEY_h 0x68 // move right
#define KEY_H 0x48

#define KEY_g 0x67 // move down
#define KEY_G 0x47

#define KEY_y 0x79 // move up right
#define KEY_Y 0x59

#define KEY_r 0x72 // move up left
#define KEY_R 0x52

#define KEY_b 0x62 // move down right
#define KEY_B 0x42

#define KEY_v 0x76 // move down left
#define KEY_V 0x56


//#define KEYCODE_Q 0x71 // f2




const double PI = 3.1415926535897;
const double speed = 10;
//const double angle = 10;

int kfd = 0;
struct termios cooked, raw;



class TeleopNao
{

public:

  TeleopNao();
  void keyLoop();

  ros::NodeHandle nh;
  ros::Publisher twist_pub;
  ros::Publisher head_pub;
  ros::Publisher speech_pub;

  

private:

  //base
  double linear_x;
  double linear_y;
  double angular_val;

  double linear_scale;
  double angular_scale;

  //head
  double head_pitch;
  double head_yaw;

  double head_pitch_scale;
  double head_yaw_scale;

  //messages
  naoqi_bridge_msgs::JointAnglesWithSpeed headAngles;

  geometry_msgs::Twist twist;
  
  //std_msgs::String string;

};


#endif