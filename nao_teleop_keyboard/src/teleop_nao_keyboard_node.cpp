/*
 * Nao Keyboard teleoperation
 */




#include "teleop_nao_keyboard.h"

#include <naoqi_bridge_msgs/JointTrajectoryAction.h>
#include <std_msgs/String.h>




TeleopNao::TeleopNao(): linear_x(0), linear_y(0), angular_val(0), head_pitch(0), head_yaw(0), linear_scale(0.3), angular_scale(0.5) ,head_pitch_scale(0.1), head_yaw_scale(0.1)
{

  nh.param("scale_angular", angular_scale, angular_scale);
  nh.param("scale_linear", linear_scale, linear_scale);

  kfd = 0;

  twist_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

  headAngles.joint_names.push_back("HeadYaw");
  headAngles.joint_names.push_back("HeadPitch");
  headAngles.joint_angles.resize(2, 0.0f);
  headAngles.relative = 0;
  headAngles.speed = std::min( head_pitch_scale, head_yaw_scale );

  head_pub = nh.advertise<naoqi_bridge_msgs::JointAnglesWithSpeed>("/joint_angles", 1);

  speech_pub = nh.advertise<std_msgs::String>("/speech", 1);

}


void quit(int sig)
{

  (void)sig;
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);

}


void TeleopNao::keyLoop()
{

  char c;
  bool dirty=false;


  // get the console in raw mode                                                              
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file                         
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);


  puts("\n--------------------------------------");
  puts("Move and stop nao with :");
  puts("        W");
  puts("  A           D\n");
  puts("Rotate right - left :");
  puts("     E       Q");
  puts("--------------------------------------");
  puts("Move head with :");
  puts("        T");
  puts("  F     G     H");
  puts("--------------------------------------");
  puts("STOP with :");
  puts("     S");
  puts("--------------------------------------\n");


  std_msgs::String string;
  string.data = "I am ready!";
  speech_pub.publish(string);
  ROS_INFO("%s", (string.data).c_str());


  while(true)
  {
    // get the next event from the keyboard  
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }


    //base
    linear_x = linear_y = angular_val = 0;
    
    //head
    head_yaw = head_pitch = 0;

    //ROS_INFO("value: 0x%02X\n", c);
  


    if( c == KEY_S || c == KEY_s)
    {
      ROS_INFO("STOP");

      // string.data = "Stopping!";
      // speech_pub.publish(string);

      linear_x = linear_y = angular_val = 0;
      head_yaw = head_pitch = 0;
      dirty = true;
    }
    else if( c == KEY_A || c == KEY_a)
    {  
      ROS_INFO("LEFT");

      // string.data = "Eyes left!";
      // speech_pub.publish(string);

      linear_y = 1.0;
      dirty = true;
    }
    else if( c == KEY_D || c == KEY_d)
    {
      ROS_INFO("RIGHT");

      // string.data = "It's all right!";
      // speech_pub.publish(string);

      linear_y = -1.0;
      dirty = true;
    }
    else if( c == KEY_W || c == KEY_w)
    {
      ROS_INFO("FORWARD");

      // string.data = "Forward the agreement!";
      // speech_pub.publish(string);

      linear_x = 1.0;
      dirty = true;
    }
    else if( c == KEY_E || c == KEY_e)
    {
      ROS_INFO("Rotate left -> right");

      // string.data = "A turn for the worse!";
      // speech_pub.publish(string);

      angular_val = -1.0;
      dirty = true;
    }
    else if( c == KEY_Q || c == KEY_q)
    {
      ROS_INFO("Rotate right -> left");

      // string.data = "An interesting turn of events!";
      // speech_pub.publish(string);

      angular_val = 1.0;
      dirty = true;
    }
    else if( c == KEY_T || c == KEY_t)
    {
      ROS_INFO("Head UP");

      headAngles.header.stamp = ros::Time::now();
      headAngles.relative = 1;
      head_yaw = 0;
      head_pitch = -0.1;

      dirty = true;
    }
    else if( c == KEY_F || c == KEY_f)
    {
      ROS_INFO("Head RIGHT");

      headAngles.header.stamp = ros::Time::now();
      headAngles.relative = 1;
      head_yaw = 0.1;
      head_pitch = 0;

      dirty = true;
    }
    else if( c == KEY_H || c == KEY_h)
    {
      ROS_INFO("Head LEFT");

      headAngles.header.stamp = ros::Time::now();
      headAngles.relative = 1;
      head_yaw = -0.1;
      head_pitch = 0;

      dirty = true;
    }
    else if( c == KEY_G || c == KEY_g)
    {
      ROS_INFO("Head DOWN");

      headAngles.header.stamp = ros::Time::now();
      headAngles.relative = 1;
      head_yaw = 0;
      head_pitch = 0.1;

      dirty = true;
    }
    else if( c == KEY_Y || c == KEY_y)
    {
      ROS_INFO("Head UP RIGHT");

      headAngles.header.stamp = ros::Time::now();
      headAngles.relative = 1;
      head_yaw = -0.1;
      head_pitch = -0.1;

      dirty = true;
    }
    else if( c == KEY_R || c == KEY_r)
    {
      ROS_INFO("Head UP LEFT");

      headAngles.header.stamp = ros::Time::now();
      headAngles.relative = 1;
      head_yaw = 0.1;
      head_pitch = -0.1;

      dirty = true;
    }
    else if( c == KEY_B || c == KEY_b)
    {
      ROS_INFO("Head DOWN RIGHT");

      headAngles.header.stamp = ros::Time::now();
      headAngles.relative = 1;
      head_yaw = -0.1;
      head_pitch = 0.1;

      dirty = true;
    }
    else if( c == KEY_V || c == KEY_v)
    {
      ROS_INFO("Head DOWN LEFT");

      headAngles.header.stamp = ros::Time::now();
      headAngles.relative = 1;
      head_yaw = 0.1;
      head_pitch = 0.1;

      dirty = true;
    }



    //translation base
    twist.linear.x = linear_scale * linear_x * (speed*PI/360); //speed value m/s
    twist.linear.y = linear_scale * linear_y * (speed*PI/360);
    //rotation base
    twist.angular.z = angular_scale * angular_val * (speed*2*PI/360); //degrees to rad

    //head
    headAngles.joint_angles[0] = head_yaw * head_yaw_scale;
    headAngles.joint_angles[1] = head_pitch * head_pitch_scale;


    if(dirty ==true)
    {
      twist_pub.publish(twist);
      head_pub.publish(headAngles);
      dirty=false;
    }

  }

  return;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_nao");
  TeleopNao teleop_nao;

  signal(SIGINT,quit);

  teleop_nao.keyLoop();
  
  return(0);
}

