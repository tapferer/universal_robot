#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <math.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int8.h>
#include "ur_arm/Joints.h"
#include <unistd.h>   // for function usleep(microseconds)

// Global Variables
int Frec = 0;
std::ofstream fout("data/test.txt");

// Function definition
void recordToTxt(sensor_msgs::JointState curState);// The callback func for subscriber"recorder"

// Main
int main(int argc, char **argv)
{
  ros::init(argc, argv, "collision_detect");
  ros::NodeHandle n;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Publisher chatter_pub = n.advertise<ur_arm::Joints>("/ur_arm/cmd_joint_vel", 1);
  ros::Subscriber recorder = n.subscribe<sensor_msgs::JointState>("/joint_states", 1, recordToTxt);// Subscribing the joint_states and record them.
  usleep(400000);//Leave 0.4s for building the subscriber
  ur_arm::Joints vel_move;
  ur_arm::Joints vel_static;

  vel_move.base = 0;
  vel_move.shoulder = 0;
  vel_move.elbow = 0.0;
  vel_move.wrist1 = 0.0;
  vel_move.wrist2 = 0.0;
  vel_move.wrist3 = 0.0;

  vel_static.base = 0;
  vel_static.shoulder = 0.0;
  vel_static.elbow = 0;
  vel_static.wrist1 = 0;
  vel_static.wrist2 = 0;
  vel_static.wrist3 = 0;

  for(int i=0;i<100;i++)
  {
      vel_move.base = vel_move.base + 0.02;
      chatter_pub.publish(vel_move);
      ROS_INFO("i = [%d].", i+1);
      usleep(20000);//0.02s
      // Need I leave some time for the Publisher to publish msgs?
  }

  ROS_INFO("The robot will stop from now.");
  chatter_pub.publish(vel_static);
  ROS_INFO("Stopped.");
  fout.close();

  return 0;
}

void recordToTxt(sensor_msgs::JointState curState)
{
    // After test, I know that this function is called 50 times per second.
    std::vector<std::string> curName;
    std::vector<double> curPos;
    std::vector<double> curVel;
    std::vector<double> curEff;

    curName = curState.name;
    curPos = curState.position;
    curVel = curState.velocity;
    curEff = curState.effort;

    // I dont record these data because I dont use them.
    fout<<"header:"<<std::endl;
    fout<<"  seq: "<<"000000"<<std::endl;
    fout<<"  stamp:"<<std::endl;
    fout<<"    secs: "<<"000000"<<std::endl;
    fout<<"    nsecs: "<<"000000"<<std::endl;
    fout<<"  frame_id: \'\'"<<std::endl;

    // write the name;
    fout <<"name: [";
    for(int i=0; i<(curName.size()-1); ++i)
    {
      fout <<'\'';
      fout << curName[i] << "\', ";
    }
    fout<<'\''<<curName[curName.size()-1]<<'\''<<']'<<std::endl;

    // write the position;
    fout <<"position: [";
    for(int i=0; i<(curPos.size()-1); ++i)
    {
      fout << curPos[i] << ", ";
    }
    fout<<curPos[curPos.size()-1]<<']'<<std::endl;

    // write the velocity
    fout <<"velocity: [";
    for(int i=0; i<(curVel.size()-1); ++i)
    {
      fout << curVel[i] <<", ";
    }
    fout<<curVel[curVel.size()-1]<<']'<<std::endl;

    // write the effort
    fout <<"effort: [";
    for(int i=0; i<(curEff.size()-1); ++i)
    {
      fout << curEff[i] << ", ";
    }
    fout<<curEff[curEff.size()-1]<<']'<<std::endl;
    fout<<"---"<<std::endl;

    ROS_INFO("I heard [%d] msgs.", Frec);
    Frec++;
}
