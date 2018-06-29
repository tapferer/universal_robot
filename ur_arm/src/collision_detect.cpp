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
int monitorTime = 0;
int monitorFrec = 50;
bool torquePub = false;
ur_arm::Joints exTorque;

// Function definition
void getCurRobotState(sensor_msgs::JointState curState);// The callback func for subscriber"monitor", get cur pos/vel/eff values.
ur_arm::Joints computeExTorque(std::vector<double> curPos, std::vector<double> curVel, std::vector<double> curEff);

// Main
int main(int argc, char **argv)
{
  ros::init(argc, argv, "collision_detect");
  ros::NodeHandle n;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Publisher collision_pub = n.advertise<ur_arm::Joints>("/external_torque",1);
  ros::Subscriber monitor = n.subscribe<sensor_msgs::JointState>("/joint_states", 1, getCurRobotState);// Subscribing the joint_states for collision compute.
  usleep(400000);//Leave 0.4s for building the publisher and subscriber

  while(ros::ok())
  {
      if(torquePub == true)
      {
          collision_pub.publish(exTorque);
          //usleep(20000);
      }
      else{};
  }
  return 0;
}

void getCurRobotState(sensor_msgs::JointState curState)
{
    monitorTime++;
    if (monitorTime==(125/monitorFrec))
    {
        monitorTime = 0;
        std::vector<double> curPos;
        std::vector<double> curVel;
        std::vector<double> curEff;
        curPos = curState.position;
        curVel = curState.velocity;
        curEff = curState.effort;
        exTorque = computeExTorque(curPos, curVel, curEff);
        torquePub = true;
        usleep(10000);
    }
    else
    {
        torquePub = false;
    }
}

ur_arm::Joints computeExTorque(std::vector<double> curPos, std::vector<double> curVel, std::vector<double> curEff)
{
    // toliaezi give the curEff to exTorque
    ur_arm::Joints torque;
    torque.base = curEff[0];
    torque.shoulder = curEff[1];
    torque.elbow = curEff[2];
    torque.wrist1 = curEff[3];
    torque.wrist2 = curEff[4];
    torque.wrist3 = curEff[5];
    return torque;
}
