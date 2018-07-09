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
std::ofstream fout1("data/jointStates.txt");
std::ofstream fout2("data/externalTorque.txt");
std::vector<double> curPos;
std::vector<double> curVel;
std::vector<double> curEff;
geometry_msgs::Twist velFoward;
geometry_msgs::Twist velBack;
geometry_msgs::Twist velToPointMid;
geometry_msgs::Twist velToPointEnd;
geometry_msgs::Twist velToPointMidInv;
geometry_msgs::Twist velToPointEndInv;
geometry_msgs::Twist velStop;
bool collisionHappen = false;
bool rule = false;// the collision judging rule.
ur_arm::Joints torque;
double collisionTorque = 7.5;

// Function definition
void recordJointStateToTxt(sensor_msgs::JointState curState);
void recordExternalTorqueToTxt(ur_arm::Joints joints);
void velCompute(ur_arm::Joints exTorque);// Judge if collision happens and compute newVel
void setVelFoward();
void setVelBack();
void setVelToPointMid();
void setVelToPointEnd();
void setVelToPointMidInv();
void setVelToPointEndInv();
void setVelStop();
double reZero(double x);
geometry_msgs::Twist fkine(std::vector<double> pos);
geometry_msgs::Twist velGet(geometry_msgs::Twist pose1, geometry_msgs::Twist pose2);

// Main
int main(int argc, char **argv)
{
  ros::init(argc, argv, "vel_control");
  ros::NodeHandle n;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("/ur_arm/cmd_tool_vel", 1);
  ros::Subscriber collisionCheck = n.subscribe<ur_arm::Joints>("/external_torque", 1, velCompute);
  ros::Subscriber recorder1 = n.subscribe<sensor_msgs::JointState>("/joint_states", 1, recordJointStateToTxt);// Subscribing the joint_states and record them.
  ros::Subscriber recorder2 = n.subscribe<ur_arm::Joints>("/external_torque", 1, recordExternalTorqueToTxt);

  usleep(500000);//Leave 0.5s for building the subscribers and publishers

  setVelFoward();
  setVelBack();
  setVelToPointMid();
  setVelToPointEnd();
  setVelStop();

  bool rule1 = false;
  bool rule2 = false;
  bool rule3 = false;
  std::vector<double> startPoint;
  std::vector<double> midPoint;
  std::vector<double> endPoint;
  std::vector<double> preparePoint;
  geometry_msgs::Twist startPose;
  geometry_msgs::Twist midPose;
  geometry_msgs::Twist endPose;
  geometry_msgs::Twist curPose;
  geometry_msgs::Twist preparePose;
  geometry_msgs::Twist vel1,vel2,vel3,vel4;

  vel_pub.publish(velFoward);
  sleep(1);

  // Get 3 key position by collision detect.
  while(!rule1)
  {
      rule1 = ((torque.shoulder>collisionTorque) || (torque.elbow>collisionTorque));
  }
  startPoint = curPos;// start position
  vel_pub.publish(velBack);
  sleep(5);
  preparePoint = curPos;// prepare position
  vel_pub.publish(velToPointMid);
  sleep(5);
  vel_pub.publish(velFoward);
  while(!rule2)
  {
      rule2 = ((torque.shoulder>collisionTorque) || (torque.elbow>collisionTorque));
  }
  midPoint = curPos;// mid position
  vel_pub.publish(velBack);
  sleep(5);
  vel_pub.publish(velToPointEnd);
  sleep(5);
  vel_pub.publish(velFoward);
  while(!rule3)
  {
      rule3 = ((torque.shoulder>collisionTorque) || (torque.elbow>collisionTorque));
  }
  endPoint = curPos;// end position
  vel_pub.publish(velBack);
  sleep(5);
  vel_pub.publish(velStop);
  sleep(2);

  // Plan the grind process. -- by kinematics -- bad code
  // You best use the joint_position control and then use the tool_vel_control by kinematics.
  startPose = fkine(startPoint);
  midPose = fkine(midPoint);
  endPose = fkine(endPoint);
  curPose = fkine(curPos);
 // preparePose = fkine(preparePoint);
 // vel1 = velGet(curPose,preparePose);
//  vel2 = velGet(preparePose,startPose);
//  vel3 = velGet(startPose,midPose);
//  vel4 = velGet(midPose,endPose);

//  vel_pub.publish(vel1);
//  bool rule4 = false;
//  while (!rule4) {
//      bool r1,r2,r3;
//      geometry_msgs::Vector3 linear;
//      linear = preparePose.linear;
//      r1 = reZero(curPos[0]-linear.x)==0;
//      r2 = reZero(curPos[1]-linear.y)==0;
//      r3 = reZero(curPos[2]-linear.z)==0;
//      rule4 = (r1 && r2) &&  r3;
//  }
//  vel_pub.publish(velStop);
//  sleep(1);

  vel_pub.publish(velToPointEndInv);
  sleep(5);
  vel_pub.publish(velToPointMidInv);
  sleep(5);
  vel_pub.publish(velFoward);
  rule1 = false;
  while(!rule1)
  {
      rule1 = ((torque.shoulder>collisionTorque) || (torque.elbow>collisionTorque));
  }
  vel_pub.publish(vel3);
  sleep(5);
  vel_pub.publish(vel4);
  sleep(5);
  vel_pub.publish(velBack);
  sleep(5);
  vel_pub.publish(velStop);

  fout1.close();
  fout2.close();
  return 0;
}

geometry_msgs::Twist velGet(geometry_msgs::Twist pose1,geometry_msgs::Twist pose2)
{
    geometry_msgs::Vector3 linear1,linear2,linearVel;
    geometry_msgs::Vector3 angular;
    geometry_msgs::Twist vel;
    double goTime = 5;

    linear1 = pose1.linear;
    linear2 = pose2.linear;

    linearVel.x = (linear2.x - linear1.x)/goTime;
    linearVel.y = (linear2.y - linear1.y)/goTime;
    linearVel.z = (linear2.z - linear1.z)/goTime;
    angular.x = 0;
    angular.y = 0;
    angular.z = 0;

    vel.linear = linearVel;
    vel.angular = angular;

    return vel;
}

geometry_msgs::Twist fkine(std::vector<double> pos)
{
    // This forward kinematics ignore the rpy of pos.
    // I set rpy all as zero.
    geometry_msgs::Vector3 linear;
    geometry_msgs::Vector3 angular;
    geometry_msgs::Twist posxyzrpy;
    double d1=0.0892;
    double a2=-0.425;
    double a3=-0.39243;
    double d4=0.109;
    double d5=0.093;
    double d6=0.082;
    double theta1,theta2,theta3,theta4,theta5,theta6,c1,c2,c3,c4,c5,c6,s1,s2,s3,s4,s5,s6,c234,s234,c23,s23;

    theta1 = pos[0];
    theta2 = pos[1];
    theta3 = pos[2];
    theta4 = pos[3];
    theta5 = pos[4];
    theta6 = pos[5];

    c1=cos(theta1);
    s1=sin(theta1);
    c2=cos(theta2);
    s2=sin(theta2);
    c3=cos(theta3);
    s3=sin(theta3);
    c4=cos(theta4);
    s4=sin(theta4);
    c5=cos(theta5);
    s5=sin(theta5);
    c6=cos(theta6);
    s6=sin(theta6);

    linear.x = d6*(-c1*s5*c234+s1*c5)+d5*c1*s234+d4*s1+a3*c1*c23+a2*c1*c2;
    linear.y = d6*(-s1*s5*c234-c1*c5)+d5*s1*s234-d4*c1+a3*s1*c23+a2*s1*c2;
    linear.z = -d6*s234*s5-d5*c234+a3*s23+a2*s2+d1;
    angular.x = 0;
    angular.y = 0;
    angular.z = 0;
    posxyzrpy.linear = linear;
    posxyzrpy.angular = angular;
    // add the angular definition by curPos;
    return posxyzrpy;
}

double reZero(double x)
{
    if (fabs(x)<1e-5)
    {
        x = 0;
    }
    return x;
}

void velCompute(ur_arm::Joints exTorque)
{
    torque = exTorque;
}


void setVelFoward()
{
    geometry_msgs::Vector3 linear;
    geometry_msgs::Vector3 angular;
    double vx,vy,vz;
    double wx,wy,wz;
    vx = 0;
    vy = -0.005;
    vz = 0;
    wx = 0;
    wy = 0;
    wz = 0;
    linear.x = vx;
    linear.y = vy;
    linear.z = vz;
    angular.x = wx;
    angular.y = wy;
    angular.z = wz;
    velFoward.linear = linear;
    velFoward.angular = angular;
}

void setVelBack()
{
    geometry_msgs::Vector3 linear;
    geometry_msgs::Vector3 angular;
    double vx,vy,vz;
    double wx,wy,wz;
    vx = 0;
    vy = 0.005;
    vz = 0;
    wx = 0;
    wy = 0;
    wz = 0;
    linear.x = vx;
    linear.y = vy;
    linear.z = vz;
    angular.x = wx;
    angular.y = wy;
    angular.z = wz;
    velBack.linear = linear;
    velBack.angular = angular;
}

void setVelToPointMid()
{
    geometry_msgs::Vector3 linear;
    geometry_msgs::Vector3 angular;
    double vx,vy,vz;
    double wx,wy,wz;
    vx = 0;
    vy = 0;
    vz = -0.005;
    wx = 0;
    wy = 0;
    wz = 0;
    linear.x = vx;
    linear.y = vy;
    linear.z = vz;
    angular.x = wx;
    angular.y = wy;
    angular.z = wz;
    velToPointMid.linear = linear;
    velToPointMid.angular = angular;
}

void setVelToPointEnd()
{
    geometry_msgs::Vector3 linear;
    geometry_msgs::Vector3 angular;
    double vx,vy,vz;
    double wx,wy,wz;
    vx = -0.005;
    vy = 0;
    vz = 0;
    wx = 0;
    wy = 0;
    wz = 0;
    linear.x = vx;
    linear.y = vy;
    linear.z = vz;
    angular.x = wx;
    angular.y = wy;
    angular.z = wz;
    velToPointEnd.linear = linear;
    velToPointEnd.angular = angular;
}

void setVelToPointMidInv()
{
    geometry_msgs::Vector3 linear;
    geometry_msgs::Vector3 angular;
    double vx,vy,vz;
    double wx,wy,wz;
    vx = 0;
    vy = 0;
    vz = 0.005;
    wx = 0;
    wy = 0;
    wz = 0;
    linear.x = vx;
    linear.y = vy;
    linear.z = vz;
    angular.x = wx;
    angular.y = wy;
    angular.z = wz;
    velToPointMidInv.linear = linear;
    velToPointMidInv.angular = angular;
}

void setVelToPointEndInv()
{
    geometry_msgs::Vector3 linear;
    geometry_msgs::Vector3 angular;
    double vx,vy,vz;
    double wx,wy,wz;
    vx = 0.005;
    vy = 0;
    vz = 0;
    wx = 0;
    wy = 0;
    wz = 0;
    linear.x = vx;
    linear.y = vy;
    linear.z = vz;
    angular.x = wx;
    angular.y = wy;
    angular.z = wz;
    velToPointEndInv.linear = linear;
    velToPointEndInv.angular = angular;
}

void setVelStop()
{
    geometry_msgs::Vector3 linear;
    geometry_msgs::Vector3 angular;
    double vx,vy,vz;
    double wx,wy,wz;
    vx = 0;
    vy = 0;
    vz = 0;
    wx = 0;
    wy = 0;
    wz = 0;
    linear.x = vx;
    linear.y = vy;
    linear.z = vz;
    angular.x = wx;
    angular.y = wy;
    angular.z = wz;
    velStop.linear = linear;
    velStop.angular = angular;
}

void recordExternalTorqueToTxt(ur_arm::Joints joints)
{
    ur_arm::Joints curJoints;
    curJoints = joints;
    fout2<<"[";
    fout2<<curJoints.base<<", ";
    fout2<<curJoints.shoulder<<", ";
    fout2<<curJoints.elbow<<", ";
    fout2<<curJoints.wrist1<<", ";
    fout2<<curJoints.wrist2<<", ";
    fout2<<curJoints.wrist3<<"]"<<std::endl;
}

void recordJointStateToTxt(sensor_msgs::JointState curState)
{
    // After test, I know that this function is called 125 times per second in real robot connection.
    std::vector<std::string> curName;

    curName = curState.name;
    curPos = curState.position;
    curVel = curState.velocity;
    curEff = curState.effort;

    // I dont record these data because I dont use them.
    fout1<<"header:"<<std::endl;
    fout1<<"  seq: "<<"000000"<<std::endl;
    fout1<<"  stamp:"<<std::endl;
    fout1<<"    secs: "<<"000000"<<std::endl;
    fout1<<"    nsecs: "<<"000000"<<std::endl;
    fout1<<"  frame_id: \'\'"<<std::endl;

    // write the name;
    fout1 <<"name: [";
    for(int i=0; i<(curName.size()-1); ++i)
    {
      fout1 <<'\'';
      fout1 << curName[i] << "\', ";
    }
    fout1<<'\''<<curName[curName.size()-1]<<'\''<<']'<<std::endl;

    // write the position;
    fout1 <<"position: [";
    for(int i=0; i<(curPos.size()-1); ++i)
    {
      fout1 << curPos[i] << ", ";
    }
    fout1<<curPos[curPos.size()-1]<<']'<<std::endl;

    // write the velocity
    fout1 <<"velocity: [";
    for(int i=0; i<(curVel.size()-1); ++i)
    {
      fout1 << curVel[i] <<", ";
    }
    fout1<<curVel[curVel.size()-1]<<']'<<std::endl;

    // write the effort
    fout1 <<"effort: [";
    for(int i=0; i<(curEff.size()-1); ++i)
    {
      fout1 << curEff[i] << ", ";
    }
    fout1<<curEff[curEff.size()-1]<<']'<<std::endl;
    fout1<<"---"<<std::endl;
}
