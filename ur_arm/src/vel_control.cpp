#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <math.h>
#include <Eigen/Dense>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int8.h>
#include "ur_arm/Joints.h"
#include <unistd.h>   // for function usleep(microseconds)

// Global Variables
int Frec = 0;
std::ofstream fout("data/test.txt");
std::vector<double> curPos;
std::vector<double> curVel;
std::vector<double> curEff;
ur_arm::Joints velNew;
ur_arm::Joints velInitial;
ur_arm::Joints velStop;

// Function definition
void recordToTxt(sensor_msgs::JointState curState);// The callback func for subscriber"recorder"
void velCompute(ur_arm::Joints exTorque);// Judge if collision happens and compute newVel
std::vector<double> deltaVelCacByJacob(std::vector<double> pos, std::vector<double> cartesianDeltaVel);
void setVelStop();
void setVelInitial();
double reZero(double x);

// Main
int main(int argc, char **argv)
{
  ros::init(argc, argv, "vel_control");
  ros::NodeHandle n;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Publisher vel_pub = n.advertise<ur_arm::Joints>("/ur_arm/cmd_joint_vel", 1);
  ros::Subscriber recorder = n.subscribe<sensor_msgs::JointState>("/joint_states", 1, recordToTxt);// Subscribing the joint_states and record them.
  ros::Subscriber collisionCheck = n.subscribe<ur_arm::Joints>("/external_torque", 1, velCompute);
  usleep(400000);//Leave 0.4s for building the subscriber

  setVelInitial();
  setVelStop();

  vel_pub.publish(velInitial);

  while(ros::ok())
  {
      vel_pub.publish(velNew);
      //usleep(2000);
  }
  fout.close();

  return 0;
}

void setVelInitial()
{
    velInitial.base = 0.0;
    velInitial.shoulder = 0.0;
    velInitial.elbow = 0;
    velInitial.wrist1 = 0;
    velInitial.wrist2 = 0;
    velInitial.wrist3 = 0;
}

void setVelStop()
{
    velStop.base = 0;
    velStop.shoulder = 0.0;
    velStop.elbow = 0;
    velStop.wrist1 = 0;
    velStop.wrist2 = 0;
    velStop.wrist3 = 0;
}

void recordToTxt(sensor_msgs::JointState curState)
{
    // After test, I know that this function is called 125 times per second in real robot connection.
    std::vector<std::string> curName;

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

void velCompute(ur_arm::Joints exTorque)
{

    ur_arm::Joints torque;
    std::vector<double> pos;
    std::vector<double> vel;
    std::vector<double> deltaVel;
    std::vector<double> cartesianDeltaVel;
    std::vector<double> newVel;
    bool rule = false;// the collision judging rule.

    torque = exTorque;
    pos = curPos;
    vel = curVel;

    // x translation
    cartesianDeltaVel.push_back(0);
    // y translation
    cartesianDeltaVel.push_back(0);
    // z translation
    cartesianDeltaVel.push_back(0.003);
    cartesianDeltaVel.push_back(0);
    cartesianDeltaVel.push_back(0);
    cartesianDeltaVel.push_back(0);


    // Rule definition
    rule = (fabs(torque.shoulder)>3.5);
    // end...
    int i = 0;
    if (rule&(i<1000))
    {
        // let the robot stop
        velNew= velStop;
    }
    else
    {
        usleep(2000);
        i++;
        // continue the cartesian moving rule
        // Caculate the joints deltaVel by Jacobian Matrix
        deltaVel = deltaVelCacByJacob(pos, cartesianDeltaVel);
        std::vector<double>  mid;
        for(int i=0; i<(vel.size()); ++i)
        {
            double midvalue;
            midvalue= vel[i] + deltaVel[i];
            mid.push_back(midvalue);
        }

        newVel.swap(mid);
        //mid.clear();

        velNew.base = newVel[0];
        velNew.shoulder = newVel[1];
        velNew.elbow = newVel[2];
        velNew.wrist1 = newVel[3];
        velNew.wrist2 = newVel[4];
        velNew.wrist3 = newVel[5];
    }

}

std::vector<double> deltaVelCacByJacob(std::vector<double> pos, std::vector<double> cartesianDeltaVel)
{
    double c1,c2,c3,c4,c5,c6,s1,s2,s3,s4,s5,s6;
    double d1=0.0892,d4=0.109,d5=0.093,d6=0.082;
    double a2=-0.425,a3=-0.39243;
    Eigen::MatrixXf Jacob(6,6);
    Eigen::MatrixXf JacobInv(6,6);
    Eigen::MatrixXf cdvel(6,1);
    Eigen::MatrixXf velResult(6,1);
    std::vector<double> deltaVel;
    for (int i=0;i<6;i++)
    {
        cdvel(i,0) = cartesianDeltaVel[i];
    }

    c1 = reZero(cos(pos[0]));
    c2 = reZero(cos(pos[1]));
    c3 = reZero(cos(pos[2]));
    c4 = reZero(cos(pos[3]));
    c5 = reZero(cos(pos[4]));
    c6 = reZero(cos(pos[5]));

    s1 = reZero(sin(pos[0]));
    s2 = reZero(sin(pos[1]));
    s3 = reZero(sin(pos[2]));
    s4 = reZero(sin(pos[3]));
    s5 = reZero(sin(pos[4]));
    s6 = reZero(sin(pos[5]));

    Jacob = Eigen::MatrixXf::Zero(6,6);

    Jacob(0,0) = c1*(d4 + c5*d6) - s1*(a2*c2 + c2*(s3*(c4*d5 + d6*s4*s5) + a3*c3 + c3*(d5*s4 - c4*d6*s5)) - s2*(s3*(d5*s4 - c4*d6*s5) + a3*s3 - c3*(c4*d5 + d6*s4*s5)));
    Jacob(0,1) = -c1*(a2*s2 + s2*(s3*(c4*d5 + d6*s4*s5) + a3*c3 + c3*(d5*s4 - c4*d6*s5)) + c2*(s3*(d5*s4 - c4*d6*s5) + a3*s3 - c3*(c4*d5 + d6*s4*s5)));
    Jacob(0,2) = -c1*(s2*(s3*(c4*d5 + d6*s4*s5) + a3*c3 + c3*(d5*s4 - c4*d6*s5)) + c2*(s3*(d5*s4 - c4*d6*s5) + a3*s3 - c3*(c4*d5 + d6*s4*s5)));
    Jacob(0,3) = c1*(d5*(c4*(c2*c3 - s2*s3) - s4*(c2*s3 + c3*s2)) + d6*s5*(c4*(c2*s3 + c3*s2) + s4*(c2*c3 - s2*s3)));
    Jacob(0,4) = - d6*(c4*(c2*c3 - s2*s3) - s4*(c2*s3 + c3*s2))*(c1*c5 + s5*(c4*(c2*c3*s1 - s1*s2*s3) - s4*(c2*s1*s3 + c3*s1*s2))) - d6*s5*(c4*(c2*s1*s3 + c3*s1*s2) + s4*(c2*c3*s1 - s1*s2*s3))*(c4*(c2*s3 + c3*s2) + s4*(c2*c3 - s2*s3));
    Jacob(0,5) = 0;

    Jacob(1,0) = s1*(d4 + c5*d6) + c1*(a2*c2 + c2*(s3*(c4*d5 + d6*s4*s5) + a3*c3 + c3*(d5*s4 - c4*d6*s5)) - s2*(s3*(d5*s4 - c4*d6*s5) + a3*s3 - c3*(c4*d5 + d6*s4*s5)));
    Jacob(1,1) = -s1*(a2*s2 + s2*(s3*(c4*d5 + d6*s4*s5) + a3*c3 + c3*(d5*s4 - c4*d6*s5)) + c2*(s3*(d5*s4 - c4*d6*s5) + a3*s3 - c3*(c4*d5 + d6*s4*s5)));
    Jacob(1,2) = -s1*(s2*(s3*(c4*d5 + d6*s4*s5) + a3*c3 + c3*(d5*s4 - c4*d6*s5)) + c2*(s3*(d5*s4 - c4*d6*s5) + a3*s3 - c3*(c4*d5 + d6*s4*s5)));
    Jacob(1,3) = s1*(d5*(c4*(c2*c3 - s2*s3) - s4*(c2*s3 + c3*s2)) + d6*s5*(c4*(c2*s3 + c3*s2) + s4*(c2*c3 - s2*s3)));
    Jacob(1,4) = d6*s5*(c4*(c1*c2*s3 + c1*c3*s2) + s4*(c1*c2*c3 - c1*s2*s3))*(c4*(c2*s3 + c3*s2) + s4*(c2*c3 - s2*s3)) - d6*(c5*s1 - s5*(c4*(c1*c2*c3 - c1*s2*s3) - s4*(c1*c2*s3 + c1*c3*s2)))*(c4*(c2*c3 - s2*s3) - s4*(c2*s3 + c3*s2));
    Jacob(1,5) = 0;

    Jacob(2,0) = 0;
    Jacob(2,1) = c1*(s1*(d4 + c5*d6) + c1*(a2*c2 + c2*(s3*(c4*d5 + d6*s4*s5) + a3*c3 + c3*(d5*s4 - c4*d6*s5)) - s2*(s3*(d5*s4 - c4*d6*s5) + a3*s3 - c3*(c4*d5 + d6*s4*s5)))) - s1*(c1*(d4 + c5*d6) - s1*(a2*c2 + c2*(s3*(c4*d5 + d6*s4*s5) + a3*c3 + c3*(d5*s4 - c4*d6*s5)) - s2*(s3*(d5*s4 - c4*d6*s5) + a3*s3 - c3*(c4*d5 + d6*s4*s5))));
    Jacob(2,2) = c1*(s1*(d4 + c5*d6) + c1*c2*(s3*(c4*d5 + d6*s4*s5) + a3*c3 + c3*(d5*s4 - c4*d6*s5)) - c1*s2*(s3*(d5*s4 - c4*d6*s5) + a3*s3 - c3*(c4*d5 + d6*s4*s5))) - s1*(c1*(d4 + c5*d6) - c2*s1*(s3*(c4*d5 + d6*s4*s5) + a3*c3 + c3*(d5*s4 - c4*d6*s5)) + s1*s2*(s3*(d5*s4 - c4*d6*s5) + a3*s3 - c3*(c4*d5 + d6*s4*s5)));
    Jacob(2,3) = c1*(d5*(c4*(c1*c2*s3 + c1*c3*s2) + s4*(c1*c2*c3 - c1*s2*s3)) - d6*s5*(c4*(c1*c2*c3 - c1*s2*s3) - s4*(c1*c2*s3 + c1*c3*s2)) + c5*d6*s1) - s1*(c1*c5*d6 - d5*(c4*(c2*s1*s3 + c3*s1*s2) + s4*(c2*c3*s1 - s1*s2*s3)) + d6*s5*(c4*(c2*c3*s1 - s1*s2*s3) - s4*(c2*s1*s3 + c3*s1*s2)));
    Jacob(2,4) = - d6*(c4*(c1*c2*s3 + c1*c3*s2) + s4*(c1*c2*c3 - c1*s2*s3))*(c1*c5 + s5*(c4*(c2*c3*s1 - s1*s2*s3) - s4*(c2*s1*s3 + c3*s1*s2))) - d6*(c4*(c2*s1*s3 + c3*s1*s2) + s4*(c2*c3*s1 - s1*s2*s3))*(c5*s1 - s5*(c4*(c1*c2*c3 - c1*s2*s3) - s4*(c1*c2*s3 + c1*c3*s2)));
    Jacob(2,5) = 0;

    Jacob(3,0) = 0;
    Jacob(3,1) = s1;
    Jacob(3,2) = s1;
    Jacob(3,3) = s1;
    Jacob(3,4) = c4*(c1*c2*s3 + c1*c3*s2) + s4*(c1*c2*c3 - c1*s2*s3);
    Jacob(3,5) = c5*s1 - s5*(c4*(c1*c2*c3 - c1*s2*s3) - s4*(c1*c2*s3 + c1*c3*s2));

    Jacob(4,0) = 0;
    Jacob(4,1) = -c1;
    Jacob(4,2) = -c1;
    Jacob(4,3) = -c1;
    Jacob(4,4) = c4*(c2*s1*s3 + c3*s1*s2) + s4*(c2*c3*s1 - s1*s2*s3);
    Jacob(4,5) = - c1*c5 - s5*(c4*(c2*c3*s1 - s1*s2*s3) - s4*(c2*s1*s3 + c3*s1*s2));

    Jacob(5,0) = 1;
    Jacob(5,1) = 0;
    Jacob(5,2) = 0;
    Jacob(5,3) = 0;
    Jacob(5,4) = s4*(c2*s3 + c3*s2) - c4*(c2*c3 - s2*s3);
    Jacob(5,5) = -s5*(c4*(c2*s3 + c3*s2) + s4*(c2*c3 - s2*s3));

    JacobInv = Jacob.inverse();
    velResult = JacobInv*cdvel;
    std::vector<double> mid;
    for (int i=0;i<6;i++)
    {
        mid.push_back(velResult(i,0));
    }
    deltaVel.swap(mid);
    //mid.clear();
    return deltaVel;
}

double reZero(double x)
{
    if(fabs(x)<10e-10)
    {
        x = 0;
    }
    else
    {
        x = x;
    }
    return x;
}
