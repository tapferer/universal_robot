#!/usr/bin/env python
# Simplified by Petori in 2018/3/31
import time
import roslib; roslib.load_manifest('ur_modern_driver')
import rospy
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState
from math import pi

JOINT_NAMES = ['right_shoulder_pan_joint','right_shoulder_lift_joint','right_elbow_joint',
               'right_wrist_1_joint','right_wrist_2_joint','right_wrist_3_joint',
               'left_shoulder_pan_joint','left_shoulder_lift_joint','left_elbow_joint',
               'left_wrist_1_joint','left_wrist_2_joint','left_wrist_3_joint']
Q1 = [-2.182,-1.3021,1.8322,-0.5627,0.9730,-0.0065,1.8932,-1.2798,1.7942,-0.5058,1.9410,0.2056]
Q2 = [-2.182,-1.3961,1.6727,-0.3093,0.9730,-0.0065,1.8932,-1.3700,1.6341,-0.2554,1.9410,0.2056]
Q3 = [-2.182,-1.3961,1.6727,-0.3093,0.9730,-0.0065,1.8932,-1.3700,1.6341,-0.2554,1.9410,0.2056]
    
client = None

def move():
    global joints_pos
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    try:
        joint_states = rospy.wait_for_message("joint_states", JointState)
        joints_pos = joint_states.position
        g.trajectory.points = [
            JointTrajectoryPoint(positions=joints_pos, velocities=[0]*12, time_from_start=rospy.Duration(0.0)),
            JointTrajectoryPoint(positions=Q1, velocities=[0]*12, time_from_start=rospy.Duration(2.0)),
            JointTrajectoryPoint(positions=Q2, velocities=[0]*12, time_from_start=rospy.Duration(3.0)),
            JointTrajectoryPoint(positions=Q3, velocities=[0]*12, time_from_start=rospy.Duration(4.0))]
        client.send_goal(g)
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise
    except:
        raise

   
def main():
    global client
    try:
        rospy.init_node("uu_simple_move", anonymous=True, disable_signals=True)
        client = actionlib.SimpleActionClient('arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        print "Waiting for server..."
        client.wait_for_server()
        print "Connected to server"
        move()
        print "Trajectory finished"
    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise

if __name__ == '__main__': main()
