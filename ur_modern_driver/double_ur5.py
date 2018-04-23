#!/usr/bin/env python
# Written by Petori in 2018/4/23
import time
import roslib; roslib.load_manifest('ur_modern_driver')
import rospy
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState
from math import pi

L_JOINT_NAMES = ['left_shoulder_pan_joint','left_shoulder_lift_joint','left_elbow_joint','left_wrist_1_joint','left_wrist_2_joint','left_wrist_3_joint']
R_JOINT_NAMES = ['right_shoulder_pan_joint','right_shoulder_lift_joint','right_elbow_joint','right_wrist_1_joint','right_wrist_2_joint','right_wrist_3_joint']

L_Q1 = [0.9598,-1.1512,1.9340,-0.8154,0.9730,-0.0065]
L_Q2 = [0.9598,-1.3021,1.8322,-0.5627,0.9730,-0.0065]
L_Q3 = [0.9598,-1.3961,1.6727,-0.3093,0.9730,-0.0065]
L_Q4 = [0.9598,-1.4313,1.4556,-0.0569,0.9730,-0.0065]
L_Q5 = [1.1579,-1.4956,1.5274,-0.0610,1.1711,-0.0135]
L_Q6 = [1.3835,-1.5071,1.5397,-0.0600,1.3966,-0.0201]
L_Q7 = [1.6155,-1.4641,1.4927,-0.0557,1.6285,-0.0264]
L_Q8 = [1.3835,-1.5071,1.5397,-0.0600,1.3966,-0.0201]
L_Q9 = [1.1579,-1.4956,1.5274,-0.0610,1.1711,-0.0135]
L_Q10 = [0.9598,-1.4313,1.4556,-0.0569,0.9730,-0.0065]
L_Q11 = [0.9598,-1.3961,1.6727,-0.3093,0.9730,-0.0065]
L_Q12 = [0.9598,-1.3021,1.8322,-0.5627,0.9730,-0.0065]
L_Q13 = [0.9598,-1.1512,1.9340,-0.8154,0.9730,-0.0065]

R_Q1 = [1.8932,-1.1341,1.8966,-0.7540,1.9410,0.2056]
R_Q2 = [1.8932,-1.2798,1.7942,-0.5058,1.9410,0.2056]
R_Q3 = [0.9598,-1.3961,1.6727,-0.3093,0.9730,-0.0065]
R_Q4 = [0.9598,-1.4313,1.4556,-0.0569,0.9730,-0.0065]
R_Q5 = [1.1579,-1.4956,1.5274,-0.0610,1.1711,-0.0135]
R_Q6 = [1.3835,-1.5071,1.5397,-0.0600,1.3966,-0.0201]
R_Q7 = [1.6155,-1.4641,1.4927,-0.0557,1.6285,-0.0264]
R_Q8 = [1.3835,-1.5071,1.5397,-0.0600,1.3966,-0.0201]
R_Q9 = [1.1579,-1.4956,1.5274,-0.0610,1.1711,-0.0135]
R_Q10 = [0.9598,-1.4313,1.4556,-0.0569,0.9730,-0.0065]
R_Q11 = [0.9598,-1.3961,1.6727,-0.3093,0.9730,-0.0065]
R_Q12 = [0.9598,-1.3021,1.8322,-0.5627,0.9730,-0.0065]
R_Q13 = [0.9598,-1.1512,1.9340,-0.8154,0.9730,-0.0065]

left_client = None
right_client = None

def move():

    global left_joints_pos
    global right_joints_pos

    gl = FollowJointTrajectoryGoal()
    gl.trajectory = JointTrajectory()
    gl.trajectory.joint_names = L_JOINT_NAMES

    gr = FollowJointTrajectoryGoal()
    gr.trajectory = JointTrajectory()
    gr.trajectory.joint_names = R_JOINT_NAMES

    try:
        left_joint_states = rospy.wait_for_message("/left/left_joint_states", JointState)
        left_joints_pos = left_joint_states.position
        gl.trajectory.points = [
            JointTrajectoryPoint(positions=left_joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0)),
            JointTrajectoryPoint(positions=L_Q1, velocities=[0]*6, time_from_start=rospy.Duration(2.0)),
            JointTrajectoryPoint(positions=L_Q2, velocities=[0]*6, time_from_start=rospy.Duration(3.0)),
            JointTrajectoryPoint(positions=L_Q3, velocities=[0]*6, time_from_start=rospy.Duration(4.0)),
            JointTrajectoryPoint(positions=L_Q4, velocities=[0]*6, time_from_start=rospy.Duration(5.0)),
            JointTrajectoryPoint(positions=L_Q5, velocities=[0]*6, time_from_start=rospy.Duration(6.0)),
            JointTrajectoryPoint(positions=L_Q6, velocities=[0]*6, time_from_start=rospy.Duration(7.0)),
            JointTrajectoryPoint(positions=L_Q7, velocities=[0]*6, time_from_start=rospy.Duration(8.0)),
            JointTrajectoryPoint(positions=L_Q8, velocities=[0]*6, time_from_start=rospy.Duration(9.0)),
            JointTrajectoryPoint(positions=L_Q9, velocities=[0]*6, time_from_start=rospy.Duration(10.0)),
            JointTrajectoryPoint(positions=L_Q10, velocities=[0]*6, time_from_start=rospy.Duration(11.0)),
            JointTrajectoryPoint(positions=L_Q11, velocities=[0]*6, time_from_start=rospy.Duration(12.0)),
            JointTrajectoryPoint(positions=L_Q12, velocities=[0]*6, time_from_start=rospy.Duration(13.0)),
            JointTrajectoryPoint(positions=L_Q13, velocities=[0]*6, time_from_start=rospy.Duration(14.0))]

        right_joint_states = rospy.wait_for_message("/right/right_joint_states", JointState)
        right_joints_pos = left_joint_states.position
        gr.trajectory.points = [
            JointTrajectoryPoint(positions=right_joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0)),
            JointTrajectoryPoint(positions=R_Q1, velocities=[0]*6, time_from_start=rospy.Duration(2.0)),
            JointTrajectoryPoint(positions=R_Q2, velocities=[0]*6, time_from_start=rospy.Duration(3.0)),
            JointTrajectoryPoint(positions=R_Q3, velocities=[0]*6, time_from_start=rospy.Duration(4.0)),
            JointTrajectoryPoint(positions=R_Q4, velocities=[0]*6, time_from_start=rospy.Duration(5.0)),
            JointTrajectoryPoint(positions=R_Q5, velocities=[0]*6, time_from_start=rospy.Duration(6.0)),
            JointTrajectoryPoint(positions=R_Q6, velocities=[0]*6, time_from_start=rospy.Duration(7.0)),
            JointTrajectoryPoint(positions=R_Q7, velocities=[0]*6, time_from_start=rospy.Duration(8.0)),
            JointTrajectoryPoint(positions=R_Q8, velocities=[0]*6, time_from_start=rospy.Duration(9.0)),
            JointTrajectoryPoint(positions=R_Q9, velocities=[0]*6, time_from_start=rospy.Duration(10.0)),
            JointTrajectoryPoint(positions=R_Q10, velocities=[0]*6, time_from_start=rospy.Duration(11.0)),
            JointTrajectoryPoint(positions=R_Q11, velocities=[0]*6, time_from_start=rospy.Duration(12.0)),
            JointTrajectoryPoint(positions=R_Q12, velocities=[0]*6, time_from_start=rospy.Duration(13.0)),
            JointTrajectoryPoint(positions=R_Q13, velocities=[0]*6, time_from_start=rospy.Duration(14.0))]


        left_client.send_goal(gl)
        right_client.send_goal(gr)

        right_client.wait_for_result()# or left_client.wait_for_result()
    except KeyboardInterrupt:
        left_client.cancel_goal()
        right_client.cancel_goal()
        raise
    except:
        raise

   
def main():
    global left_client
    global right_client
    try:
        rospy.init_node("double_ur5", anonymous=True, disable_signals=True)
        left_client = actionlib.SimpleActionClient('left/left_follow_joint_trajectory', FollowJointTrajectoryAction)
        right_client = actionlib.SimpleActionClient('right/right_follow_joint_trajectory', FollowJointTrajectoryAction)
        print "Waiting for robots' servers..."
        left_client.wait_for_server()
        right_client.wait_for_server()
        print "Connected to robots' servers"
        move()
        print "Trajectory finished"
    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise

if __name__ == '__main__': main()
