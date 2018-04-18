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
#Q1 = [0,-1.57,0,-1.57,0,0,0,-1.57,0,-1.57,0,0]
Q1 = [0.9598,-1.1512,1.9340,-0.8154,0.9730,-0.0065,1.8932,-1.1341,1.8966,-0.7540,1.9410,0.2056]
Q2 = [0.9598,-1.3021,1.8322,-0.5627,0.9730,-0.0065,1.8932,-1.2798,1.7942,-0.5058,1.9410,0.2056]
Q3 = [0.9598,-1.3961,1.6727,-0.3093,0.9730,-0.0065,1.8932,-1.3700,1.6341,-0.2554,1.9410,0.2056]
Q4 = [0.9598,-1.4313,1.4556,-0.0569,0.9730,-0.0065,1.8932,-1.4025,1.4151,-0.0040,1.9410,0.2056]
Q5 = [1.1579,-1.4956,1.5274,-0.0610,1.1711,-0.0135,1.6706,-1.5101,1.5360,-0.0178,1.7184,0.2037]
Q6 = [1.3835,-1.5071,1.5397,-0.0600,1.3966,-0.0201,1.4231,-1.5682,1.5961,-0.0199,1.4709,0.2017]
Q7 = [1.6155,-1.4641,1.4927,-0.0557,1.6285,-0.0264,1.1770,-1.5681,1.5960,-0.0194,1.2249,0.1997]
Q8 = [1.3835,-1.5071,1.5397,-0.0600,1.3966,-0.0201,1.4231,-1.5682,1.5961,-0.0199,1.4709,0.2017]
Q9 = [1.1579,-1.4956,1.5274,-0.0610,1.1711,-0.0135,1.6706,-1.5101,1.5360,-0.0178,1.7184,0.2037]
Q10 = [0.9598,-1.4313,1.4556,-0.0569,0.9730,-0.0065,1.8932,-1.4025,1.4151,-0.0040,1.9410,0.2056]
Q11 = [0.9598,-1.3961,1.6727,-0.3093,0.9730,-0.0065,1.8932,-1.3700,1.6341,-0.2554,1.9410,0.2056]
Q12 = [0.9598,-1.3021,1.8322,-0.5627,0.9730,-0.0065,1.8932,-1.2798,1.7942,-0.5058,1.9410,0.2056]
Q13 = [0.9598,-1.1512,1.9340,-0.8154,0.9730,-0.0065,1.8932,-1.1341,1.8966,-0.7540,1.9410,0.2056]
    
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
            JointTrajectoryPoint(positions=Q3, velocities=[0]*12, time_from_start=rospy.Duration(4.0)),
            JointTrajectoryPoint(positions=Q4, velocities=[0]*12, time_from_start=rospy.Duration(5.0)),
            JointTrajectoryPoint(positions=Q5, velocities=[0]*12, time_from_start=rospy.Duration(6.0)),
            JointTrajectoryPoint(positions=Q6, velocities=[0]*12, time_from_start=rospy.Duration(7.0)),
            JointTrajectoryPoint(positions=Q7, velocities=[0]*12, time_from_start=rospy.Duration(8.0)),
            JointTrajectoryPoint(positions=Q8, velocities=[0]*12, time_from_start=rospy.Duration(9.0)),
            JointTrajectoryPoint(positions=Q9, velocities=[0]*12, time_from_start=rospy.Duration(10.0)),
            JointTrajectoryPoint(positions=Q10, velocities=[0]*12, time_from_start=rospy.Duration(11.0)),
            JointTrajectoryPoint(positions=Q11, velocities=[0]*12, time_from_start=rospy.Duration(12.0)),
            JointTrajectoryPoint(positions=Q12, velocities=[0]*12, time_from_start=rospy.Duration(13.0)),
            JointTrajectoryPoint(positions=Q13, velocities=[0]*12, time_from_start=rospy.Duration(14.0))]
            #JointTrajectoryPoint(positions=Q14, velocities=[0]*12, time_from_start=rospy.Duration(4.0))
            #JointTrajectoryPoint(positions=Q15, velocities=[0]*12, time_from_start=rospy.Duration(4.0))

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
