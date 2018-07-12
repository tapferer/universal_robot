#!/usr/bin/env python
import time
import roslib; roslib.load_manifest('ur_modern_driver')
import rospy
import actionlib
import sys
import subprocess
import os
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState
from math import pi

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

Q1 = [1, -1, 1, 1, 2, 3]
Q2 = [1, -0.99999, 0.99575, 1.00424, 2, 3]
Q3 = [1, -0.999961, 0.991469, 1.00849, 2, 3]
Q4 = [1, -0.999911, 0.987156, 1.01276, 2, 3]
Q5 = [1, -0.999842, 0.98281, 1.01703, 2, 3]
Q6 = [1, -0.999752, 0.978432, 1.02132, 2, 3]
Q7 = [1, -0.999641, 0.974021, 1.02562, 2, 3]
Q8 = [1, -0.99951, 0.969577, 1.02993, 2, 3]
Q9 = [1, -0.999358, 0.965099, 1.03426, 2, 3]
Q10 = [1, -0.999185, 0.960586, 1.0386, 2, 3]
Q11 = [1, -0.99899, 0.95604, 1.04295, 2, 3]
Q12 = [1, -0.998774, 0.951458, 1.04732, 2, 3]
Q13 = [1, -0.998536, 0.946841, 1.0517, 2, 3]
Q14 = [1, -0.998277, 0.942188, 1.05609, 2, 3]
Q15 = [1, -0.997995, 0.937498, 1.0605, 2, 3]
Q16 = [1, -0.99769, 0.932772, 1.06492, 2, 3]
Q17 = [1, -0.997364, 0.928009, 1.06935, 2, 3]
Q18 = [1, -0.997014, 0.923208, 1.07381, 2, 3]
Q19 = [1, -0.996641, 0.918368, 1.07827, 2, 3]
Q20 = [1, -0.996245, 0.913489, 1.08276, 2, 3]
Q21 = [1, -0.995825, 0.908572, 1.08725, 2, 3]
Q22 = [1, -0.995381, 0.903614, 1.09177, 2, 3]
Q23 = [1, -0.994913, 0.898615, 1.0963, 2, 3]
Q24 = [1, -0.994421, 0.893575, 1.10085, 2, 3]
Q25 = [1, -0.993903, 0.888494, 1.10541, 2, 3]
Q26 = [1, -0.993361, 0.88337, 1.10999, 2, 3]
Q27 = [1, -0.992793, 0.878202, 1.11459, 2, 3]
Q28 = [1, -0.992199, 0.872991, 1.11921, 2, 3]
Q29 = [1, -0.99158, 0.867736, 1.12384, 2, 3]
Q30 = [1, -0.990934, 0.862435, 1.1285, 2, 3]
Q31 = [1, -0.990261, 0.857088, 1.13317, 2, 3]
Q32 = [1, -0.98956, 0.851694, 1.13787, 2, 3]
Q33 = [1, -0.988833, 0.846252, 1.14258, 2, 3]
Q34 = [1, -0.988077, 0.840762, 1.14732, 2, 3]
Q35 = [1, -0.987293, 0.835222, 1.15207, 2, 3]
Q36 = [1, -0.986479, 0.829632, 1.15685, 2, 3]
Q37 = [1, -0.985637, 0.823991, 1.16165, 2, 3]
Q38 = [1, -0.984765, 0.818297, 1.16647, 2, 3]
Q39 = [1, -0.983862, 0.81255, 1.17131, 2, 3]
Q40 = [1, -0.982929, 0.806749, 1.17618, 2, 3]
Q41 = [1, -0.981964, 0.800892, 1.18107, 2, 3]
Q42 = [1, -0.980968, 0.794979, 1.18599, 2, 3]
Q43 = [1, -0.979939, 0.789007, 1.19093, 2, 3]
Q44 = [1, -0.978877, 0.782977, 1.1959, 2, 3]
Q45 = [1, -0.977781, 0.776886, 1.2009, 2, 3]
Q46 = [1, -0.976651, 0.770734, 1.20592, 2, 3]
Q47 = [1, -0.975487, 0.764518, 1.21097, 2, 3]
Q48 = [1, -0.974286, 0.758238, 1.21605, 2, 3]
Q49 = [1, -0.973049, 0.751891, 1.22116, 2, 3]
Q50 = [1, -0.971775, 0.745477, 1.2263, 2, 3]
Q51 = [1, -0.970463, 0.738993, 1.23147, 2, 3]
Q52 = [1, -0.969112, 0.732438, 1.23667, 2, 3]
Q53 = [1, -0.967721, 0.72581, 1.24191, 2, 3]
Q54 = [1, -0.966289, 0.719107, 1.24718, 2, 3]
Q55 = [1, -0.964816, 0.712327, 1.25249, 2, 3]
Q56 = [1, -0.963301, 0.705468, 1.25783, 2, 3]
Q57 = [1, -0.961741, 0.698527, 1.26321, 2, 3]
Q58 = [1, -0.960137, 0.691503, 1.26863, 2, 3]
Q59 = [1, -0.958487, 0.684392, 1.27409, 2, 3]
Q60 = [1, -0.956789, 0.677192, 1.2796, 2, 3]
Q61 = [1, -0.955043, 0.669901, 1.28514, 2, 3]
Q62 = [1, -0.953246, 0.662515, 1.29073, 2, 3]
Q63 = [1, -0.951399, 0.655031, 1.29637, 2, 3]
Q64 = [1, -0.949497, 0.647446, 1.30205, 2, 3]
Q65 = [1, -0.947542, 0.639756, 1.30779, 2, 3]
Q66 = [1, -0.945529, 0.631958, 1.31357, 2, 3]
Q67 = [1, -0.943458, 0.624048, 1.31941, 2, 3]
Q68 = [1, -0.941327, 0.616021, 1.32531, 2, 3]
Q69 = [1, -0.939133, 0.607874, 1.33126, 2, 3]
Q70 = [1, -0.936873, 0.5996, 1.33727, 2, 3]
Q71 = [1, -0.934547, 0.591195, 1.34335, 2, 3]
Q72 = [1, -0.93215, 0.582654, 1.3495, 2, 3]
Q73 = [1, -0.92968, 0.57397, 1.35571, 2, 3]
Q74 = [1, -0.927134, 0.565136, 1.362, 2, 3]
Q75 = [1, -0.924508, 0.556147, 1.36836, 2, 3]
Q76 = [1, -0.921799, 0.546994, 1.37481, 2, 3]
Q77 = [1, -0.919003, 0.537669, 1.38133, 2, 3]
Q78 = [1, -0.916115, 0.528162, 1.38795, 2, 3]
Q79 = [1, -0.913132, 0.518465, 1.39467, 2, 3]
Q80 = [1, -0.910047, 0.508566, 1.40148, 2, 3]
Q81 = [1, -0.906855, 0.498454, 1.4084, 2, 3]
Q82 = [1, -0.903549, 0.488115, 1.41543, 2, 3]
Q83 = [1, -0.900123, 0.477534, 1.42259, 2, 3]
Q84 = [1, -0.896569, 0.466695, 1.42987, 2, 3]
Q85 = [1, -0.892877, 0.455581, 1.4373, 2, 3]
Q86 = [1, -0.889039, 0.44417, 1.44487, 2, 3]
Q87 = [1, -0.885042, 0.432439, 1.4526, 2, 3]
Q88 = [1, -0.880875, 0.420362, 1.46051, 2, 3]
Q89 = [1, -0.876522, 0.407907, 1.46862, 2, 3]
Q90 = [1, -0.871966, 0.395039, 1.47693, 2, 3]
Q91 = [1, -0.867188, 0.381717, 1.48547, 2, 3]
Q92 = [1, -0.862163, 0.367892, 1.49427, 2, 3]
Q93 = [1, -0.856863, 0.353503, 1.50336, 2, 3]
Q94 = [1, -0.851255, 0.338481, 1.51277, 2, 3]
Q95 = [1, -0.845295, 0.322736, 1.52256, 2, 3]
Q96 = [1, -0.83893, 0.306157, 1.53277, 2, 3]
Q97 = [1, -0.832091, 0.288601, 1.54349, 2, 3]
Q98 = [1, -0.824687, 0.269877, 1.55481, 2, 3]
Q99 = [1, -0.816592, 0.249722, 1.56687, 2, 3]
Q100 = [1, -0.807623, 0.227758, 1.57986, 2, 3]
Q101 = [1, -0.7975, 0.2034, 1.5941, 2, 3]


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
            JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0)),
            JointTrajectoryPoint(positions=Q1, velocities=[0]*6, time_from_start=rospy.Duration(5)),
            JointTrajectoryPoint(positions=Q2, velocities=[0]*6, time_from_start=rospy.Duration(5.05)),
            JointTrajectoryPoint(positions=Q3, velocities=[0]*6, time_from_start=rospy.Duration(5.1)),
            JointTrajectoryPoint(positions=Q4, velocities=[0]*6, time_from_start=rospy.Duration(5.15)),
            JointTrajectoryPoint(positions=Q5, velocities=[0]*6, time_from_start=rospy.Duration(5.2)),
            JointTrajectoryPoint(positions=Q6, velocities=[0]*6, time_from_start=rospy.Duration(5.25)),
            JointTrajectoryPoint(positions=Q7, velocities=[0]*6, time_from_start=rospy.Duration(5.3)),
            JointTrajectoryPoint(positions=Q8, velocities=[0]*6, time_from_start=rospy.Duration(5.35)),
            JointTrajectoryPoint(positions=Q9, velocities=[0]*6, time_from_start=rospy.Duration(5.4)),
            JointTrajectoryPoint(positions=Q10, velocities=[0]*6, time_from_start=rospy.Duration(5.45)),
            JointTrajectoryPoint(positions=Q11, velocities=[0]*6, time_from_start=rospy.Duration(5.5)),
            JointTrajectoryPoint(positions=Q12, velocities=[0]*6, time_from_start=rospy.Duration(5.55)),
            JointTrajectoryPoint(positions=Q13, velocities=[0]*6, time_from_start=rospy.Duration(5.6)),
            JointTrajectoryPoint(positions=Q14, velocities=[0]*6, time_from_start=rospy.Duration(5.65)),
            JointTrajectoryPoint(positions=Q15, velocities=[0]*6, time_from_start=rospy.Duration(5.7)),
            JointTrajectoryPoint(positions=Q16, velocities=[0]*6, time_from_start=rospy.Duration(5.75)),
            JointTrajectoryPoint(positions=Q17, velocities=[0]*6, time_from_start=rospy.Duration(5.8)),
            JointTrajectoryPoint(positions=Q18, velocities=[0]*6, time_from_start=rospy.Duration(5.85)),
            JointTrajectoryPoint(positions=Q19, velocities=[0]*6, time_from_start=rospy.Duration(5.9)),
            JointTrajectoryPoint(positions=Q20, velocities=[0]*6, time_from_start=rospy.Duration(5.95)),
            JointTrajectoryPoint(positions=Q21, velocities=[0]*6, time_from_start=rospy.Duration(6)),
            JointTrajectoryPoint(positions=Q22, velocities=[0]*6, time_from_start=rospy.Duration(6.05)),
            JointTrajectoryPoint(positions=Q23, velocities=[0]*6, time_from_start=rospy.Duration(6.1)),
            JointTrajectoryPoint(positions=Q24, velocities=[0]*6, time_from_start=rospy.Duration(6.15)),
            JointTrajectoryPoint(positions=Q25, velocities=[0]*6, time_from_start=rospy.Duration(6.2)),
            JointTrajectoryPoint(positions=Q26, velocities=[0]*6, time_from_start=rospy.Duration(6.25)),
            JointTrajectoryPoint(positions=Q27, velocities=[0]*6, time_from_start=rospy.Duration(6.3)),
            JointTrajectoryPoint(positions=Q28, velocities=[0]*6, time_from_start=rospy.Duration(6.35)),
            JointTrajectoryPoint(positions=Q29, velocities=[0]*6, time_from_start=rospy.Duration(6.4)),
            JointTrajectoryPoint(positions=Q30, velocities=[0]*6, time_from_start=rospy.Duration(6.45)),
            JointTrajectoryPoint(positions=Q31, velocities=[0]*6, time_from_start=rospy.Duration(6.5)),
            JointTrajectoryPoint(positions=Q32, velocities=[0]*6, time_from_start=rospy.Duration(6.55)),
            JointTrajectoryPoint(positions=Q33, velocities=[0]*6, time_from_start=rospy.Duration(6.6)),
            JointTrajectoryPoint(positions=Q34, velocities=[0]*6, time_from_start=rospy.Duration(6.65)),
            JointTrajectoryPoint(positions=Q35, velocities=[0]*6, time_from_start=rospy.Duration(6.7)),
            JointTrajectoryPoint(positions=Q36, velocities=[0]*6, time_from_start=rospy.Duration(6.75)),
            JointTrajectoryPoint(positions=Q37, velocities=[0]*6, time_from_start=rospy.Duration(6.8)),
            JointTrajectoryPoint(positions=Q38, velocities=[0]*6, time_from_start=rospy.Duration(6.85)),
            JointTrajectoryPoint(positions=Q39, velocities=[0]*6, time_from_start=rospy.Duration(6.9)),
            JointTrajectoryPoint(positions=Q40, velocities=[0]*6, time_from_start=rospy.Duration(6.95)),
            JointTrajectoryPoint(positions=Q41, velocities=[0]*6, time_from_start=rospy.Duration(7)),
            JointTrajectoryPoint(positions=Q42, velocities=[0]*6, time_from_start=rospy.Duration(7.05)),
            JointTrajectoryPoint(positions=Q43, velocities=[0]*6, time_from_start=rospy.Duration(7.1)),
            JointTrajectoryPoint(positions=Q44, velocities=[0]*6, time_from_start=rospy.Duration(7.15)),
            JointTrajectoryPoint(positions=Q45, velocities=[0]*6, time_from_start=rospy.Duration(7.2)),
            JointTrajectoryPoint(positions=Q46, velocities=[0]*6, time_from_start=rospy.Duration(7.25)),
            JointTrajectoryPoint(positions=Q47, velocities=[0]*6, time_from_start=rospy.Duration(7.3)),
            JointTrajectoryPoint(positions=Q48, velocities=[0]*6, time_from_start=rospy.Duration(7.35)),
            JointTrajectoryPoint(positions=Q49, velocities=[0]*6, time_from_start=rospy.Duration(7.4)),
            JointTrajectoryPoint(positions=Q50, velocities=[0]*6, time_from_start=rospy.Duration(7.45)),
            JointTrajectoryPoint(positions=Q51, velocities=[0]*6, time_from_start=rospy.Duration(7.5)),
            JointTrajectoryPoint(positions=Q52, velocities=[0]*6, time_from_start=rospy.Duration(7.55)),
            JointTrajectoryPoint(positions=Q53, velocities=[0]*6, time_from_start=rospy.Duration(7.6)),
            JointTrajectoryPoint(positions=Q54, velocities=[0]*6, time_from_start=rospy.Duration(7.65)),
            JointTrajectoryPoint(positions=Q55, velocities=[0]*6, time_from_start=rospy.Duration(7.7)),
            JointTrajectoryPoint(positions=Q56, velocities=[0]*6, time_from_start=rospy.Duration(7.75)),
            JointTrajectoryPoint(positions=Q57, velocities=[0]*6, time_from_start=rospy.Duration(7.8)),
            JointTrajectoryPoint(positions=Q58, velocities=[0]*6, time_from_start=rospy.Duration(7.85)),
            JointTrajectoryPoint(positions=Q59, velocities=[0]*6, time_from_start=rospy.Duration(7.9)),
            JointTrajectoryPoint(positions=Q60, velocities=[0]*6, time_from_start=rospy.Duration(7.95)),
            JointTrajectoryPoint(positions=Q61, velocities=[0]*6, time_from_start=rospy.Duration(8)),
            JointTrajectoryPoint(positions=Q62, velocities=[0]*6, time_from_start=rospy.Duration(8.05)),
            JointTrajectoryPoint(positions=Q63, velocities=[0]*6, time_from_start=rospy.Duration(8.1)),
            JointTrajectoryPoint(positions=Q64, velocities=[0]*6, time_from_start=rospy.Duration(8.15)),
            JointTrajectoryPoint(positions=Q65, velocities=[0]*6, time_from_start=rospy.Duration(8.2)),
            JointTrajectoryPoint(positions=Q66, velocities=[0]*6, time_from_start=rospy.Duration(8.25)),
            JointTrajectoryPoint(positions=Q67, velocities=[0]*6, time_from_start=rospy.Duration(8.3)),
            JointTrajectoryPoint(positions=Q68, velocities=[0]*6, time_from_start=rospy.Duration(8.35)),
            JointTrajectoryPoint(positions=Q69, velocities=[0]*6, time_from_start=rospy.Duration(8.4)),
            JointTrajectoryPoint(positions=Q70, velocities=[0]*6, time_from_start=rospy.Duration(8.45)),
            JointTrajectoryPoint(positions=Q71, velocities=[0]*6, time_from_start=rospy.Duration(8.5)),
            JointTrajectoryPoint(positions=Q72, velocities=[0]*6, time_from_start=rospy.Duration(8.55)),
            JointTrajectoryPoint(positions=Q73, velocities=[0]*6, time_from_start=rospy.Duration(8.6)),
            JointTrajectoryPoint(positions=Q74, velocities=[0]*6, time_from_start=rospy.Duration(8.65)),
            JointTrajectoryPoint(positions=Q75, velocities=[0]*6, time_from_start=rospy.Duration(8.7)),
            JointTrajectoryPoint(positions=Q76, velocities=[0]*6, time_from_start=rospy.Duration(8.75)),
            JointTrajectoryPoint(positions=Q77, velocities=[0]*6, time_from_start=rospy.Duration(8.8)),
            JointTrajectoryPoint(positions=Q78, velocities=[0]*6, time_from_start=rospy.Duration(8.85)),
            JointTrajectoryPoint(positions=Q79, velocities=[0]*6, time_from_start=rospy.Duration(8.9)),
            JointTrajectoryPoint(positions=Q80, velocities=[0]*6, time_from_start=rospy.Duration(8.95)),
            JointTrajectoryPoint(positions=Q81, velocities=[0]*6, time_from_start=rospy.Duration(9)),
            JointTrajectoryPoint(positions=Q82, velocities=[0]*6, time_from_start=rospy.Duration(9.05)),
            JointTrajectoryPoint(positions=Q83, velocities=[0]*6, time_from_start=rospy.Duration(9.1)),
            JointTrajectoryPoint(positions=Q84, velocities=[0]*6, time_from_start=rospy.Duration(9.15)),
            JointTrajectoryPoint(positions=Q85, velocities=[0]*6, time_from_start=rospy.Duration(9.2)),
            JointTrajectoryPoint(positions=Q86, velocities=[0]*6, time_from_start=rospy.Duration(9.25)),
            JointTrajectoryPoint(positions=Q87, velocities=[0]*6, time_from_start=rospy.Duration(9.3)),
            JointTrajectoryPoint(positions=Q88, velocities=[0]*6, time_from_start=rospy.Duration(9.35)),
            JointTrajectoryPoint(positions=Q89, velocities=[0]*6, time_from_start=rospy.Duration(9.4)),
            JointTrajectoryPoint(positions=Q90, velocities=[0]*6, time_from_start=rospy.Duration(9.45)),
            JointTrajectoryPoint(positions=Q91, velocities=[0]*6, time_from_start=rospy.Duration(9.5)),
            JointTrajectoryPoint(positions=Q92, velocities=[0]*6, time_from_start=rospy.Duration(9.55)),
            JointTrajectoryPoint(positions=Q93, velocities=[0]*6, time_from_start=rospy.Duration(9.6)),
            JointTrajectoryPoint(positions=Q94, velocities=[0]*6, time_from_start=rospy.Duration(9.65)),
            JointTrajectoryPoint(positions=Q95, velocities=[0]*6, time_from_start=rospy.Duration(9.7)),
            JointTrajectoryPoint(positions=Q96, velocities=[0]*6, time_from_start=rospy.Duration(9.75)),
            JointTrajectoryPoint(positions=Q97, velocities=[0]*6, time_from_start=rospy.Duration(9.8)),
            JointTrajectoryPoint(positions=Q98, velocities=[0]*6, time_from_start=rospy.Duration(9.85)),
            JointTrajectoryPoint(positions=Q99, velocities=[0]*6, time_from_start=rospy.Duration(9.9)),
            JointTrajectoryPoint(positions=Q100, velocities=[0]*6, time_from_start=rospy.Duration(9.95)),
            JointTrajectoryPoint(positions=Q101, velocities=[0]*6, time_from_start=rospy.Duration(10))]
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
        rospy.init_node("simple_move", anonymous=True, disable_signals=True)
        client = actionlib.SimpleActionClient('arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        print "Waiting for server..."
        client.wait_for_server()
        print "Connected to server"
        child1 = subprocess.Popen('rostopic echo /joint_states >1.txt',shell=True)
        move()
        print "Trajectory finished"
        time.sleep(0.5)
        child2 = subprocess.Popen('cp 1.txt data/polish.txt',shell=True)
        time.sleep(0.5)
        child3 = subprocess.Popen('rm 1.txt',shell=True)
        time.sleep(0.5)
        if True:
            child1.kill()
            child2.kill()
            child3.kill()
    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise

if __name__ == '__main__': main()
