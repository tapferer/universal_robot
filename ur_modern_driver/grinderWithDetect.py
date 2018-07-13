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

Q1 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q2 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q3 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q4 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q5 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q6 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q7 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q8 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q9 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q10 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q11 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q12 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q13 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q14 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q15 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q16 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q17 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q18 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q19 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q20 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q21 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q22 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q23 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q24 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q25 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q26 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q27 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q28 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q29 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q30 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q31 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q32 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q33 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q34 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q35 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q36 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q37 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q38 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q39 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q40 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q41 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q42 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q43 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q44 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q45 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q46 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q47 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q48 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q49 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q50 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q51 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q52 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q53 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q54 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q55 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q56 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q57 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q58 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q59 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q60 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q61 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q62 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q63 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q64 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q65 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q66 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q67 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q68 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q69 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q70 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q71 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q72 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q73 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q74 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q75 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q76 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q77 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q78 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q79 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q80 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q81 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q82 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q83 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q84 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q85 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q86 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q87 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q88 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q89 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q90 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q91 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q92 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q93 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q94 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q95 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q96 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q97 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q98 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q99 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q100 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]
Q101 = [8.72359e-317, 6.95291e-310, 6.93317e-310, 1.80236e-314, 4.94066e-324, 1.5316e-322]


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
        rospy.init_node("grinder_move", anonymous=True, disable_signals=True)
        client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
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
