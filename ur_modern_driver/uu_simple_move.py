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
Q1=[0,-1.5708,0,1.5708,0,0,0,-1.5708,0,1.5708,0,0]
Q2=[-1.181,-2.1233,2.2074,-1.6549,1.5708,2.7518,0.86094,-1.6655,1.9082,-1.8135,1.5708,0.70986]
Q3=[-1.181,-2.1233,2.2074,-1.6549,1.5708,2.7518,0.86094,-1.6655,1.9082,-1.8135,1.5708,0.70986]
Q4=[-1.181,-2.1233,2.2074,-1.6549,1.5708,2.7518,0.86094,-1.6655,1.9082,-1.8135,1.5708,0.70986]
Q5=[-1.181,-2.1233,2.2074,-1.6549,1.5708,2.7518,0.86094,-1.6655,1.9082,-1.8135,1.5708,0.70986]
Q6=[-1.181,-2.1233,2.2074,-1.6549,1.5708,2.7518,0.86094,-1.6655,1.9082,-1.8135,1.5708,0.70986]
Q7=[-1.181,-2.1233,2.2074,-1.6549,1.5708,2.7518,0.86094,-1.6655,1.9082,-1.8135,1.5708,0.70986]
Q8=[-1.181,-2.1233,2.2074,-1.6549,1.5708,2.7518,0.86094,-1.6655,1.9082,-1.8135,1.5708,0.70986]
Q9=[-1.181,-2.1233,2.2074,-1.6549,1.5708,2.7518,0.86094,-1.6655,1.9082,-1.8135,1.5708,0.70986]
Q10=[-1.181,-2.1233,2.2074,-1.6549,1.5708,2.7518,0.86094,-1.6655,1.9082,-1.8135,1.5708,0.70986]
Q11=[-1.181,-2.1233,2.2074,-1.6549,1.5708,2.7518,0.86094,-1.6655,1.9082,-1.8135,1.5708,0.70986]
Q12=[-1.181,-2.1233,2.2074,-1.6549,1.5708,2.7518,0.86094,-1.6655,1.9082,-1.8135,1.5708,0.70986]
Q13=[-1.2022,-2.08,2.1834,-1.6756,1.5708,2.773,0.82563,-1.7084,1.9417,-1.8032,1.5708,0.74517]
Q14=[-1.2244,-2.0322,2.1582,-1.6973,1.5708,2.7952,0.78741,-1.7557,1.9779,-1.7926,1.5708,0.78338]
Q15=[-1.2465,-1.9843,2.1329,-1.7189,1.5708,2.8173,0.7492,-1.803,2.014,-1.7821,1.5708,0.8216]
Q16=[-1.2687,-1.9365,2.1077,-1.7406,1.5708,2.8395,0.71099,-1.8503,2.0501,-1.7716,1.5708,0.85981]
Q17=[-1.2899,-1.8932,2.0837,-1.7613,1.5708,2.8607,0.67568,-1.8932,2.0837,-1.7613,1.5708,0.89512]
Q18=[-1.3024,-1.8503,2.0501,-1.7716,1.5708,2.8732,0.6228,-1.9365,2.1077,-1.7406,1.5708,0.94799]
Q19=[-1.3152,-1.803,2.014,-1.7821,1.5708,2.886,0.56276,-1.9843,2.1329,-1.7189,1.5708,1.008]
Q20=[-1.328,-1.7557,1.9779,-1.7926,1.5708,2.8988,0.50271,-2.0322,2.1582,-1.6973,1.5708,1.0681]
Q21=[-1.3409,-1.7084,1.9417,-1.8032,1.5708,2.9117,0.44266,-2.08,2.1834,-1.6756,1.5708,1.1281]
Q22=[-1.3534,-1.6655,1.9082,-1.8135,1.5708,2.9242,0.38979,-2.1233,2.2074,-1.6549,1.5708,1.181]
Q23=[-1.3984,-1.6829,1.9229,-1.8107,1.5708,2.9692,0.4292,-2.1609,2.2208,-1.6298,1.5708,1.1416]
Q24=[-1.4484,-1.7009,1.9381,-1.808,1.5708,3.0192,0.47232,-2.2017,2.2345,-1.6033,1.5708,1.0985]
Q25=[-1.4984,-1.7189,1.9533,-1.8052,1.5708,3.0692,0.51543,-2.2426,2.2483,-1.5768,1.5708,1.0554]
Q26=[-1.5483,-1.7369,1.9685,-1.8025,1.5708,3.1191,0.55855,-2.2835,2.262,-1.5503,1.5708,1.0122]
Q27=[-1.5934,-1.7543,1.9832,-1.7997,1.5708,3.119,0.59797,-2.3211,2.2754,-1.5251,1.5708,0.97283]
Q28=[-1.6407,-1.7607,1.9882,-1.7983,1.5708,3.0717,0.66862,-2.3378,2.2793,-1.5121,1.5708,0.90217]
Q29=[-1.6936,-1.7671,1.9933,-1.7969,1.5708,3.0188,0.75367,-2.3552,2.2831,-1.4987,1.5708,0.81713]
Q30=[-1.7465,-1.7736,1.9983,-1.7955,1.5708,2.9659,0.83871,-2.3725,2.287,-1.4853,1.5708,0.73209]
Q31=[-1.7994,-1.7801,2.0033,-1.7941,1.5708,2.9129,0.92375,-2.3899,2.2909,-1.4719,1.5708,0.64704]
Q32=[-1.8468,-1.7865,2.0083,-1.7927,1.5708,2.8656,0.99441,-2.4066,2.2948,-1.4589,1.5708,0.57639]
Q33=[-1.8912,-1.7801,2.0033,-1.7941,1.5708,2.8212,1.0842,-2.3899,2.2909,-1.4719,1.5708,0.48658]
Q34=[-1.9404,-1.7736,1.9983,-1.7955,1.5708,2.772,1.2013,-2.3725,2.287,-1.4853,1.5708,0.3695]
Q35=[-1.9897,-1.7671,1.9933,-1.7969,1.5708,2.7227,1.3184,-2.3552,2.2831,-1.4987,1.5708,0.25242]
Q36=[-2.0389,-1.7607,1.9882,-1.7983,1.5708,2.6735,1.4355,-2.3378,2.2793,-1.5121,1.5708,0.13534]
Q37=[-2.0833,-1.7543,1.9832,-1.7997,1.5708,2.6291,1.5253,-2.3211,2.2754,-1.5251,1.5708,0.045534]
Q38=[-2.1208,-1.7369,1.9685,-1.8025,1.5708,2.5916,1.6018,-2.2835,2.262,-1.5503,1.5708,-0.030958]
Q39=[-2.1616,-1.7189,1.9533,-1.8052,1.5708,2.5508,1.6959,-2.2426,2.2483,-1.5768,1.5708,-0.12507]
Q40=[-2.2024,-1.7009,1.9381,-1.808,1.5708,2.51,1.79,-2.2017,2.2345,-1.6033,1.5708,-0.21918]
Q41=[-2.2432,-1.6829,1.9229,-1.8107,1.5708,2.4692,1.8841,-2.1609,2.2208,-1.6298,1.5708,-0.31329]
Q42=[-2.2807,-1.6655,1.9082,-1.8135,1.5708,2.4317,1.9606,-2.1233,2.2074,-1.6549,1.5708,-0.38979]
Q43=[-2.2807,-1.6655,1.9082,-1.8135,1.5708,2.4317,1.9606,-2.1233,2.2074,-1.6549,1.5708,-0.38979]
Q44=[-2.2807,-1.6655,1.9082,-1.8135,1.5708,2.4317,1.9606,-2.1233,2.2074,-1.6549,1.5708,-0.38979]
Q45=[-2.2807,-1.6655,1.9082,-1.8135,1.5708,2.4317,1.9606,-2.1233,2.2074,-1.6549,1.5708,-0.38979]
Q46=[-2.2807,-1.6655,1.9082,-1.8135,1.5708,2.4317,1.9606,-2.1233,2.2074,-1.6549,1.5708,-0.38979]
Q47=[-2.2807,-1.6655,1.9082,-1.8135,1.5708,2.4317,1.9606,-2.1233,2.2074,-1.6549,1.5708,-0.38979]
Q48=[-2.316,-1.7084,1.9417,-1.8032,1.5708,2.3964,1.9394,-2.08,2.1834,-1.6756,1.5708,-0.36859]
Q49=[-2.3542,-1.7557,1.9779,-1.7926,1.5708,2.3582,1.9172,-2.0322,2.1582,-1.6973,1.5708,-0.34642]
Q50=[-2.3924,-1.803,2.014,-1.7821,1.5708,2.32,1.895,-1.9843,2.1329,-1.7189,1.5708,-0.32425]
Q51=[-2.4306,-1.8503,2.0501,-1.7716,1.5708,2.2818,1.8729,-1.9365,2.1077,-1.7406,1.5708,-0.30208]
Q52=[-2.4659,-1.8932,2.0837,-1.7613,1.5708,2.2465,1.8517,-1.8932,2.0837,-1.7613,1.5708,-0.28089]
Q53=[-2.5363,-1.8932,2.0837,-1.7613,1.5708,2.1761,1.8792,-1.8288,2.0285,-1.7724,1.5708,-0.30843]
Q54=[-2.621,-1.8932,2.0837,-1.7613,1.5708,2.0914,1.9085,-1.7529,1.9653,-1.7839,1.5708,-0.33769]
Q55=[-2.7056,-1.8932,2.0837,-1.7613,1.5708,2.0067,1.9377,-1.677,1.9022,-1.7953,1.5708,-0.36694]
Q56=[-2.7903,-1.8932,2.0837,-1.7613,1.5708,1.9221,1.967,-1.6012,1.839,-1.8067,1.5708,-0.39619]
Q57=[-2.8607,-1.8932,2.0837,-1.7613,1.5708,1.8517,1.9945,-1.5368,1.7838,-1.8178,1.5708,-0.42374]
Q58=[-2.8395,-1.9365,2.1077,-1.7406,1.5708,1.8729,1.9555,-1.5617,1.8079,-1.8169,1.5708,-0.38465]
Q59=[-2.8173,-1.9843,2.1329,-1.7189,1.5708,1.895,1.9127,-1.588,1.8333,-1.8161,1.5708,-0.34194]
Q60=[-2.7952,-2.0322,2.1582,-1.6973,1.5708,1.9172,1.87,-1.6143,1.8587,-1.8152,1.5708,-0.29923]
Q61=[-2.773,-2.08,2.1834,-1.6756,1.5708,1.9394,1.8273,-1.6406,1.8841,-1.8143,1.5708,-0.25652]
Q62=[-2.7518,-2.1233,2.2074,-1.6549,1.5708,1.9606,1.7882,-1.6655,1.9082,-1.8135,1.5708,-0.21744]
Q63=[-2.7124,-2.1609,2.2208,-1.6298,1.5708,2,1.7432,-1.6829,1.9229,-1.8107,1.5708,-0.17242]
Q64=[-2.6693,-2.2017,2.2345,-1.6033,1.5708,2.0431,1.6932,-1.7009,1.9381,-1.808,1.5708,-0.12243]
Q65=[-2.6262,-2.2426,2.2483,-1.5768,1.5708,2.0862,1.6432,-1.7189,1.9533,-1.8052,1.5708,-0.072441]
Q66=[-2.583,-2.2835,2.262,-1.5503,1.5708,2.1293,1.5932,-1.7369,1.9685,-1.8025,1.5708,-0.022452]
Q67=[-2.5436,-2.3211,2.2754,-1.5251,1.5708,2.1688,1.5482,-1.7543,1.9832,-1.7997,1.5708,0.022566]
Q68=[-2.473,-2.3378,2.2793,-1.5121,1.5708,2.2394,1.5009,-1.7607,1.9882,-1.7983,1.5708,0.069909]
Q69=[-2.3879,-2.3552,2.2831,-1.4987,1.5708,2.3245,1.448,-1.7671,1.9933,-1.7969,1.5708,0.12282]
Q70=[-2.3029,-2.3725,2.287,-1.4853,1.5708,2.4095,1.3951,-1.7736,1.9983,-1.7955,1.5708,0.17573]
Q71=[-2.2178,-2.3899,2.2909,-1.4719,1.5708,2.4945,1.3421,-1.7801,2.0033,-1.7941,1.5708,0.22865]
Q72=[-2.1472,-2.4066,2.2948,-1.4589,1.5708,2.5652,1.2948,-1.7865,2.0083,-1.7927,1.5708,0.27599]
Q73=[-2.0574,-2.3899,2.2909,-1.4719,1.5708,2.655,1.2504,-1.7801,2.0033,-1.7941,1.5708,0.3204]
Q74=[-1.9403,-2.3725,2.287,-1.4853,1.5708,2.7721,1.2012,-1.7736,1.9983,-1.7955,1.5708,0.36964]
Q75=[-1.8232,-2.3552,2.2831,-1.4987,1.5708,2.8892,1.1519,-1.7671,1.9933,-1.7969,1.5708,0.41887]
Q76=[-1.7061,-2.3378,2.2793,-1.5121,1.5708,3.0062,1.1027,-1.7607,1.9882,-1.7983,1.5708,0.46811]
Q77=[-1.6163,-2.3211,2.2754,-1.5251,1.5708,3.0961,1.0583,-1.7543,1.9832,-1.7997,1.5708,0.51252]
Q78=[-1.6163,-2.3211,2.2754,-1.5251,1.5708,3.0961,1.0583,-1.7543,1.9832,-1.7997,1.5708,0.51252]
Q79=[-1.6163,-2.3211,2.2754,-1.5251,1.5708,3.0961,1.0583,-1.7543,1.9832,-1.7997,1.5708,0.51252]
Q80=[-1.6163,-2.3211,2.2754,-1.5251,1.5708,3.0961,1.0583,-1.7543,1.9832,-1.7997,1.5708,0.51252]
Q81=[-1.6163,-2.3211,2.2754,-1.5251,1.5708,3.0961,1.0583,-1.7543,1.9832,-1.7997,1.5708,0.51252]
Q82=[-1.6163,-2.3211,2.2754,-1.5251,1.5708,3.0961,1.0583,-1.7543,1.9832,-1.7997,1.5708,0.51252]
Q83=[-1.6163,-2.3211,2.2754,-1.5251,1.5708,3.0961,1.0583,-1.7543,1.9832,-1.7997,1.5708,0.51252]
Q84=[-1.6163,-2.3211,2.2754,-1.5251,1.5708,3.0961,1.0583,-1.7543,1.9832,-1.7997,1.5708,0.51252]
Q85=[-1.6163,-2.3211,2.2754,-1.5251,1.5708,3.0961,1.0583,-1.7543,1.9832,-1.7997,1.5708,0.51252]
Q86=[-1.6163,-2.3211,2.2754,-1.5251,1.5708,3.0961,1.0583,-1.7543,1.9832,-1.7997,1.5708,0.51252]
Q87=[-1.6163,-2.3211,2.2754,-1.5251,1.5708,3.0961,1.0583,-1.7543,1.9832,-1.7997,1.5708,0.51252]
Q88=[-1.6163,-2.3211,2.2754,-1.5251,1.5708,3.0961,1.0583,-1.7543,1.9832,-1.7997,1.5708,0.51252]
Q89=[-1.6163,-2.3211,2.2754,-1.5251,1.5708,3.0961,1.0583,-1.7543,1.9832,-1.7997,1.5708,0.51252]
Q90=[-1.6163,-2.3211,2.2754,-1.5251,1.5708,3.0961,1.0583,-1.7543,1.9832,-1.7997,1.5708,0.51252]
Q91=[-1.6163,-2.3211,2.2754,-1.5251,1.5708,3.0961,1.0583,-1.7543,1.9832,-1.7997,1.5708,0.51252]
Q92=[-1.6163,-2.3211,2.2754,-1.5251,1.5708,3.0961,1.0583,-1.7543,1.9832,-1.7997,1.5708,0.51252]
Q93=[-1.6163,-2.3211,2.2754,-1.5251,1.5708,3.0961,1.0583,-1.7543,1.9832,-1.7997,1.5708,0.51252]
Q94=[-1.6163,-2.3211,2.2754,-1.5251,1.5708,3.0961,1.0583,-1.7543,1.9832,-1.7997,1.5708,0.51252]
Q95=[-1.6163,-2.3211,2.2754,-1.5251,1.5708,3.0961,1.0583,-1.7543,1.9832,-1.7997,1.5708,0.51252]
Q96=[-1.6163,-2.3211,2.2754,-1.5251,1.5708,3.0961,1.0583,-1.7543,1.9832,-1.7997,1.5708,0.51252]
Q97=[-1.6163,-2.3211,2.2754,-1.5251,1.5708,3.0961,1.0583,-1.7543,1.9832,-1.7997,1.5708,0.51252]
Q98=[-1.6163,-2.3211,2.2754,-1.5251,1.5708,3.0961,1.0583,-1.7543,1.9832,-1.7997,1.5708,0.51252]
Q99=[-1.6163,-2.3211,2.2754,-1.5251,1.5708,3.0961,1.0583,-1.7543,1.9832,-1.7997,1.5708,0.51252]
Q100=[-1.6163,-2.3211,2.2754,-1.5251,1.5708,3.0961,1.0583,-1.7543,1.9832,-1.7997,1.5708,0.51252]
Q101=[-1.6163,-2.3211,2.2754,-1.5251,1.5708,3.0961,1.0583,-1.7543,1.9832,-1.7997,1.5708,0.51252]
Q102=[-1.6163,-2.3211,2.2754,-1.5251,1.5708,3.0961,1.0583,-1.7543,1.9832,-1.7997,1.5708,0.51252]
Q103=[-1.6163,-2.3211,2.2754,-1.5251,1.5708,3.0961,1.0583,-1.7543,1.9832,-1.7997,1.5708,0.51252]
Q104=[-1.6163,-2.3211,2.2754,-1.5251,1.5708,3.0961,1.0583,-1.7543,1.9832,-1.7997,1.5708,0.51252]
Q105=[-1.6163,-2.3211,2.2754,-1.5251,1.5708,3.0961,1.0583,-1.7543,1.9832,-1.7997,1.5708,0.51252]
Q106=[-1.6163,-2.3211,2.2754,-1.5251,1.5708,3.0961,1.0583,-1.7543,1.9832,-1.7997,1.5708,0.51252]
Q107=[-1.6163,-2.3211,2.2754,-1.5251,1.5708,3.0961,1.0583,-1.7543,1.9832,-1.7997,1.5708,0.51252]
Q108=[-1.6163,-2.3211,2.2754,-1.5251,1.5708,3.0961,1.0583,-1.7543,1.9832,-1.7997,1.5708,0.51252]
Q109=[-1.6163,-2.3211,2.2754,-1.5251,1.5708,3.0961,1.0583,-1.7543,1.9832,-1.7997,1.5708,0.51252]
Q110=[-1.6163,-2.3211,2.2754,-1.5251,1.5708,3.0961,1.0583,-1.7543,1.9832,-1.7997,1.5708,0.51252]
Q111=[-1.6163,-2.3211,2.2754,-1.5251,1.5708,3.0961,1.0583,-1.7543,1.9832,-1.7997,1.5708,0.51252]
Q112=[-1.6163,-2.3211,2.2754,-1.5251,1.5708,3.0961,1.0583,-1.7543,1.9832,-1.7997,1.5708,0.51252]

    
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
            JointTrajectoryPoint(positions=Q1,velocities=[0,0,0,0,0,0,0,0,0,0,0,0,],time_from_start=rospy.Duration(0.05)),
JointTrajectoryPoint(positions=Q2,velocities=[0,0,0,0,0,0,0,0,0,0,0,0,],time_from_start=rospy.Duration(0.1)),
JointTrajectoryPoint(positions=Q3,velocities=[0,0,0,0,0,0,0,0,0,0,0,0,],time_from_start=rospy.Duration(0.2)),
JointTrajectoryPoint(positions=Q4,velocities=[0,0,0,0,0,0,0,0,0,0,0,0,],time_from_start=rospy.Duration(0.3)),
JointTrajectoryPoint(positions=Q5,velocities=[0,0,0,0,0,0,0,0,0,0,0,0,],time_from_start=rospy.Duration(0.4)),
JointTrajectoryPoint(positions=Q6,velocities=[0,0,0,0,0,0,0,0,0,0,0,0,],time_from_start=rospy.Duration(0.5)),
JointTrajectoryPoint(positions=Q7,velocities=[0,0,0,0,0,0,0,0,0,0,0,0,],time_from_start=rospy.Duration(0.6)),
JointTrajectoryPoint(positions=Q8,velocities=[0,0,0,0,0,0,0,0,0,0,0,0,],time_from_start=rospy.Duration(0.7)),
JointTrajectoryPoint(positions=Q9,velocities=[0,0,0,0,0,0,0,0,0,0,0,0,],time_from_start=rospy.Duration(0.8)),
JointTrajectoryPoint(positions=Q10,velocities=[0,0,0,0,0,0,0,0,0,0,0,0,],time_from_start=rospy.Duration(0.9)),
JointTrajectoryPoint(positions=Q11,velocities=[0,0,0,0,0,0,0,0,0,0,0,0,],time_from_start=rospy.Duration(1)),
JointTrajectoryPoint(positions=Q12,velocities=[-0.21193,0.43293,-0.23985,-0.2072,0,0.21193,-0.35309,-0.42854,0.33536,0.10299,0,0.35309,],time_from_start=rospy.Duration(1.1)),
JointTrajectoryPoint(positions=Q13,velocities=[-0.22171,0.47848,-0.25254,-0.21652,0,0.22171,-0.38214,-0.47306,0.36133,0.10519,0,0.38214,],time_from_start=rospy.Duration(1.2)),
JointTrajectoryPoint(positions=Q14,velocities=[-0.22171,0.47848,-0.25254,-0.21652,0,0.22171,-0.38214,-0.47306,0.36133,0.10519,0,0.38214,],time_from_start=rospy.Duration(1.3)),
JointTrajectoryPoint(positions=Q15,velocities=[-0.22171,0.47848,-0.25254,-0.21652,0,0.22171,-0.38214,-0.47306,0.36133,0.10519,0,0.38214,],time_from_start=rospy.Duration(1.4)),
JointTrajectoryPoint(positions=Q16,velocities=[-0.21193,0.43293,-0.23985,-0.2072,0,0.21193,-0.35309,-0.42854,0.33536,0.10299,0,0.35309,],time_from_start=rospy.Duration(1.5)),
JointTrajectoryPoint(positions=Q17,velocities=[-0.12494,0.42854,-0.33536,-0.10299,0,0.12494,-0.52874,-0.43293,0.23985,0.2072,0,0.52874,],time_from_start=rospy.Duration(1.6)),
JointTrajectoryPoint(positions=Q18,velocities=[-0.12821,0.47306,-0.36133,-0.10519,0,0.12821,-0.60048,-0.47848,0.25254,0.21652,0,0.60048,],time_from_start=rospy.Duration(1.7)),
JointTrajectoryPoint(positions=Q19,velocities=[-0.12821,0.47306,-0.36133,-0.10519,0,0.12821,-0.60048,-0.47848,0.25254,0.21652,0,0.60048,],time_from_start=rospy.Duration(1.8)),
JointTrajectoryPoint(positions=Q20,velocities=[-0.12821,0.47306,-0.36133,-0.10519,0,0.12821,-0.60048,-0.47848,0.25254,0.21652,0,0.60048,],time_from_start=rospy.Duration(1.9)),
JointTrajectoryPoint(positions=Q21,velocities=[-0.12494,0.42854,-0.33536,-0.10299,0,0.12494,-0.52874,-0.43293,0.23985,0.2072,0,0.52874,],time_from_start=rospy.Duration(2)),
JointTrajectoryPoint(positions=Q22,velocities=[-0.45018,-0.17363,0.14734,0.027316,0,0.45018,0.39417,-0.37559,0.13376,0.25113,0,-0.39417,],time_from_start=rospy.Duration(2.1)),
JointTrajectoryPoint(positions=Q23,velocities=[-0.49989,-0.18008,0.15193,0.027466,0,0.49989,0.43116,-0.40884,0.13752,0.26512,0,-0.43116,],time_from_start=rospy.Duration(2.2)),
JointTrajectoryPoint(positions=Q24,velocities=[-0.49989,-0.18008,0.15193,0.027466,0,0.49989,0.43116,-0.40884,0.13752,0.26512,0,-0.43116,],time_from_start=rospy.Duration(2.3)),
JointTrajectoryPoint(positions=Q25,velocities=[-0.49989,-0.18008,0.15193,0.027466,0,0.49989,0.43116,-0.40884,0.13752,0.26512,0,-0.43116,],time_from_start=rospy.Duration(2.4)),
JointTrajectoryPoint(positions=Q26,velocities=[-0.45018,-0.17363,0.14734,0.027316,0,-62.3817,0.39417,-0.37559,0.13376,0.25113,0,-0.39417,],time_from_start=rospy.Duration(2.5)),
JointTrajectoryPoint(positions=Q27,velocities=[-0.47343,-0.063844,0.049868,0.014149,0,0.47343,0.70655,-0.16754,0.038507,0.13031,0,-0.70655,],time_from_start=rospy.Duration(2.6)),
JointTrajectoryPoint(positions=Q28,velocities=[-0.52913,-0.064677,0.050372,0.014189,0,0.52913,0.85043,-0.17353,0.038807,0.13387,0,-0.85043,],time_from_start=rospy.Duration(2.7)),
JointTrajectoryPoint(positions=Q29,velocities=[-0.52913,-0.064677,0.050372,0.014189,0,0.52913,0.85043,-0.17353,0.038807,0.13387,0,-0.85043,],time_from_start=rospy.Duration(2.8)),
JointTrajectoryPoint(positions=Q30,velocities=[-0.52913,-0.064677,0.050372,0.014189,0,0.52913,0.85043,-0.17353,0.038807,0.13387,0,-0.85043,],time_from_start=rospy.Duration(2.9)),
JointTrajectoryPoint(positions=Q31,velocities=[-0.47343,-0.063844,0.049868,0.014149,0,0.47343,0.70655,-0.16754,0.038507,0.13031,0,-0.70655,],time_from_start=rospy.Duration(3)),
JointTrajectoryPoint(positions=Q32,velocities=[-0.44413,0.063844,-0.049868,-0.014149,0,0.44413,0.89809,0.16754,-0.038507,-0.13031,0,-0.89809,],time_from_start=rospy.Duration(3.1)),
JointTrajectoryPoint(positions=Q33,velocities=[-0.49236,0.064677,-0.050372,-0.014189,0,0.49236,1.1708,0.17353,-0.038807,-0.13387,0,-1.1708,],time_from_start=rospy.Duration(3.2)),
JointTrajectoryPoint(positions=Q34,velocities=[-0.49236,0.064677,-0.050372,-0.014189,0,0.49236,1.1708,0.17353,-0.038807,-0.13387,0,-1.1708,],time_from_start=rospy.Duration(3.3)),
JointTrajectoryPoint(positions=Q35,velocities=[-0.49236,0.064677,-0.050372,-0.014189,0,0.49236,1.1708,0.17353,-0.038807,-0.13387,0,-1.1708,],time_from_start=rospy.Duration(3.4)),
JointTrajectoryPoint(positions=Q36,velocities=[-0.44413,0.063844,-0.049868,-0.014149,0,0.44413,0.89809,0.16754,-0.038507,-0.13031,0,-0.89809,],time_from_start=rospy.Duration(3.5)),
JointTrajectoryPoint(positions=Q37,velocities=[-0.37481,0.17363,-0.14734,-0.027316,0,0.37481,0.76492,0.37559,-0.13376,-0.25113,0,-0.76492,],time_from_start=rospy.Duration(3.6)),
JointTrajectoryPoint(positions=Q38,velocities=[-0.40791,0.18008,-0.15193,-0.027466,0,0.40791,0.94112,0.40884,-0.13752,-0.26512,0,-0.94112,],time_from_start=rospy.Duration(3.7)),
JointTrajectoryPoint(positions=Q39,velocities=[-0.40791,0.18008,-0.15193,-0.027466,0,0.40791,0.94112,0.40884,-0.13752,-0.26512,0,-0.94112,],time_from_start=rospy.Duration(3.8)),
JointTrajectoryPoint(positions=Q40,velocities=[-0.40791,0.18008,-0.15193,-0.027466,0,0.40791,0.94112,0.40884,-0.13752,-0.26512,0,-0.94112,],time_from_start=rospy.Duration(3.9)),
JointTrajectoryPoint(positions=Q41,velocities=[-0.37481,0.17363,-0.14734,-0.027316,0,0.37481,0.76492,0.37559,-0.13376,-0.25113,0,-0.76492,],time_from_start=rospy.Duration(4)),
JointTrajectoryPoint(positions=Q42,velocities=[0,0,0,0,0,0,0,0,0,0,0,0,],time_from_start=rospy.Duration(4.1)),
JointTrajectoryPoint(positions=Q43,velocities=[0,0,0,0,0,0,0,0,0,0,0,0,],time_from_start=rospy.Duration(4.2)),
JointTrajectoryPoint(positions=Q44,velocities=[0,0,0,0,0,0,0,0,0,0,0,0,],time_from_start=rospy.Duration(4.3)),
JointTrajectoryPoint(positions=Q45,velocities=[0,0,0,0,0,0,0,0,0,0,0,0,],time_from_start=rospy.Duration(4.4)),
JointTrajectoryPoint(positions=Q46,velocities=[0,0,0,0,0,0,0,0,0,0,0,0,],time_from_start=rospy.Duration(4.5)),
JointTrajectoryPoint(positions=Q47,velocities=[-0.35309,-0.42854,0.33536,0.10299,0,0.35309,-0.21193,0.43293,-0.23985,-0.2072,0,0.21193,],time_from_start=rospy.Duration(4.6)),
JointTrajectoryPoint(positions=Q48,velocities=[-0.38214,-0.47306,0.36133,0.10519,0,0.38214,-0.22171,0.47848,-0.25254,-0.21652,0,0.22171,],time_from_start=rospy.Duration(4.7)),
JointTrajectoryPoint(positions=Q49,velocities=[-0.38214,-0.47306,0.36133,0.10519,0,0.38214,-0.22171,0.47848,-0.25254,-0.21652,0,0.22171,],time_from_start=rospy.Duration(4.8)),
JointTrajectoryPoint(positions=Q50,velocities=[-0.38214,-0.47306,0.36133,0.10519,0,0.38214,-0.22171,0.47848,-0.25254,-0.21652,0,0.22171,],time_from_start=rospy.Duration(4.9)),
JointTrajectoryPoint(positions=Q51,velocities=[-0.35309,-0.42854,0.33536,0.10299,0,0.35309,-0.21193,0.43293,-0.23985,-0.2072,0,0.21193,],time_from_start=rospy.Duration(5)),
JointTrajectoryPoint(positions=Q52,velocities=[-0.70402,0,0,0,0,0.70402,0.27549,0.64409,-0.55215,-0.11148,0,-0.27549,],time_from_start=rospy.Duration(5.1)),
JointTrajectoryPoint(positions=Q53,velocities=[-0.84662,0,0,0,0,0.84662,0.29251,0.75857,-0.63148,-0.11407,0,-0.29251,],time_from_start=rospy.Duration(5.2)),
JointTrajectoryPoint(positions=Q54,velocities=[-0.84662,0,0,0,0,0.84662,0.29251,0.75857,-0.63148,-0.11407,0,-0.29251,],time_from_start=rospy.Duration(5.3)),
JointTrajectoryPoint(positions=Q55,velocities=[-0.84662,0,0,0,0,0.84662,0.29251,0.75857,-0.63148,-0.11407,0,-0.29251,],time_from_start=rospy.Duration(5.4)),
JointTrajectoryPoint(positions=Q56,velocities=[-0.70402,0,0,0,0,0.70402,0.27549,0.64409,-0.55215,-0.11148,0,-0.27549,],time_from_start=rospy.Duration(5.5)),
JointTrajectoryPoint(positions=Q57,velocities=[0.21193,-0.43293,0.23985,0.2072,0,-0.21193,-0.39082,-0.24927,0.24111,0.0087119,0,0.39082,],time_from_start=rospy.Duration(5.6)),
JointTrajectoryPoint(positions=Q58,velocities=[0.22171,-0.47848,0.25254,0.21652,0,-0.22171,-0.42712,-0.26304,0.25394,0.008727,0,0.42712,],time_from_start=rospy.Duration(5.7)),
JointTrajectoryPoint(positions=Q59,velocities=[0.22171,-0.47848,0.25254,0.21652,0,-0.22171,-0.42712,-0.26304,0.25394,0.008727,0,0.42712,],time_from_start=rospy.Duration(5.8)),
JointTrajectoryPoint(positions=Q60,velocities=[0.22171,-0.47848,0.25254,0.21652,0,-0.22171,-0.42712,-0.26304,0.25394,0.008727,0,0.42712,],time_from_start=rospy.Duration(5.9)),
JointTrajectoryPoint(positions=Q61,velocities=[0.21193,-0.43293,0.23985,0.2072,0,-0.21193,-0.39082,-0.24927,0.24111,0.0087119,0,0.39082,],time_from_start=rospy.Duration(6)),
JointTrajectoryPoint(positions=Q62,velocities=[0.39417,-0.37559,0.13376,0.25113,0,-0.39417,-0.45018,-0.17363,0.14734,0.027316,0,0.45018,],time_from_start=rospy.Duration(6.1)),
JointTrajectoryPoint(positions=Q63,velocities=[0.43116,-0.40884,0.13752,0.26512,0,-0.43116,-0.49989,-0.18008,0.15193,0.027466,0,0.49989,],time_from_start=rospy.Duration(6.2)),
JointTrajectoryPoint(positions=Q64,velocities=[0.43116,-0.40884,0.13752,0.26512,0,-0.43116,-0.49989,-0.18008,0.15193,0.027466,0,0.49989,],time_from_start=rospy.Duration(6.3)),
JointTrajectoryPoint(positions=Q65,velocities=[0.43116,-0.40884,0.13752,0.26512,0,-0.43116,-0.49989,-0.18008,0.15193,0.027466,0,0.49989,],time_from_start=rospy.Duration(6.4)),
JointTrajectoryPoint(positions=Q66,velocities=[0.39417,-0.37559,0.13376,0.25113,0,-0.39417,-0.45018,-0.17363,0.14734,0.027316,0,0.45018,],time_from_start=rospy.Duration(6.5)),
JointTrajectoryPoint(positions=Q67,velocities=[0.70655,-0.16754,0.038507,0.13031,0,-0.70655,-0.47343,-0.063844,0.049868,0.014149,0,0.47343,],time_from_start=rospy.Duration(6.6)),
JointTrajectoryPoint(positions=Q68,velocities=[0.85043,-0.17353,0.038807,0.13387,0,-0.85043,-0.52913,-0.064677,0.050372,0.014189,0,0.52913,],time_from_start=rospy.Duration(6.7)),
JointTrajectoryPoint(positions=Q69,velocities=[0.85043,-0.17353,0.038807,0.13387,0,-0.85043,-0.52913,-0.064677,0.050372,0.014189,0,0.52913,],time_from_start=rospy.Duration(6.8)),
JointTrajectoryPoint(positions=Q70,velocities=[0.85043,-0.17353,0.038807,0.13387,0,-0.85043,-0.52913,-0.064677,0.050372,0.014189,0,0.52913,],time_from_start=rospy.Duration(6.9)),
JointTrajectoryPoint(positions=Q71,velocities=[0.70655,-0.16754,0.038507,0.13031,0,-0.70655,-0.47343,-0.063844,0.049868,0.014149,0,0.47343,],time_from_start=rospy.Duration(7)),
JointTrajectoryPoint(positions=Q72,velocities=[0.89809,0.16754,-0.038507,-0.13031,0,-0.89809,-0.44413,0.063844,-0.049868,-0.014149,0,0.44413,],time_from_start=rospy.Duration(7.1)),
JointTrajectoryPoint(positions=Q73,velocities=[1.1708,0.17353,-0.038807,-0.13387,0,-1.1708,-0.49236,0.064677,-0.050372,-0.014189,0,0.49236,],time_from_start=rospy.Duration(7.2)),
JointTrajectoryPoint(positions=Q74,velocities=[1.1708,0.17353,-0.038807,-0.13387,0,-1.1708,-0.49236,0.064677,-0.050372,-0.014189,0,0.49236,],time_from_start=rospy.Duration(7.3)),
JointTrajectoryPoint(positions=Q75,velocities=[1.1708,0.17353,-0.038807,-0.13387,0,-1.1708,-0.49236,0.064677,-0.050372,-0.014189,0,0.49236,],time_from_start=rospy.Duration(7.4)),
JointTrajectoryPoint(positions=Q76,velocities=[0.89809,0.16754,-0.038507,-0.13031,0,-0.89809,-0.44413,0.063844,-0.049868,-0.014149,0,0.44413,],time_from_start=rospy.Duration(7.5)),
JointTrajectoryPoint(positions=Q77,velocities=[0,0,0,0,0,0,0,0,0,0,0,0,],time_from_start=rospy.Duration(7.6)),
JointTrajectoryPoint(positions=Q78,velocities=[0,0,0,0,0,0,0,0,0,0,0,0,],time_from_start=rospy.Duration(7.7)),
JointTrajectoryPoint(positions=Q79,velocities=[0,0,0,0,0,0,0,0,0,0,0,0,],time_from_start=rospy.Duration(7.8)),
JointTrajectoryPoint(positions=Q80,velocities=[0,0,0,0,0,0,0,0,0,0,0,0,],time_from_start=rospy.Duration(7.9)),
JointTrajectoryPoint(positions=Q81,velocities=[0,0,0,0,0,0,0,0,0,0,0,0,],time_from_start=rospy.Duration(8)),
JointTrajectoryPoint(positions=Q82,velocities=[0,0,0,0,0,0,0,0,0,0,0,0,],time_from_start=rospy.Duration(8.1)),
JointTrajectoryPoint(positions=Q83,velocities=[0,0,0,0,0,0,0,0,0,0,0,0,],time_from_start=rospy.Duration(8.2)),
JointTrajectoryPoint(positions=Q84,velocities=[0,0,0,0,0,0,0,0,0,0,0,0,],time_from_start=rospy.Duration(8.3)),
JointTrajectoryPoint(positions=Q85,velocities=[0,0,0,0,0,0,0,0,0,0,0,0,],time_from_start=rospy.Duration(8.4)),
JointTrajectoryPoint(positions=Q86,velocities=[0,0,0,0,0,0,0,0,0,0,0,0,],time_from_start=rospy.Duration(8.5)),
JointTrajectoryPoint(positions=Q87,velocities=[0,0,0,0,0,0,0,0,0,0,0,0,],time_from_start=rospy.Duration(8.6)),
JointTrajectoryPoint(positions=Q88,velocities=[0,0,0,0,0,0,0,0,0,0,0,0,],time_from_start=rospy.Duration(8.7)),
JointTrajectoryPoint(positions=Q89,velocities=[0,0,0,0,0,0,0,0,0,0,0,0,],time_from_start=rospy.Duration(8.8)),
JointTrajectoryPoint(positions=Q90,velocities=[0,0,0,0,0,0,0,0,0,0,0,0,],time_from_start=rospy.Duration(8.9)),
JointTrajectoryPoint(positions=Q91,velocities=[0,0,0,0,0,0,0,0,0,0,0,0,],time_from_start=rospy.Duration(9)),
JointTrajectoryPoint(positions=Q92,velocities=[0,0,0,0,0,0,0,0,0,0,0,0,],time_from_start=rospy.Duration(9.1)),
JointTrajectoryPoint(positions=Q93,velocities=[0,0,0,0,0,0,0,0,0,0,0,0,],time_from_start=rospy.Duration(9.2)),
JointTrajectoryPoint(positions=Q94,velocities=[0,0,0,0,0,0,0,0,0,0,0,0,],time_from_start=rospy.Duration(9.3)),
JointTrajectoryPoint(positions=Q95,velocities=[0,0,0,0,0,0,0,0,0,0,0,0,],time_from_start=rospy.Duration(9.4)),
JointTrajectoryPoint(positions=Q96,velocities=[0,0,0,0,0,0,0,0,0,0,0,0,],time_from_start=rospy.Duration(9.5)),
JointTrajectoryPoint(positions=Q97,velocities=[0,0,0,0,0,0,0,0,0,0,0,0,],time_from_start=rospy.Duration(9.6)),
JointTrajectoryPoint(positions=Q98,velocities=[0,0,0,0,0,0,0,0,0,0,0,0,],time_from_start=rospy.Duration(9.7)),
JointTrajectoryPoint(positions=Q99,velocities=[0,0,0,0,0,0,0,0,0,0,0,0,],time_from_start=rospy.Duration(9.8)),
JointTrajectoryPoint(positions=Q100,velocities=[0,0,0,0,0,0,0,0,0,0,0,0,],time_from_start=rospy.Duration(9.9)),
JointTrajectoryPoint(positions=Q101,velocities=[0,0,0,0,0,0,0,0,0,0,0,0,],time_from_start=rospy.Duration(10)),
JointTrajectoryPoint(positions=Q102,velocities=[0,0,0,0,0,0,0,0,0,0,0,0,],time_from_start=rospy.Duration(10.1)),
JointTrajectoryPoint(positions=Q103,velocities=[0,0,0,0,0,0,0,0,0,0,0,0,],time_from_start=rospy.Duration(10.2)),
JointTrajectoryPoint(positions=Q104,velocities=[0,0,0,0,0,0,0,0,0,0,0,0,],time_from_start=rospy.Duration(10.3)),
JointTrajectoryPoint(positions=Q105,velocities=[0,0,0,0,0,0,0,0,0,0,0,0,],time_from_start=rospy.Duration(10.4)),
JointTrajectoryPoint(positions=Q106,velocities=[0,0,0,0,0,0,0,0,0,0,0,0,],time_from_start=rospy.Duration(10.5)),
JointTrajectoryPoint(positions=Q107,velocities=[0,0,0,0,0,0,0,0,0,0,0,0,],time_from_start=rospy.Duration(10.6)),
JointTrajectoryPoint(positions=Q108,velocities=[0,0,0,0,0,0,0,0,0,0,0,0,],time_from_start=rospy.Duration(10.7)),
JointTrajectoryPoint(positions=Q109,velocities=[0,0,0,0,0,0,0,0,0,0,0,0,],time_from_start=rospy.Duration(10.8)),
JointTrajectoryPoint(positions=Q110,velocities=[0,0,0,0,0,0,0,0,0,0,0,0,],time_from_start=rospy.Duration(10.9)),
JointTrajectoryPoint(positions=Q111,velocities=[0,0,0,0,0,0,0,0,0,0,0,0,],time_from_start=rospy.Duration(11)),
JointTrajectoryPoint(positions=Q112,velocities=[0,0,0,0,0,0,0,0,0,0,0,0,],time_from_start=rospy.Duration(11.1))]

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
