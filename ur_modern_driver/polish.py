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

Q1 = [1.8897,-1.1908,2.0406,-2.4398,-1.5347,-0.37315]
Q2 = [1.8888,-1.1913,2.0413,-2.44,-1.5347,-0.37399]
Q3 = [1.888,-1.1917,2.042,-2.4402,-1.5346,-0.37482]
Q4 = [1.8872,-1.1921,2.0426,-2.4404,-1.5346,-0.37566]
Q5 = [1.8863,-1.1925,2.0433,-2.4406,-1.5346,-0.3765]
Q6 = [1.8855,-1.1929,2.0439,-2.4408,-1.5346,-0.37733]
Q7 = [1.8846,-1.1933,2.0446,-2.4411,-1.5346,-0.37817]
Q8 = [1.8838,-1.1938,2.0452,-2.4413,-1.5346,-0.37901]
Q9 = [1.883,-1.1942,2.0459,-2.4415,-1.5345,-0.37985]
Q10 = [1.8821,-1.1946,2.0465,-2.4417,-1.5345,-0.38069]
Q11 = [1.8813,-1.195,2.0472,-2.4419,-1.5345,-0.38153]
Q12 = [1.8804,-1.1954,2.0478,-2.4421,-1.5345,-0.38238]
Q13 = [1.8796,-1.1958,2.0485,-2.4423,-1.5345,-0.38322]
Q14 = [1.8788,-1.1962,2.0491,-2.4425,-1.5345,-0.38406]
Q15 = [1.8779,-1.1966,2.0498,-2.4427,-1.5344,-0.3849]
Q16 = [1.8771,-1.1971,2.0504,-2.4429,-1.5344,-0.38575]
Q17 = [1.8762,-1.1975,2.0511,-2.4431,-1.5344,-0.38659]
Q18 = [1.8754,-1.1979,2.0517,-2.4433,-1.5344,-0.38744]
Q19 = [1.8745,-1.1983,2.0524,-2.4435,-1.5344,-0.38828]
Q20 = [1.8737,-1.1987,2.053,-2.4437,-1.5344,-0.38913]
Q21 = [1.8728,-1.1991,2.0536,-2.4439,-1.5344,-0.38998]
Q22 = [1.872,-1.1995,2.0543,-2.4441,-1.5343,-0.39083]
Q23 = [1.8712,-1.1999,2.0549,-2.4443,-1.5343,-0.39168]
Q24 = [1.8703,-1.2003,2.0556,-2.4445,-1.5343,-0.39252]
Q25 = [1.8695,-1.2007,2.0562,-2.4447,-1.5343,-0.39337]
Q26 = [1.8686,-1.2011,2.0568,-2.4449,-1.5343,-0.39422]
Q27 = [1.8678,-1.2016,2.0575,-2.4451,-1.5343,-0.39508]
Q28 = [1.8669,-1.202,2.0581,-2.4453,-1.5342,-0.39593]
Q29 = [1.8661,-1.2024,2.0587,-2.4455,-1.5342,-0.39678]
Q30 = [1.8652,-1.2028,2.0594,-2.4457,-1.5342,-0.39763]
Q31 = [1.8643,-1.2032,2.06,-2.4459,-1.5342,-0.39849]
Q32 = [1.8635,-1.2036,2.0606,-2.4461,-1.5342,-0.39934]
Q33 = [1.8626,-1.204,2.0613,-2.4463,-1.5342,-0.40019]
Q34 = [1.8618,-1.2044,2.0619,-2.4465,-1.5341,-0.40105]
Q35 = [1.8609,-1.2048,2.0625,-2.4467,-1.5341,-0.40191]
Q36 = [1.8601,-1.2052,2.0631,-2.4469,-1.5341,-0.40276]
Q37 = [1.8592,-1.2056,2.0638,-2.4471,-1.5341,-0.40362]
Q38 = [1.8584,-1.206,2.0644,-2.4472,-1.5341,-0.40448]
Q39 = [1.8575,-1.2064,2.065,-2.4474,-1.5341,-0.40533]
Q40 = [1.8566,-1.2068,2.0656,-2.4476,-1.5341,-0.40619]
Q41 = [1.8558,-1.2072,2.0663,-2.4478,-1.534,-0.40705]
Q42 = [1.8549,-1.2076,2.0669,-2.448,-1.534,-0.40791]
Q43 = [1.8541,-1.208,2.0675,-2.4482,-1.534,-0.40877]
Q44 = [1.8532,-1.2084,2.0681,-2.4484,-1.534,-0.40964]
Q45 = [1.8523,-1.2088,2.0687,-2.4486,-1.534,-0.4105]
Q46 = [1.8515,-1.2092,2.0693,-2.4488,-1.534,-0.41136]
Q47 = [1.8506,-1.2096,2.07,-2.449,-1.5339,-0.41222]
Q48 = [1.8498,-1.21,2.0706,-2.4491,-1.5339,-0.41309]
Q49 = [1.8489,-1.2104,2.0712,-2.4493,-1.5339,-0.41395]
Q50 = [1.848,-1.2108,2.0718,-2.4495,-1.5339,-0.41482]
Q51 = [1.8472,-1.2112,2.0724,-2.4497,-1.5339,-0.41568]
Q52 = [1.8463,-1.2116,2.073,-2.4499,-1.5339,-0.41655]
Q53 = [1.8454,-1.2119,2.0736,-2.4501,-1.5339,-0.41742]
Q54 = [1.8446,-1.2123,2.0742,-2.4502,-1.5338,-0.41828]
Q55 = [1.8437,-1.2127,2.0748,-2.4504,-1.5338,-0.41915]
Q56 = [1.8428,-1.2131,2.0755,-2.4506,-1.5338,-0.42002]
Q57 = [1.842,-1.2135,2.0761,-2.4508,-1.5338,-0.42089]
Q58 = [1.8411,-1.2139,2.0767,-2.451,-1.5338,-0.42176]
Q59 = [1.8402,-1.2143,2.0773,-2.4512,-1.5338,-0.42263]
Q60 = [1.8393,-1.2147,2.0779,-2.4513,-1.5337,-0.4235]
Q61 = [1.8385,-1.2151,2.0785,-2.4515,-1.5337,-0.42437]
Q62 = [1.8376,-1.2155,2.0791,-2.4517,-1.5337,-0.42525]
Q63 = [1.8367,-1.2159,2.0797,-2.4519,-1.5337,-0.42612]
Q64 = [1.8359,-1.2162,2.0803,-2.4521,-1.5337,-0.42699]
Q65 = [1.835,-1.2166,2.0809,-2.4522,-1.5337,-0.42787]
Q66 = [1.8341,-1.217,2.0815,-2.4524,-1.5337,-0.42874]
Q67 = [1.8332,-1.2174,2.0821,-2.4526,-1.5336,-0.42962]
Q68 = [1.8324,-1.2178,2.0827,-2.4528,-1.5336,-0.43049]
Q69 = [1.8315,-1.2182,2.0833,-2.4529,-1.5336,-0.43137]
Q70 = [1.8306,-1.2186,2.0838,-2.4531,-1.5336,-0.43225]
Q71 = [1.8297,-1.2189,2.0844,-2.4533,-1.5336,-0.43312]
Q72 = [1.8288,-1.2193,2.085,-2.4535,-1.5336,-0.434]
Q73 = [1.828,-1.2197,2.0856,-2.4536,-1.5336,-0.43488]
Q74 = [1.8271,-1.2201,2.0862,-2.4538,-1.5335,-0.43576]
Q75 = [1.8262,-1.2205,2.0868,-2.454,-1.5335,-0.43664]
Q76 = [1.8253,-1.2209,2.0874,-2.4542,-1.5335,-0.43752]
Q77 = [1.8245,-1.2212,2.088,-2.4543,-1.5335,-0.4384]
Q78 = [1.8236,-1.2216,2.0886,-2.4545,-1.5335,-0.43928]
Q79 = [1.8227,-1.222,2.0891,-2.4547,-1.5335,-0.44017]
Q80 = [1.8218,-1.2224,2.0897,-2.4548,-1.5334,-0.44105]
Q81 = [1.8209,-1.2228,2.0903,-2.455,-1.5334,-0.44193]
Q82 = [1.82,-1.2231,2.0909,-2.4552,-1.5334,-0.44282]
Q83 = [1.8192,-1.2235,2.0915,-2.4553,-1.5334,-0.4437]
Q84 = [1.8183,-1.2239,2.092,-2.4555,-1.5334,-0.44459]
Q85 = [1.8174,-1.2243,2.0926,-2.4557,-1.5334,-0.44547]
Q86 = [1.8165,-1.2247,2.0932,-2.4558,-1.5334,-0.44636]
Q87 = [1.8156,-1.225,2.0938,-2.456,-1.5333,-0.44725]
Q88 = [1.8147,-1.2254,2.0943,-2.4562,-1.5333,-0.44813]
Q89 = [1.8138,-1.2258,2.0949,-2.4563,-1.5333,-0.44902]
Q90 = [1.8129,-1.2262,2.0955,-2.4565,-1.5333,-0.44991]
Q91 = [1.8121,-1.2265,2.0961,-2.4567,-1.5333,-0.4508]
Q92 = [1.8112,-1.2269,2.0966,-2.4568,-1.5333,-0.45169]
Q93 = [1.8103,-1.2273,2.0972,-2.457,-1.5333,-0.45258]
Q94 = [1.8094,-1.2276,2.0978,-2.4572,-1.5332,-0.45347]
Q95 = [1.8085,-1.228,2.0983,-2.4573,-1.5332,-0.45436]
Q96 = [1.8076,-1.2284,2.0989,-2.4575,-1.5332,-0.45526]
Q97 = [1.8067,-1.2288,2.0995,-2.4576,-1.5332,-0.45615]
Q98 = [1.8058,-1.2291,2.1,-2.4578,-1.5332,-0.45704]
Q99 = [1.8049,-1.2295,2.1006,-2.458,-1.5332,-0.45794]
Q100 = [1.804,-1.2299,2.1012,-2.4581,-1.5332,-0.45883]
Q101 = [1.8082,-1.2463,2.1267,-2.4674,-1.5332,-0.45471]
Q102 = [1.8134,-1.2624,2.1515,-2.4762,-1.5333,-0.44948]
Q103 = [1.8188,-1.2785,2.1759,-2.4848,-1.5334,-0.44404]
Q104 = [1.8245,-1.2947,2.2,-2.493,-1.5335,-0.43838]
Q105 = [1.8304,-1.3109,2.2239,-2.5008,-1.5336,-0.43248]
Q106 = [1.8365,-1.3271,2.2474,-2.5083,-1.5337,-0.42632]
Q107 = [1.843,-1.3434,2.2707,-2.5155,-1.5338,-0.41989]
Q108 = [1.8497,-1.3598,2.2936,-2.5223,-1.5339,-0.41318]
Q109 = [1.8567,-1.3763,2.3162,-2.5287,-1.5341,-0.40616]
Q110 = [1.864,-1.3929,2.3386,-2.5348,-1.5342,-0.39882]
Q111 = [1.8717,-1.4096,2.3606,-2.5404,-1.5343,-0.39113]


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
            JointTrajectoryPoint(positions=Q101, velocities=[0]*6, time_from_start=rospy.Duration(10.1)),
            JointTrajectoryPoint(positions=Q102, velocities=[0]*6, time_from_start=rospy.Duration(10.2)),
            JointTrajectoryPoint(positions=Q103, velocities=[0]*6, time_from_start=rospy.Duration(10.3)),
            JointTrajectoryPoint(positions=Q104, velocities=[0]*6, time_from_start=rospy.Duration(10.4)),
            JointTrajectoryPoint(positions=Q105, velocities=[0]*6, time_from_start=rospy.Duration(10.5)),
            JointTrajectoryPoint(positions=Q106, velocities=[0]*6, time_from_start=rospy.Duration(10.6)),
            JointTrajectoryPoint(positions=Q107, velocities=[0]*6, time_from_start=rospy.Duration(10.7)),
            JointTrajectoryPoint(positions=Q108, velocities=[0]*6, time_from_start=rospy.Duration(10.8)),
            JointTrajectoryPoint(positions=Q109, velocities=[0]*6, time_from_start=rospy.Duration(10.9)),
            JointTrajectoryPoint(positions=Q110, velocities=[0]*6, time_from_start=rospy.Duration(11)),
            JointTrajectoryPoint(positions=Q111, velocities=[0]*6, time_from_start=rospy.Duration(11.1))]
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
