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

Q1 = [2.18462, -1.00166, 1.87644, -2.43004, -1.58809, -0.00119508]
Q2 = [2.18378, -1.00236, 1.87778, -2.4307, -1.5881, -0.00203583]
Q3 = [2.18293, -1.00306, 1.87913, -2.43136, -1.58812, -0.00287775]
Q4 = [2.18209, -1.00377, 1.88047, -2.43202, -1.58813, -0.00372084]
Q5 = [2.18125, -1.00447, 1.88182, -2.43267, -1.58814, -0.00456512]
Q6 = [2.1804, -1.00517, 1.88316, -2.43333, -1.58815, -0.00541057]
Q7 = [2.17955, -1.00587, 1.8845, -2.43398, -1.58817, -0.0062572]
Q8 = [2.17871, -1.00657, 1.88584, -2.43464, -1.58818, -0.00710502]
Q9 = [2.17786, -1.00727, 1.88717, -2.43529, -1.58819, -0.00795402]
Q10 = [2.17701, -1.00796, 1.88851, -2.43594, -1.58821, -0.0088042]
Q11 = [2.17616, -1.00866, 1.88984, -2.43659, -1.58822, -0.00965557]
Q12 = [2.1753, -1.00936, 1.89117, -2.43724, -1.58823, -0.0105081]
Q13 = [2.17445, -1.01005, 1.89251, -2.43789, -1.58825, -0.0113619]
Q14 = [2.1736, -1.01075, 1.89383, -2.43854, -1.58826, -0.0122168]
Q15 = [2.17274, -1.01144, 1.89516, -2.43919, -1.58827, -0.013073]
Q16 = [2.17188, -1.01214, 1.89649, -2.43984, -1.58829, -0.0139303]
Q17 = [2.17102, -1.01283, 1.89781, -2.44048, -1.5883, -0.0147888]
Q18 = [2.17016, -1.01352, 1.89914, -2.44113, -1.58831, -0.0156485]
Q19 = [2.1693, -1.01422, 1.90046, -2.44178, -1.58832, -0.0165094]
Q20 = [2.16844, -1.01491, 1.90178, -2.44242, -1.58834, -0.0173716]
Q21 = [2.16758, -1.0156, 1.9031, -2.44306, -1.58835, -0.0182349]
Q22 = [2.16671, -1.01629, 1.90442, -2.44371, -1.58836, -0.0190994]
Q23 = [2.16585, -1.01698, 1.90573, -2.44435, -1.58838, -0.0199651]
Q24 = [2.16498, -1.01766, 1.90705, -2.44499, -1.58839, -0.0208321]
Q25 = [2.16411, -1.01835, 1.90836, -2.44563, -1.5884, -0.0217002]
Q26 = [2.16324, -1.01904, 1.90967, -2.44627, -1.58842, -0.0225696]
Q27 = [2.16237, -1.01972, 1.91098, -2.44691, -1.58843, -0.0234401]
Q28 = [2.1615, -1.02041, 1.91229, -2.44755, -1.58844, -0.0243119]
Q29 = [2.16063, -1.0211, 1.9136, -2.44818, -1.58846, -0.0251849]
Q30 = [2.15975, -1.02178, 1.9149, -2.44882, -1.58847, -0.0260591]
Q31 = [2.15888, -1.02246, 1.9162, -2.44946, -1.58848, -0.0269345]
Q32 = [2.158, -1.02315, 1.91751, -2.45009, -1.5885, -0.0278112]
Q33 = [2.15712, -1.02383, 1.91881, -2.45073, -1.58851, -0.0286891]
Q34 = [2.15624, -1.02451, 1.92011, -2.45136, -1.58852, -0.0295681]
Q35 = [2.15536, -1.02519, 1.92141, -2.45199, -1.58854, -0.0304484]
Q36 = [2.15448, -1.02587, 1.9227, -2.45262, -1.58855, -0.03133]
Q37 = [2.1536, -1.02655, 1.924, -2.45325, -1.58856, -0.0322127]
Q38 = [2.15272, -1.02723, 1.92529, -2.45389, -1.58858, -0.0330967]
Q39 = [2.15183, -1.02791, 1.92658, -2.45451, -1.58859, -0.0339819]
Q40 = [2.15094, -1.02858, 1.92787, -2.45514, -1.5886, -0.0348684]
Q41 = [2.15006, -1.02926, 1.92916, -2.45577, -1.58862, -0.0357561]
Q42 = [2.14917, -1.02994, 1.93045, -2.4564, -1.58863, -0.036645]
Q43 = [2.14828, -1.03061, 1.93174, -2.45703, -1.58864, -0.0375351]
Q44 = [2.14739, -1.03129, 1.93302, -2.45765, -1.58866, -0.0384265]
Q45 = [2.14649, -1.03196, 1.9343, -2.45828, -1.58867, -0.0393191]
Q46 = [2.1456, -1.03264, 1.93558, -2.4589, -1.58868, -0.040213]
Q47 = [2.14471, -1.03331, 1.93686, -2.45952, -1.58869, -0.0411081]
Q48 = [2.14381, -1.03398, 1.93814, -2.46015, -1.58871, -0.0420045]
Q49 = [2.14291, -1.03465, 1.93942, -2.46077, -1.58872, -0.0429021]
Q50 = [2.14201, -1.03532, 1.9407, -2.46139, -1.58873, -0.0438009]
Q51 = [2.14111, -1.03599, 1.94197, -2.46201, -1.58875, -0.044701]
Q52 = [2.14021, -1.03666, 1.94324, -2.46263, -1.58876, -0.0456023]
Q53 = [2.13931, -1.03733, 1.94451, -2.46325, -1.58877, -0.0465049]
Q54 = [2.1384, -1.038, 1.94578, -2.46387, -1.58879, -0.0474087]
Q55 = [2.1375, -1.03867, 1.94705, -2.46448, -1.5888, -0.0483138]
Q56 = [2.13659, -1.03933, 1.94832, -2.4651, -1.58881, -0.0492201]
Q57 = [2.13569, -1.04, 1.94958, -2.46571, -1.58883, -0.0501277]
Q58 = [2.13478, -1.04066, 1.95085, -2.46633, -1.58884, -0.0510366]
Q59 = [2.13387, -1.04133, 1.95211, -2.46694, -1.58885, -0.0519467]
Q60 = [2.13296, -1.04199, 1.95337, -2.46756, -1.58887, -0.0528581]
Q61 = [2.13204, -1.04265, 1.95463, -2.46817, -1.58888, -0.0537707]
Q62 = [2.13113, -1.04332, 1.95589, -2.46878, -1.58889, -0.0546846]
Q63 = [2.13021, -1.04398, 1.95714, -2.46939, -1.58891, -0.0555998]
Q64 = [2.1293, -1.04464, 1.9584, -2.47, -1.58892, -0.0565162]
Q65 = [2.12838, -1.0453, 1.95965, -2.47061, -1.58893, -0.0574339]
Q66 = [2.12746, -1.04596, 1.9609, -2.47122, -1.58895, -0.0583529]
Q67 = [2.12654, -1.04662, 1.96215, -2.47183, -1.58896, -0.0592731]
Q68 = [2.12562, -1.04728, 1.9634, -2.47244, -1.58897, -0.0601946]
Q69 = [2.1247, -1.04794, 1.96465, -2.47304, -1.58899, -0.0611174]
Q70 = [2.12377, -1.04859, 1.9659, -2.47365, -1.589, -0.0620414]
Q71 = [2.12285, -1.04925, 1.96714, -2.47425, -1.58901, -0.0629668]
Q72 = [2.12192, -1.04991, 1.96838, -2.47486, -1.58903, -0.0638934]
Q73 = [2.12099, -1.05056, 1.96963, -2.47546, -1.58904, -0.0648212]
Q74 = [2.12006, -1.05122, 1.97087, -2.47606, -1.58905, -0.0657504]
Q75 = [2.11913, -1.05187, 1.9721, -2.47667, -1.58907, -0.0666809]
Q76 = [2.1182, -1.05252, 1.97334, -2.47727, -1.58908, -0.0676126]
Q77 = [2.11727, -1.05317, 1.97458, -2.47787, -1.5891, -0.0685456]
Q78 = [2.11634, -1.05383, 1.97581, -2.47847, -1.58911, -0.0694799]
Q79 = [2.1154, -1.05448, 1.97704, -2.47907, -1.58912, -0.0704155]
Q80 = [2.11446, -1.05513, 1.97828, -2.47966, -1.58914, -0.0713524]
Q81 = [2.11352, -1.05578, 1.97951, -2.48026, -1.58915, -0.0722905]
Q82 = [2.11259, -1.05643, 1.98073, -2.48086, -1.58916, -0.07323]
Q83 = [2.11164, -1.05707, 1.98196, -2.48145, -1.58918, -0.0741707]
Q84 = [2.1107, -1.05772, 1.98319, -2.48205, -1.58919, -0.0751128]
Q85 = [2.10976, -1.05837, 1.98441, -2.48264, -1.5892, -0.0760561]
Q86 = [2.10881, -1.05901, 1.98563, -2.48324, -1.58922, -0.0770007]
Q87 = [2.10787, -1.05966, 1.98685, -2.48383, -1.58923, -0.0779467]
Q88 = [2.10692, -1.06031, 1.98807, -2.48442, -1.58924, -0.0788939]
Q89 = [2.10597, -1.06095, 1.98929, -2.48501, -1.58926, -0.0798424]
Q90 = [2.10502, -1.06159, 1.99051, -2.4856, -1.58927, -0.0807923]
Q91 = [2.10407, -1.06224, 1.99172, -2.48619, -1.58928, -0.0817434]
Q92 = [2.10312, -1.06288, 1.99293, -2.48678, -1.5893, -0.0826959]
Q93 = [2.10217, -1.06352, 1.99415, -2.48737, -1.58931, -0.0836496]
Q94 = [2.10121, -1.06416, 1.99536, -2.48796, -1.58932, -0.0846047]
Q95 = [2.10026, -1.0648, 1.99657, -2.48854, -1.58934, -0.085561]
Q96 = [2.0993, -1.06544, 1.99777, -2.48913, -1.58935, -0.0865187]
Q97 = [2.09834, -1.06608, 1.99898, -2.48971, -1.58936, -0.0874777]
Q98 = [2.09738, -1.06672, 2.00018, -2.4903, -1.58938, -0.088438]
Q99 = [2.09642, -1.06735, 2.00139, -2.49088, -1.58939, -0.0893996]
Q100 = [2.09545, -1.06799, 2.00259, -2.49146, -1.5894, -0.0903625]
Q101 = [2.09449, -1.06863, 2.00379, -2.49205, -1.58942, -0.0913268]


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
