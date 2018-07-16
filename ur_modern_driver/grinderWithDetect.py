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

Q1 = [1.97747, -1.22859, 2.06742, -2.37663, -1.59046, -0.2229]
Q2 = [1.97659, -1.22963, 2.06903, -2.37721, -1.59049, -0.223786]
Q3 = [1.9757, -1.23067, 2.07063, -2.3778, -1.59052, -0.224674]
Q4 = [1.97481, -1.23171, 2.07224, -2.37838, -1.59055, -0.225564]
Q5 = [1.97392, -1.23275, 2.07384, -2.37896, -1.59058, -0.226456]
Q6 = [1.97302, -1.23379, 2.07544, -2.37953, -1.59061, -0.22735]
Q7 = [1.97213, -1.23483, 2.07704, -2.38011, -1.59064, -0.228246]
Q8 = [1.97123, -1.23587, 2.07863, -2.38069, -1.59067, -0.229144]
Q9 = [1.97033, -1.23691, 2.08023, -2.38126, -1.5907, -0.230044]
Q10 = [1.96942, -1.23795, 2.08182, -2.38183, -1.59073, -0.230946]
Q11 = [1.96852, -1.23899, 2.08342, -2.3824, -1.59076, -0.231849]
Q12 = [1.96762, -1.24003, 2.08501, -2.38297, -1.59079, -0.232755]
Q13 = [1.96671, -1.24106, 2.0866, -2.38354, -1.59082, -0.233662]
Q14 = [1.9658, -1.2421, 2.08819, -2.38411, -1.59085, -0.234572]
Q15 = [1.96489, -1.24314, 2.08977, -2.38468, -1.59088, -0.235483]
Q16 = [1.96397, -1.24418, 2.09136, -2.38524, -1.59091, -0.236397]
Q17 = [1.96306, -1.24522, 2.09294, -2.38581, -1.59094, -0.237312]
Q18 = [1.96214, -1.24626, 2.09453, -2.38637, -1.59097, -0.23823]
Q19 = [1.96122, -1.2473, 2.09611, -2.38693, -1.591, -0.239149]
Q20 = [1.9603, -1.24834, 2.09769, -2.38749, -1.59103, -0.240071]
Q21 = [1.95937, -1.24938, 2.09927, -2.38805, -1.59106, -0.240994]
Q22 = [1.95845, -1.25042, 2.10084, -2.38861, -1.59109, -0.241919]
Q23 = [1.95752, -1.25145, 2.10242, -2.38916, -1.59112, -0.242847]
Q24 = [1.95659, -1.25249, 2.104, -2.38972, -1.59115, -0.243776]
Q25 = [1.95566, -1.25353, 2.10557, -2.39027, -1.59118, -0.244708]
Q26 = [1.95472, -1.25457, 2.10714, -2.39082, -1.59121, -0.245641]
Q27 = [1.95379, -1.25561, 2.10871, -2.39137, -1.59124, -0.246577]
Q28 = [1.95285, -1.25665, 2.11028, -2.39192, -1.59127, -0.247514]
Q29 = [1.95191, -1.25769, 2.11185, -2.39247, -1.5913, -0.248454]
Q30 = [1.95097, -1.25872, 2.11341, -2.39302, -1.59133, -0.249396]
Q31 = [1.95002, -1.25976, 2.11498, -2.39356, -1.59136, -0.250339]
Q32 = [1.94908, -1.2608, 2.11654, -2.39411, -1.59139, -0.251285]
Q33 = [1.94813, -1.26184, 2.1181, -2.39465, -1.59142, -0.252233]
Q34 = [1.94718, -1.26288, 2.11967, -2.39519, -1.59145, -0.253183]
Q35 = [1.94623, -1.26392, 2.12122, -2.39574, -1.59148, -0.254135]
Q36 = [1.94527, -1.26495, 2.12278, -2.39627, -1.59151, -0.255089]
Q37 = [1.94432, -1.26599, 2.12434, -2.39681, -1.59155, -0.256045]
Q38 = [1.94336, -1.26703, 2.12589, -2.39735, -1.59158, -0.257003]
Q39 = [1.9424, -1.26807, 2.12745, -2.39789, -1.59161, -0.257964]
Q40 = [1.94144, -1.26911, 2.129, -2.39842, -1.59164, -0.258926]
Q41 = [1.94047, -1.27015, 2.13055, -2.39895, -1.59167, -0.259891]
Q42 = [1.9395, -1.27118, 2.1321, -2.39948, -1.5917, -0.260858]
Q43 = [1.93853, -1.27222, 2.13365, -2.40001, -1.59173, -0.261827]
Q44 = [1.93756, -1.27326, 2.1352, -2.40054, -1.59176, -0.262798]
Q45 = [1.93659, -1.2743, 2.13674, -2.40107, -1.59179, -0.263771]
Q46 = [1.93561, -1.27534, 2.13829, -2.4016, -1.59183, -0.264746]
Q47 = [1.93464, -1.27637, 2.13983, -2.40212, -1.59186, -0.265723]
Q48 = [1.93366, -1.27741, 2.14137, -2.40265, -1.59189, -0.266703]
Q49 = [1.93267, -1.27845, 2.14291, -2.40317, -1.59192, -0.267685]
Q50 = [1.93169, -1.27949, 2.14445, -2.40369, -1.59195, -0.268669]
Q51 = [1.9307, -1.28053, 2.14598, -2.40421, -1.59198, -0.269655]
Q52 = [1.92972, -1.28157, 2.14752, -2.40473, -1.59201, -0.270643]
Q53 = [1.92872, -1.2826, 2.14905, -2.40524, -1.59205, -0.271634]
Q54 = [1.92773, -1.28364, 2.15059, -2.40576, -1.59208, -0.272626]
Q55 = [1.92674, -1.28468, 2.15212, -2.40627, -1.59211, -0.273621]
Q56 = [1.92574, -1.28572, 2.15365, -2.40679, -1.59214, -0.274618]
Q57 = [1.92474, -1.28676, 2.15518, -2.4073, -1.59217, -0.275617]
Q58 = [1.92374, -1.2878, 2.15671, -2.40781, -1.59221, -0.276619]
Q59 = [1.92273, -1.28883, 2.15823, -2.40832, -1.59224, -0.277623]
Q60 = [1.92173, -1.28987, 2.15976, -2.40883, -1.59227, -0.278629]
Q61 = [1.92072, -1.29091, 2.16128, -2.40933, -1.5923, -0.279637]
Q62 = [1.91971, -1.29195, 2.1628, -2.40984, -1.59233, -0.280647]
Q63 = [1.9187, -1.29299, 2.16432, -2.41034, -1.59237, -0.28166]
Q64 = [1.91768, -1.29403, 2.16584, -2.41084, -1.5924, -0.282675]
Q65 = [1.91666, -1.29507, 2.16736, -2.41135, -1.59243, -0.283692]
Q66 = [1.91564, -1.2961, 2.16887, -2.41184, -1.59246, -0.284711]
Q67 = [1.91462, -1.29714, 2.17039, -2.41234, -1.5925, -0.285733]
Q68 = [1.9136, -1.29818, 2.1719, -2.41284, -1.59253, -0.286757]
Q69 = [1.91257, -1.29922, 2.17341, -2.41334, -1.59256, -0.287783]
Q70 = [1.91154, -1.30026, 2.17493, -2.41383, -1.59259, -0.288812]
Q71 = [1.91051, -1.3013, 2.17643, -2.41432, -1.59263, -0.289843]
Q72 = [1.90948, -1.30234, 2.17794, -2.41481, -1.59266, -0.290876]
Q73 = [1.90844, -1.30338, 2.17945, -2.4153, -1.59269, -0.291912]
Q74 = [1.9074, -1.30441, 2.18095, -2.41579, -1.59272, -0.292949]
Q75 = [1.90636, -1.30545, 2.18246, -2.41628, -1.59276, -0.29399]
Q76 = [1.90532, -1.30649, 2.18396, -2.41677, -1.59279, -0.295032]
Q77 = [1.90427, -1.30753, 2.18546, -2.41725, -1.59282, -0.296077]
Q78 = [1.90323, -1.30857, 2.18696, -2.41774, -1.59286, -0.297124]
Q79 = [1.90218, -1.30961, 2.18846, -2.41822, -1.59289, -0.298173]
Q80 = [1.90113, -1.31065, 2.18996, -2.4187, -1.59292, -0.299225]
Q81 = [1.90007, -1.31169, 2.19145, -2.41918, -1.59295, -0.30028]
Q82 = [1.89901, -1.31273, 2.19295, -2.41966, -1.59299, -0.301336]
Q83 = [1.89795, -1.31377, 2.19444, -2.42013, -1.59302, -0.302395]
Q84 = [1.89689, -1.31481, 2.19593, -2.42061, -1.59305, -0.303456]
Q85 = [1.89583, -1.31585, 2.19742, -2.42108, -1.59309, -0.30452]
Q86 = [1.89476, -1.31689, 2.19891, -2.42155, -1.59312, -0.305586]
Q87 = [1.89369, -1.31793, 2.20039, -2.42202, -1.59315, -0.306655]
Q88 = [1.89262, -1.31897, 2.20188, -2.42249, -1.59319, -0.307726]
Q89 = [1.89155, -1.32001, 2.20336, -2.42296, -1.59322, -0.308799]
Q90 = [1.89047, -1.32105, 2.20485, -2.42343, -1.59325, -0.309875]
Q91 = [1.88939, -1.32209, 2.20633, -2.42389, -1.59329, -0.310953]
Q92 = [1.88831, -1.32313, 2.20781, -2.42436, -1.59332, -0.312034]
Q93 = [1.88723, -1.32417, 2.20929, -2.42482, -1.59336, -0.313117]
Q94 = [1.88614, -1.32521, 2.21076, -2.42528, -1.59339, -0.314203]
Q95 = [1.88506, -1.32625, 2.21224, -2.42574, -1.59342, -0.315291]
Q96 = [1.88397, -1.32729, 2.21371, -2.4262, -1.59346, -0.316381]
Q97 = [1.88287, -1.32833, 2.21519, -2.42666, -1.59349, -0.317474]
Q98 = [1.88178, -1.32937, 2.21666, -2.42711, -1.59352, -0.318569]
Q99 = [1.88068, -1.33041, 2.21813, -2.42757, -1.59356, -0.319667]
Q100 = [1.87958, -1.33145, 2.2196, -2.42802, -1.59359, -0.320768]
Q101 = [1.87847, -1.33249, 2.22106, -2.42847, -1.59363, -0.321871]


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
