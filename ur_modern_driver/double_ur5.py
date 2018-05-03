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

L_Q1=[0,-1.5708,0,1.5708,0,0]
L_Q2=[-1.181,-2.1233,2.2074,-1.6549,1.5708,2.7518]
L_Q3=[-1.181,-2.1233,2.2074,-1.6549,1.5708,2.7518]
L_Q4=[-1.181,-2.1233,2.2074,-1.6549,1.5708,2.7518]
L_Q5=[-1.181,-2.1233,2.2074,-1.6549,1.5708,2.7518]
L_Q6=[-1.181,-2.1233,2.2074,-1.6549,1.5708,2.7518]
L_Q7=[-1.181,-2.1233,2.2074,-1.6549,1.5708,2.7518]
L_Q8=[-1.181,-2.1233,2.2074,-1.6549,1.5708,2.7518]
L_Q9=[-1.181,-2.1233,2.2074,-1.6549,1.5708,2.7518]
L_Q10=[-1.181,-2.1233,2.2074,-1.6549,1.5708,2.7518]
L_Q11=[-1.181,-2.1233,2.2074,-1.6549,1.5708,2.7518]
L_Q12=[-1.181,-2.1233,2.2074,-1.6549,1.5708,2.7518]
L_Q13=[-1.2022,-2.08,2.1834,-1.6756,1.5708,2.773]
L_Q14=[-1.2244,-2.0322,2.1582,-1.6973,1.5708,2.7952]
L_Q15=[-1.2465,-1.9843,2.1329,-1.7189,1.5708,2.8173]
L_Q16=[-1.2687,-1.9365,2.1077,-1.7406,1.5708,2.8395]
L_Q17=[-1.2899,-1.8932,2.0837,-1.7613,1.5708,2.8607]
L_Q18=[-1.3024,-1.8503,2.0501,-1.7716,1.5708,2.8732]
L_Q19=[-1.3152,-1.803,2.014,-1.7821,1.5708,2.886]
L_Q20=[-1.328,-1.7557,1.9779,-1.7926,1.5708,2.8988]
L_Q21=[-1.3409,-1.7084,1.9417,-1.8032,1.5708,2.9117]
L_Q22=[-1.3534,-1.6655,1.9082,-1.8135,1.5708,2.9242]
L_Q23=[-1.3984,-1.6829,1.9229,-1.8107,1.5708,2.9692]
L_Q24=[-1.4484,-1.7009,1.9381,-1.808,1.5708,3.0192]
L_Q25=[-1.4984,-1.7189,1.9533,-1.8052,1.5708,3.0692]
L_Q26=[-1.5483,-1.7369,1.9685,-1.8025,1.5708,3.1191]
L_Q27=[-1.5934,-1.7543,1.9832,-1.7997,1.5708,-3.119]
L_Q28=[-1.6407,-1.7607,1.9882,-1.7983,1.5708,-3.0717]
L_Q29=[-1.6936,-1.7671,1.9933,-1.7969,1.5708,-3.0188]
L_Q30=[-1.7465,-1.7736,1.9983,-1.7955,1.5708,-2.9659]
L_Q31=[-1.7994,-1.7801,2.0033,-1.7941,1.5708,-2.9129]
L_Q32=[-1.8468,-1.7865,2.0083,-1.7927,1.5708,-2.8656]
L_Q33=[-1.8912,-1.7801,2.0033,-1.7941,1.5708,-2.8212]
L_Q34=[-1.9404,-1.7736,1.9983,-1.7955,1.5708,-2.772]
L_Q35=[-1.9897,-1.7671,1.9933,-1.7969,1.5708,-2.7227]
L_Q36=[-2.0389,-1.7607,1.9882,-1.7983,1.5708,-2.6735]
L_Q37=[-2.0833,-1.7543,1.9832,-1.7997,1.5708,-2.6291]
L_Q38=[-2.1208,-1.7369,1.9685,-1.8025,1.5708,-2.5916]
L_Q39=[-2.1616,-1.7189,1.9533,-1.8052,1.5708,-2.5508]
L_Q40=[-2.2024,-1.7009,1.9381,-1.808,1.5708,-2.51]
L_Q41=[-2.2432,-1.6829,1.9229,-1.8107,1.5708,-2.4692]
L_Q42=[-2.2807,-1.6655,1.9082,-1.8135,1.5708,-2.4317]
L_Q43=[-2.2807,-1.6655,1.9082,-1.8135,1.5708,-2.4317]
L_Q44=[-2.2807,-1.6655,1.9082,-1.8135,1.5708,-2.4317]
L_Q45=[-2.2807,-1.6655,1.9082,-1.8135,1.5708,-2.4317]
L_Q46=[-2.2807,-1.6655,1.9082,-1.8135,1.5708,-2.4317]
L_Q47=[-2.2807,-1.6655,1.9082,-1.8135,1.5708,-2.4317]
L_Q48=[-2.316,-1.7084,1.9417,-1.8032,1.5708,-2.3964]
L_Q49=[-2.3542,-1.7557,1.9779,-1.7926,1.5708,-2.3582]
L_Q50=[-2.3924,-1.803,2.014,-1.7821,1.5708,-2.32]
L_Q51=[-2.4306,-1.8503,2.0501,-1.7716,1.5708,-2.2818]
L_Q52=[-2.4659,-1.8932,2.0837,-1.7613,1.5708,-2.2465]
L_Q53=[-2.5363,-1.8932,2.0837,-1.7613,1.5708,-2.1761]
L_Q54=[-2.621,-1.8932,2.0837,-1.7613,1.5708,-2.0914]
L_Q55=[-2.7056,-1.8932,2.0837,-1.7613,1.5708,-2.0067]
L_Q56=[-2.7903,-1.8932,2.0837,-1.7613,1.5708,-1.9221]
L_Q57=[-2.8607,-1.8932,2.0837,-1.7613,1.5708,-1.8517]
L_Q58=[-2.8395,-1.9365,2.1077,-1.7406,1.5708,-1.8729]
L_Q59=[-2.8173,-1.9843,2.1329,-1.7189,1.5708,-1.895]
L_Q60=[-2.7952,-2.0322,2.1582,-1.6973,1.5708,-1.9172]
L_Q61=[-2.773,-2.08,2.1834,-1.6756,1.5708,-1.9394]
L_Q62=[-2.7518,-2.1233,2.2074,-1.6549,1.5708,-1.9606]
L_Q63=[-2.7124,-2.1609,2.2208,-1.6298,1.5708,-2]
L_Q64=[-2.6693,-2.2017,2.2345,-1.6033,1.5708,-2.0431]
L_Q65=[-2.6262,-2.2426,2.2483,-1.5768,1.5708,-2.0862]
L_Q66=[-2.583,-2.2835,2.262,-1.5503,1.5708,-2.1293]
L_Q67=[-2.5436,-2.3211,2.2754,-1.5251,1.5708,-2.1688]
L_Q68=[-2.473,-2.3378,2.2793,-1.5121,1.5708,-2.2394]
L_Q69=[-2.3879,-2.3552,2.2831,-1.4987,1.5708,-2.3245]
L_Q70=[-2.3029,-2.3725,2.287,-1.4853,1.5708,-2.4095]
L_Q71=[-2.2178,-2.3899,2.2909,-1.4719,1.5708,-2.4945]
L_Q72=[-2.1472,-2.4066,2.2948,-1.4589,1.5708,-2.5652]
L_Q73=[-2.0574,-2.3899,2.2909,-1.4719,1.5708,-2.655]
L_Q74=[-1.9403,-2.3725,2.287,-1.4853,1.5708,-2.7721]
L_Q75=[-1.8232,-2.3552,2.2831,-1.4987,1.5708,-2.8892]
L_Q76=[-1.7061,-2.3378,2.2793,-1.5121,1.5708,-3.0062]
L_Q77=[-1.6163,-2.3211,2.2754,-1.5251,1.5708,-3.0961]
L_Q78=[-1.6163,-2.3211,2.2754,-1.5251,1.5708,-3.0961]
L_Q79=[-1.6163,-2.3211,2.2754,-1.5251,1.5708,-3.0961]
L_Q80=[-1.6163,-2.3211,2.2754,-1.5251,1.5708,-3.0961]
L_Q81=[-1.6163,-2.3211,2.2754,-1.5251,1.5708,-3.0961]
L_Q82=[-1.6163,-2.3211,2.2754,-1.5251,1.5708,-3.0961]
L_Q83=[-1.6163,-2.3211,2.2754,-1.5251,1.5708,-3.0961]
L_Q84=[-1.6163,-2.3211,2.2754,-1.5251,1.5708,-3.0961]
L_Q85=[-1.6163,-2.3211,2.2754,-1.5251,1.5708,-3.0961]
L_Q86=[-1.6163,-2.3211,2.2754,-1.5251,1.5708,-3.0961]
L_Q87=[-1.6163,-2.3211,2.2754,-1.5251,1.5708,-3.0961]
L_Q88=[-1.6163,-2.3211,2.2754,-1.5251,1.5708,-3.0961]
L_Q89=[-1.6163,-2.3211,2.2754,-1.5251,1.5708,-3.0961]
L_Q90=[-1.6163,-2.3211,2.2754,-1.5251,1.5708,-3.0961]
L_Q91=[-1.6163,-2.3211,2.2754,-1.5251,1.5708,-3.0961]
L_Q92=[-1.6163,-2.3211,2.2754,-1.5251,1.5708,-3.0961]
L_Q93=[-1.6163,-2.3211,2.2754,-1.5251,1.5708,-3.0961]
L_Q94=[-1.6163,-2.3211,2.2754,-1.5251,1.5708,-3.0961]
L_Q95=[-1.6163,-2.3211,2.2754,-1.5251,1.5708,-3.0961]
L_Q96=[-1.6163,-2.3211,2.2754,-1.5251,1.5708,-3.0961]
L_Q97=[-1.6163,-2.3211,2.2754,-1.5251,1.5708,-3.0961]
L_Q98=[-1.6163,-2.3211,2.2754,-1.5251,1.5708,-3.0961]
L_Q99=[-1.6163,-2.3211,2.2754,-1.5251,1.5708,-3.0961]
L_Q100=[-1.6163,-2.3211,2.2754,-1.5251,1.5708,-3.0961]
L_Q101=[-1.6163,-2.3211,2.2754,-1.5251,1.5708,-3.0961]
L_Q102=[-1.6163,-2.3211,2.2754,-1.5251,1.5708,-3.0961]
L_Q103=[-1.6163,-2.3211,2.2754,-1.5251,1.5708,-3.0961]
L_Q104=[-1.6163,-2.3211,2.2754,-1.5251,1.5708,-3.0961]
L_Q105=[-1.6163,-2.3211,2.2754,-1.5251,1.5708,-3.0961]
L_Q106=[-1.6163,-2.3211,2.2754,-1.5251,1.5708,-3.0961]
L_Q107=[-1.6163,-2.3211,2.2754,-1.5251,1.5708,-3.0961]
L_Q108=[-1.6163,-2.3211,2.2754,-1.5251,1.5708,-3.0961]
L_Q109=[-1.6163,-2.3211,2.2754,-1.5251,1.5708,-3.0961]
L_Q110=[-1.6163,-2.3211,2.2754,-1.5251,1.5708,-3.0961]
L_Q111=[-1.6163,-2.3211,2.2754,-1.5251,1.5708,-3.0961]
L_Q112=[-1.6163,-2.3211,2.2754,-1.5251,1.5708,-3.0961]

R_Q1=[0,-1.5708,0,1.5708,0,0]
R_Q2=[0.86094,-1.6655,1.9082,-1.8135,1.5708,0.70986]
R_Q3=[0.86094,-1.6655,1.9082,-1.8135,1.5708,0.70986]
R_Q4=[0.86094,-1.6655,1.9082,-1.8135,1.5708,0.70986]
R_Q5=[0.86094,-1.6655,1.9082,-1.8135,1.5708,0.70986]
R_Q6=[0.86094,-1.6655,1.9082,-1.8135,1.5708,0.70986]
R_Q7=[0.86094,-1.6655,1.9082,-1.8135,1.5708,0.70986]
R_Q8=[0.86094,-1.6655,1.9082,-1.8135,1.5708,0.70986]
R_Q9=[0.86094,-1.6655,1.9082,-1.8135,1.5708,0.70986]
R_Q10=[0.86094,-1.6655,1.9082,-1.8135,1.5708,0.70986]
R_Q11=[0.86094,-1.6655,1.9082,-1.8135,1.5708,0.70986]
R_Q12=[0.86094,-1.6655,1.9082,-1.8135,1.5708,0.70986]
R_Q13=[0.82563,-1.7084,1.9417,-1.8032,1.5708,0.74517]
R_Q14=[0.78741,-1.7557,1.9779,-1.7926,1.5708,0.78338]
R_Q15=[0.7492,-1.803,2.014,-1.7821,1.5708,0.8216]
R_Q16=[0.71099,-1.8503,2.0501,-1.7716,1.5708,0.85981]
R_Q17=[0.67568,-1.8932,2.0837,-1.7613,1.5708,0.89512]
R_Q18=[0.6228,-1.9365,2.1077,-1.7406,1.5708,0.94799]
R_Q19=[0.56276,-1.9843,2.1329,-1.7189,1.5708,1.008]
R_Q20=[0.50271,-2.0322,2.1582,-1.6973,1.5708,1.0681]
R_Q21=[0.44266,-2.08,2.1834,-1.6756,1.5708,1.1281]
R_Q22=[0.38979,-2.1233,2.2074,-1.6549,1.5708,1.181]
R_Q23=[0.4292,-2.1609,2.2208,-1.6298,1.5708,1.1416]
R_Q24=[0.47232,-2.2017,2.2345,-1.6033,1.5708,1.0985]
R_Q25=[0.51543,-2.2426,2.2483,-1.5768,1.5708,1.0554]
R_Q26=[0.55855,-2.2835,2.262,-1.5503,1.5708,1.0122]
R_Q27=[0.59797,-2.3211,2.2754,-1.5251,1.5708,0.97283]
R_Q28=[0.66862,-2.3378,2.2793,-1.5121,1.5708,0.90217]
R_Q29=[0.75367,-2.3552,2.2831,-1.4987,1.5708,0.81713]
R_Q30=[0.83871,-2.3725,2.287,-1.4853,1.5708,0.73209]
R_Q31=[0.92375,-2.3899,2.2909,-1.4719,1.5708,0.64704]
R_Q32=[0.99441,-2.4066,2.2948,-1.4589,1.5708,0.57639]
R_Q33=[1.0842,-2.3899,2.2909,-1.4719,1.5708,0.48658]
R_Q34=[1.2013,-2.3725,2.287,-1.4853,1.5708,0.3695]
R_Q35=[1.3184,-2.3552,2.2831,-1.4987,1.5708,0.25242]
R_Q36=[1.4355,-2.3378,2.2793,-1.5121,1.5708,0.13534]
R_Q37=[1.5253,-2.3211,2.2754,-1.5251,1.5708,0.045534]
R_Q38=[1.6018,-2.2835,2.262,-1.5503,1.5708,-0.030958]
R_Q39=[1.6959,-2.2426,2.2483,-1.5768,1.5708,-0.12507]
R_Q40=[1.79,-2.2017,2.2345,-1.6033,1.5708,-0.21918]
R_Q41=[1.8841,-2.1609,2.2208,-1.6298,1.5708,-0.31329]
R_Q42=[1.9606,-2.1233,2.2074,-1.6549,1.5708,-0.38979]
R_Q43=[1.9606,-2.1233,2.2074,-1.6549,1.5708,-0.38979]
R_Q44=[1.9606,-2.1233,2.2074,-1.6549,1.5708,-0.38979]
R_Q45=[1.9606,-2.1233,2.2074,-1.6549,1.5708,-0.38979]
R_Q46=[1.9606,-2.1233,2.2074,-1.6549,1.5708,-0.38979]
R_Q47=[1.9606,-2.1233,2.2074,-1.6549,1.5708,-0.38979]
R_Q48=[1.9394,-2.08,2.1834,-1.6756,1.5708,-0.36859]
R_Q49=[1.9172,-2.0322,2.1582,-1.6973,1.5708,-0.34642]
R_Q50=[1.895,-1.9843,2.1329,-1.7189,1.5708,-0.32425]
R_Q51=[1.8729,-1.9365,2.1077,-1.7406,1.5708,-0.30208]
R_Q52=[1.8517,-1.8932,2.0837,-1.7613,1.5708,-0.28089]
R_Q53=[1.8792,-1.8288,2.0285,-1.7724,1.5708,-0.30843]
R_Q54=[1.9085,-1.7529,1.9653,-1.7839,1.5708,-0.33769]
R_Q55=[1.9377,-1.677,1.9022,-1.7953,1.5708,-0.36694]
R_Q56=[1.967,-1.6012,1.839,-1.8067,1.5708,-0.39619]
R_Q57=[1.9945,-1.5368,1.7838,-1.8178,1.5708,-0.42374]
R_Q58=[1.9555,-1.5617,1.8079,-1.8169,1.5708,-0.38465]
R_Q59=[1.9127,-1.588,1.8333,-1.8161,1.5708,-0.34194]
R_Q60=[1.87,-1.6143,1.8587,-1.8152,1.5708,-0.29923]
R_Q61=[1.8273,-1.6406,1.8841,-1.8143,1.5708,-0.25652]
R_Q62=[1.7882,-1.6655,1.9082,-1.8135,1.5708,-0.21744]
R_Q63=[1.7432,-1.6829,1.9229,-1.8107,1.5708,-0.17242]
R_Q64=[1.6932,-1.7009,1.9381,-1.808,1.5708,-0.12243]
R_Q65=[1.6432,-1.7189,1.9533,-1.8052,1.5708,-0.072441]
R_Q66=[1.5932,-1.7369,1.9685,-1.8025,1.5708,-0.022452]
R_Q67=[1.5482,-1.7543,1.9832,-1.7997,1.5708,0.022566]
R_Q68=[1.5009,-1.7607,1.9882,-1.7983,1.5708,0.069909]
R_Q69=[1.448,-1.7671,1.9933,-1.7969,1.5708,0.12282]
R_Q70=[1.3951,-1.7736,1.9983,-1.7955,1.5708,0.17573]
R_Q71=[1.3421,-1.7801,2.0033,-1.7941,1.5708,0.22865]
R_Q72=[1.2948,-1.7865,2.0083,-1.7927,1.5708,0.27599]
R_Q73=[1.2504,-1.7801,2.0033,-1.7941,1.5708,0.3204]
R_Q74=[1.2012,-1.7736,1.9983,-1.7955,1.5708,0.36964]
R_Q75=[1.1519,-1.7671,1.9933,-1.7969,1.5708,0.41887]
R_Q76=[1.1027,-1.7607,1.9882,-1.7983,1.5708,0.46811]
R_Q77=[1.0583,-1.7543,1.9832,-1.7997,1.5708,0.51252]
R_Q78=[1.0583,-1.7543,1.9832,-1.7997,1.5708,0.51252]
R_Q79=[1.0583,-1.7543,1.9832,-1.7997,1.5708,0.51252]
R_Q80=[1.0583,-1.7543,1.9832,-1.7997,1.5708,0.51252]
R_Q81=[1.0583,-1.7543,1.9832,-1.7997,1.5708,0.51252]
R_Q82=[1.0583,-1.7543,1.9832,-1.7997,1.5708,0.51252]
R_Q83=[1.0583,-1.7543,1.9832,-1.7997,1.5708,0.51252]
R_Q84=[1.0583,-1.7543,1.9832,-1.7997,1.5708,0.51252]
R_Q85=[1.0583,-1.7543,1.9832,-1.7997,1.5708,0.51252]
R_Q86=[1.0583,-1.7543,1.9832,-1.7997,1.5708,0.51252]
R_Q87=[1.0583,-1.7543,1.9832,-1.7997,1.5708,0.51252]
R_Q88=[1.0583,-1.7543,1.9832,-1.7997,1.5708,0.51252]
R_Q89=[1.0583,-1.7543,1.9832,-1.7997,1.5708,0.51252]
R_Q90=[1.0583,-1.7543,1.9832,-1.7997,1.5708,0.51252]
R_Q91=[1.0583,-1.7543,1.9832,-1.7997,1.5708,0.51252]
R_Q92=[1.0583,-1.7543,1.9832,-1.7997,1.5708,0.51252]
R_Q93=[1.0583,-1.7543,1.9832,-1.7997,1.5708,0.51252]
R_Q94=[1.0583,-1.7543,1.9832,-1.7997,1.5708,0.51252]
R_Q95=[1.0583,-1.7543,1.9832,-1.7997,1.5708,0.51252]
R_Q96=[1.0583,-1.7543,1.9832,-1.7997,1.5708,0.51252]
R_Q97=[1.0583,-1.7543,1.9832,-1.7997,1.5708,0.51252]
R_Q98=[1.0583,-1.7543,1.9832,-1.7997,1.5708,0.51252]
R_Q99=[1.0583,-1.7543,1.9832,-1.7997,1.5708,0.51252]
R_Q100=[1.0583,-1.7543,1.9832,-1.7997,1.5708,0.51252]
R_Q101=[1.0583,-1.7543,1.9832,-1.7997,1.5708,0.51252]
R_Q102=[1.0583,-1.7543,1.9832,-1.7997,1.5708,0.51252]
R_Q103=[1.0583,-1.7543,1.9832,-1.7997,1.5708,0.51252]
R_Q104=[1.0583,-1.7543,1.9832,-1.7997,1.5708,0.51252]
R_Q105=[1.0583,-1.7543,1.9832,-1.7997,1.5708,0.51252]
R_Q106=[1.0583,-1.7543,1.9832,-1.7997,1.5708,0.51252]
R_Q107=[1.0583,-1.7543,1.9832,-1.7997,1.5708,0.51252]
R_Q108=[1.0583,-1.7543,1.9832,-1.7997,1.5708,0.51252]
R_Q109=[1.0583,-1.7543,1.9832,-1.7997,1.5708,0.51252]
R_Q110=[1.0583,-1.7543,1.9832,-1.7997,1.5708,0.51252]
R_Q111=[1.0583,-1.7543,1.9832,-1.7997,1.5708,0.51252]
R_Q112=[1.0583,-1.7543,1.9832,-1.7997,1.5708,0.51252]

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
        left_joint_states = rospy.wait_for_message("/left/joint_states", JointState)
        left_joints_pos = left_joint_states.position
        gl.trajectory.points = [
            JointTrajectoryPoint(positions=left_joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0)),
            JointTrajectoryPoint(positions=L_Q1,velocities=[0,0,0,0,0,0],time_from_start=rospy.Duration(0.05)),
JointTrajectoryPoint(positions=L_Q2,velocities=[0,0,0,0,0,0],time_from_start=rospy.Duration(0.1)),
JointTrajectoryPoint(positions=L_Q3,velocities=[0,0,0,0,0,0],time_from_start=rospy.Duration(0.2)),
JointTrajectoryPoint(positions=L_Q4,velocities=[0,0,0,0,0,0],time_from_start=rospy.Duration(0.3)),
JointTrajectoryPoint(positions=L_Q5,velocities=[0,0,0,0,0,0],time_from_start=rospy.Duration(0.4)),
JointTrajectoryPoint(positions=L_Q6,velocities=[0,0,0,0,0,0],time_from_start=rospy.Duration(0.5)),
JointTrajectoryPoint(positions=L_Q7,velocities=[0,0,0,0,0,0],time_from_start=rospy.Duration(0.6)),
JointTrajectoryPoint(positions=L_Q8,velocities=[0,0,0,0,0,0],time_from_start=rospy.Duration(0.7)),
JointTrajectoryPoint(positions=L_Q9,velocities=[0,0,0,0,0,0],time_from_start=rospy.Duration(0.8)),
JointTrajectoryPoint(positions=L_Q10,velocities=[0,0,0,0,0,0],time_from_start=rospy.Duration(0.9)),
JointTrajectoryPoint(positions=L_Q11,velocities=[0,0,0,0,0,0],time_from_start=rospy.Duration(1)),
JointTrajectoryPoint(positions=L_Q12,velocities=[-0.21193,0.43293,-0.23985,-0.2072,0,0.21193],time_from_start=rospy.Duration(1.1)),
JointTrajectoryPoint(positions=L_Q13,velocities=[-0.22171,0.47848,-0.25254,-0.21652,0,0.22171],time_from_start=rospy.Duration(1.2)),
JointTrajectoryPoint(positions=L_Q14,velocities=[-0.22171,0.47848,-0.25254,-0.21652,0,0.22171],time_from_start=rospy.Duration(1.3)),
JointTrajectoryPoint(positions=L_Q15,velocities=[-0.22171,0.47848,-0.25254,-0.21652,0,0.22171],time_from_start=rospy.Duration(1.4)),
JointTrajectoryPoint(positions=L_Q16,velocities=[-0.21193,0.43293,-0.23985,-0.2072,0,0.21193],time_from_start=rospy.Duration(1.5)),
JointTrajectoryPoint(positions=L_Q17,velocities=[-0.12494,0.42854,-0.33536,-0.10299,0,0.12494],time_from_start=rospy.Duration(1.6)),
JointTrajectoryPoint(positions=L_Q18,velocities=[-0.12821,0.47306,-0.36133,-0.10519,0,0.12821],time_from_start=rospy.Duration(1.7)),
JointTrajectoryPoint(positions=L_Q19,velocities=[-0.12821,0.47306,-0.36133,-0.10519,0,0.12821],time_from_start=rospy.Duration(1.8)),
JointTrajectoryPoint(positions=L_Q20,velocities=[-0.12821,0.47306,-0.36133,-0.10519,0,0.12821],time_from_start=rospy.Duration(1.9)),
JointTrajectoryPoint(positions=L_Q21,velocities=[-0.12494,0.42854,-0.33536,-0.10299,0,0.12494],time_from_start=rospy.Duration(2)),
JointTrajectoryPoint(positions=L_Q22,velocities=[-0.45018,-0.17363,0.14734,0.027316,0,0.45018],time_from_start=rospy.Duration(2.1)),
JointTrajectoryPoint(positions=L_Q23,velocities=[-0.49989,-0.18008,0.15193,0.027466,0,0.49989],time_from_start=rospy.Duration(2.2)),
JointTrajectoryPoint(positions=L_Q24,velocities=[-0.49989,-0.18008,0.15193,0.027466,0,0.49989],time_from_start=rospy.Duration(2.3)),
JointTrajectoryPoint(positions=L_Q25,velocities=[-0.49989,-0.18008,0.15193,0.027466,0,0.49989],time_from_start=rospy.Duration(2.4)),
JointTrajectoryPoint(positions=L_Q26,velocities=[-0.45018,-0.17363,0.14734,0.027316,0,-62.3817],time_from_start=rospy.Duration(2.5)),
JointTrajectoryPoint(positions=L_Q27,velocities=[-0.47343,-0.063844,0.049868,0.014149,0,0.47343],time_from_start=rospy.Duration(2.6)),
JointTrajectoryPoint(positions=L_Q28,velocities=[-0.52913,-0.064677,0.050372,0.014189,0,0.52913],time_from_start=rospy.Duration(2.7)),
JointTrajectoryPoint(positions=L_Q29,velocities=[-0.52913,-0.064677,0.050372,0.014189,0,0.52913],time_from_start=rospy.Duration(2.8)),
JointTrajectoryPoint(positions=L_Q30,velocities=[-0.52913,-0.064677,0.050372,0.014189,0,0.52913],time_from_start=rospy.Duration(2.9)),
JointTrajectoryPoint(positions=L_Q31,velocities=[-0.47343,-0.063844,0.049868,0.014149,0,0.47343],time_from_start=rospy.Duration(3)),
JointTrajectoryPoint(positions=L_Q32,velocities=[-0.44413,0.063844,-0.049868,-0.014149,0,0.44413],time_from_start=rospy.Duration(3.1)),
JointTrajectoryPoint(positions=L_Q33,velocities=[-0.49236,0.064677,-0.050372,-0.014189,0,0.49236],time_from_start=rospy.Duration(3.2)),
JointTrajectoryPoint(positions=L_Q34,velocities=[-0.49236,0.064677,-0.050372,-0.014189,0,0.49236],time_from_start=rospy.Duration(3.3)),
JointTrajectoryPoint(positions=L_Q35,velocities=[-0.49236,0.064677,-0.050372,-0.014189,0,0.49236],time_from_start=rospy.Duration(3.4)),
JointTrajectoryPoint(positions=L_Q36,velocities=[-0.44413,0.063844,-0.049868,-0.014149,0,0.44413],time_from_start=rospy.Duration(3.5)),
JointTrajectoryPoint(positions=L_Q37,velocities=[-0.37481,0.17363,-0.14734,-0.027316,0,0.37481],time_from_start=rospy.Duration(3.6)),
JointTrajectoryPoint(positions=L_Q38,velocities=[-0.40791,0.18008,-0.15193,-0.027466,0,0.40791],time_from_start=rospy.Duration(3.7)),
JointTrajectoryPoint(positions=L_Q39,velocities=[-0.40791,0.18008,-0.15193,-0.027466,0,0.40791],time_from_start=rospy.Duration(3.8)),
JointTrajectoryPoint(positions=L_Q40,velocities=[-0.40791,0.18008,-0.15193,-0.027466,0,0.40791],time_from_start=rospy.Duration(3.9)),
JointTrajectoryPoint(positions=L_Q41,velocities=[-0.37481,0.17363,-0.14734,-0.027316,0,0.37481],time_from_start=rospy.Duration(4)),
JointTrajectoryPoint(positions=L_Q42,velocities=[0,0,0,0,0,0],time_from_start=rospy.Duration(4.1)),
JointTrajectoryPoint(positions=L_Q43,velocities=[0,0,0,0,0,0],time_from_start=rospy.Duration(4.2)),
JointTrajectoryPoint(positions=L_Q44,velocities=[0,0,0,0,0,0],time_from_start=rospy.Duration(4.3)),
JointTrajectoryPoint(positions=L_Q45,velocities=[0,0,0,0,0,0],time_from_start=rospy.Duration(4.4)),
JointTrajectoryPoint(positions=L_Q46,velocities=[0,0,0,0,0,0],time_from_start=rospy.Duration(4.5)),
JointTrajectoryPoint(positions=L_Q47,velocities=[-0.35309,-0.42854,0.33536,0.10299,0,0.35309],time_from_start=rospy.Duration(4.6)),
JointTrajectoryPoint(positions=L_Q48,velocities=[-0.38214,-0.47306,0.36133,0.10519,0,0.38214],time_from_start=rospy.Duration(4.7)),
JointTrajectoryPoint(positions=L_Q49,velocities=[-0.38214,-0.47306,0.36133,0.10519,0,0.38214],time_from_start=rospy.Duration(4.8)),
JointTrajectoryPoint(positions=L_Q50,velocities=[-0.38214,-0.47306,0.36133,0.10519,0,0.38214],time_from_start=rospy.Duration(4.9)),
JointTrajectoryPoint(positions=L_Q51,velocities=[-0.35309,-0.42854,0.33536,0.10299,0,0.35309],time_from_start=rospy.Duration(5)),
JointTrajectoryPoint(positions=L_Q52,velocities=[-0.70402,0,0,0,0,0.70402],time_from_start=rospy.Duration(5.1)),
JointTrajectoryPoint(positions=L_Q53,velocities=[-0.84662,0,0,0,0,0.84662],time_from_start=rospy.Duration(5.2)),
JointTrajectoryPoint(positions=L_Q54,velocities=[-0.84662,0,0,0,0,0.84662],time_from_start=rospy.Duration(5.3)),
JointTrajectoryPoint(positions=L_Q55,velocities=[-0.84662,0,0,0,0,0.84662],time_from_start=rospy.Duration(5.4)),
JointTrajectoryPoint(positions=L_Q56,velocities=[-0.70402,0,0,0,0,0.70402],time_from_start=rospy.Duration(5.5)),
JointTrajectoryPoint(positions=L_Q57,velocities=[0.21193,-0.43293,0.23985,0.2072,0,-0.21193],time_from_start=rospy.Duration(5.6)),
JointTrajectoryPoint(positions=L_Q58,velocities=[0.22171,-0.47848,0.25254,0.21652,0,-0.22171],time_from_start=rospy.Duration(5.7)),
JointTrajectoryPoint(positions=L_Q59,velocities=[0.22171,-0.47848,0.25254,0.21652,0,-0.22171],time_from_start=rospy.Duration(5.8)),
JointTrajectoryPoint(positions=L_Q60,velocities=[0.22171,-0.47848,0.25254,0.21652,0,-0.22171],time_from_start=rospy.Duration(5.9)),
JointTrajectoryPoint(positions=L_Q61,velocities=[0.21193,-0.43293,0.23985,0.2072,0,-0.21193],time_from_start=rospy.Duration(6)),
JointTrajectoryPoint(positions=L_Q62,velocities=[0.39417,-0.37559,0.13376,0.25113,0,-0.39417],time_from_start=rospy.Duration(6.1)),
JointTrajectoryPoint(positions=L_Q63,velocities=[0.43116,-0.40884,0.13752,0.26512,0,-0.43116],time_from_start=rospy.Duration(6.2)),
JointTrajectoryPoint(positions=L_Q64,velocities=[0.43116,-0.40884,0.13752,0.26512,0,-0.43116],time_from_start=rospy.Duration(6.3)),
JointTrajectoryPoint(positions=L_Q65,velocities=[0.43116,-0.40884,0.13752,0.26512,0,-0.43116],time_from_start=rospy.Duration(6.4)),
JointTrajectoryPoint(positions=L_Q66,velocities=[0.39417,-0.37559,0.13376,0.25113,0,-0.39417],time_from_start=rospy.Duration(6.5)),
JointTrajectoryPoint(positions=L_Q67,velocities=[0.70655,-0.16754,0.038507,0.13031,0,-0.70655],time_from_start=rospy.Duration(6.6)),
JointTrajectoryPoint(positions=L_Q68,velocities=[0.85043,-0.17353,0.038807,0.13387,0,-0.85043],time_from_start=rospy.Duration(6.7)),
JointTrajectoryPoint(positions=L_Q69,velocities=[0.85043,-0.17353,0.038807,0.13387,0,-0.85043],time_from_start=rospy.Duration(6.8)),
JointTrajectoryPoint(positions=L_Q70,velocities=[0.85043,-0.17353,0.038807,0.13387,0,-0.85043],time_from_start=rospy.Duration(6.9)),
JointTrajectoryPoint(positions=L_Q71,velocities=[0.70655,-0.16754,0.038507,0.13031,0,-0.70655],time_from_start=rospy.Duration(7)),
JointTrajectoryPoint(positions=L_Q72,velocities=[0.89809,0.16754,-0.038507,-0.13031,0,-0.89809],time_from_start=rospy.Duration(7.1)),
JointTrajectoryPoint(positions=L_Q73,velocities=[1.1708,0.17353,-0.038807,-0.13387,0,-1.1708],time_from_start=rospy.Duration(7.2)),
JointTrajectoryPoint(positions=L_Q74,velocities=[1.1708,0.17353,-0.038807,-0.13387,0,-1.1708],time_from_start=rospy.Duration(7.3)),
JointTrajectoryPoint(positions=L_Q75,velocities=[1.1708,0.17353,-0.038807,-0.13387,0,-1.1708],time_from_start=rospy.Duration(7.4)),
JointTrajectoryPoint(positions=L_Q76,velocities=[0.89809,0.16754,-0.038507,-0.13031,0,-0.89809],time_from_start=rospy.Duration(7.5)),
JointTrajectoryPoint(positions=L_Q77,velocities=[0,0,0,0,0,0],time_from_start=rospy.Duration(7.6)),
JointTrajectoryPoint(positions=L_Q78,velocities=[0,0,0,0,0,0],time_from_start=rospy.Duration(7.7)),
JointTrajectoryPoint(positions=L_Q79,velocities=[0,0,0,0,0,0],time_from_start=rospy.Duration(7.8)),
JointTrajectoryPoint(positions=L_Q80,velocities=[0,0,0,0,0,0],time_from_start=rospy.Duration(7.9)),
JointTrajectoryPoint(positions=L_Q81,velocities=[0,0,0,0,0,0],time_from_start=rospy.Duration(8)),
JointTrajectoryPoint(positions=L_Q82,velocities=[0,0,0,0,0,0],time_from_start=rospy.Duration(8.1)),
JointTrajectoryPoint(positions=L_Q83,velocities=[0,0,0,0,0,0],time_from_start=rospy.Duration(8.2)),
JointTrajectoryPoint(positions=L_Q84,velocities=[0,0,0,0,0,0],time_from_start=rospy.Duration(8.3)),
JointTrajectoryPoint(positions=L_Q85,velocities=[0,0,0,0,0,0],time_from_start=rospy.Duration(8.4)),
JointTrajectoryPoint(positions=L_Q86,velocities=[0,0,0,0,0,0],time_from_start=rospy.Duration(8.5)),
JointTrajectoryPoint(positions=L_Q87,velocities=[0,0,0,0,0,0],time_from_start=rospy.Duration(8.6)),
JointTrajectoryPoint(positions=L_Q88,velocities=[0,0,0,0,0,0],time_from_start=rospy.Duration(8.7)),
JointTrajectoryPoint(positions=L_Q89,velocities=[0,0,0,0,0,0],time_from_start=rospy.Duration(8.8)),
JointTrajectoryPoint(positions=L_Q90,velocities=[0,0,0,0,0,0],time_from_start=rospy.Duration(8.9)),
JointTrajectoryPoint(positions=L_Q91,velocities=[0,0,0,0,0,0],time_from_start=rospy.Duration(9)),
JointTrajectoryPoint(positions=L_Q92,velocities=[0,0,0,0,0,0],time_from_start=rospy.Duration(9.1)),
JointTrajectoryPoint(positions=L_Q93,velocities=[0,0,0,0,0,0],time_from_start=rospy.Duration(9.2)),
JointTrajectoryPoint(positions=L_Q94,velocities=[0,0,0,0,0,0],time_from_start=rospy.Duration(9.3)),
JointTrajectoryPoint(positions=L_Q95,velocities=[0,0,0,0,0,0],time_from_start=rospy.Duration(9.4)),
JointTrajectoryPoint(positions=L_Q96,velocities=[0,0,0,0,0,0],time_from_start=rospy.Duration(9.5)),
JointTrajectoryPoint(positions=L_Q97,velocities=[0,0,0,0,0,0],time_from_start=rospy.Duration(9.6)),
JointTrajectoryPoint(positions=L_Q98,velocities=[0,0,0,0,0,0],time_from_start=rospy.Duration(9.7)),
JointTrajectoryPoint(positions=L_Q99,velocities=[0,0,0,0,0,0],time_from_start=rospy.Duration(9.8)),
JointTrajectoryPoint(positions=L_Q100,velocities=[0,0,0,0,0,0],time_from_start=rospy.Duration(9.9)),
JointTrajectoryPoint(positions=L_Q101,velocities=[0,0,0,0,0,0],time_from_start=rospy.Duration(10)),
JointTrajectoryPoint(positions=L_Q102,velocities=[0,0,0,0,0,0],time_from_start=rospy.Duration(10.1)),
JointTrajectoryPoint(positions=L_Q103,velocities=[0,0,0,0,0,0],time_from_start=rospy.Duration(10.2)),
JointTrajectoryPoint(positions=L_Q104,velocities=[0,0,0,0,0,0],time_from_start=rospy.Duration(10.3)),
JointTrajectoryPoint(positions=L_Q105,velocities=[0,0,0,0,0,0],time_from_start=rospy.Duration(10.4)),
JointTrajectoryPoint(positions=L_Q106,velocities=[0,0,0,0,0,0],time_from_start=rospy.Duration(10.5)),
JointTrajectoryPoint(positions=L_Q107,velocities=[0,0,0,0,0,0],time_from_start=rospy.Duration(10.6)),
JointTrajectoryPoint(positions=L_Q108,velocities=[0,0,0,0,0,0],time_from_start=rospy.Duration(10.7)),
JointTrajectoryPoint(positions=L_Q109,velocities=[0,0,0,0,0,0],time_from_start=rospy.Duration(10.8)),
JointTrajectoryPoint(positions=L_Q110,velocities=[0,0,0,0,0,0],time_from_start=rospy.Duration(10.9)),
JointTrajectoryPoint(positions=L_Q111,velocities=[0,0,0,0,0,0],time_from_start=rospy.Duration(11)),
JointTrajectoryPoint(positions=L_Q112,velocities=[0,0,0,0,0,0],time_from_start=rospy.Duration(11.1))]

        right_joint_states = rospy.wait_for_message("/right/joint_states", JointState)
        right_joints_pos = right_joint_states.position
        gr.trajectory.points = [
            JointTrajectoryPoint(positions=right_joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0)),
            JointTrajectoryPoint(positions=R_Q1,velocities=[0,0,0,0,0,0,],time_from_start=rospy.Duration(0.05)),
JointTrajectoryPoint(positions=R_Q2,velocities=[0,0,0,0,0,0,],time_from_start=rospy.Duration(0.1)),
JointTrajectoryPoint(positions=R_Q3,velocities=[0,0,0,0,0,0,],time_from_start=rospy.Duration(0.2)),
JointTrajectoryPoint(positions=R_Q4,velocities=[0,0,0,0,0,0,],time_from_start=rospy.Duration(0.3)),
JointTrajectoryPoint(positions=R_Q5,velocities=[0,0,0,0,0,0,],time_from_start=rospy.Duration(0.4)),
JointTrajectoryPoint(positions=R_Q6,velocities=[0,0,0,0,0,0,],time_from_start=rospy.Duration(0.5)),
JointTrajectoryPoint(positions=R_Q7,velocities=[0,0,0,0,0,0,],time_from_start=rospy.Duration(0.6)),
JointTrajectoryPoint(positions=R_Q8,velocities=[0,0,0,0,0,0,],time_from_start=rospy.Duration(0.7)),
JointTrajectoryPoint(positions=R_Q9,velocities=[0,0,0,0,0,0,],time_from_start=rospy.Duration(0.8)),
JointTrajectoryPoint(positions=R_Q10,velocities=[0,0,0,0,0,0,],time_from_start=rospy.Duration(0.9)),
JointTrajectoryPoint(positions=R_Q11,velocities=[0,0,0,0,0,0,],time_from_start=rospy.Duration(1)),
JointTrajectoryPoint(positions=R_Q12,velocities=[-0.35309,-0.42854,0.33536,0.10299,0,0.35309,],time_from_start=rospy.Duration(1.1)),
JointTrajectoryPoint(positions=R_Q13,velocities=[-0.38214,-0.47306,0.36133,0.10519,0,0.38214,],time_from_start=rospy.Duration(1.2)),
JointTrajectoryPoint(positions=R_Q14,velocities=[-0.38214,-0.47306,0.36133,0.10519,0,0.38214,],time_from_start=rospy.Duration(1.3)),
JointTrajectoryPoint(positions=R_Q15,velocities=[-0.38214,-0.47306,0.36133,0.10519,0,0.38214,],time_from_start=rospy.Duration(1.4)),
JointTrajectoryPoint(positions=R_Q16,velocities=[-0.35309,-0.42854,0.33536,0.10299,0,0.35309,],time_from_start=rospy.Duration(1.5)),
JointTrajectoryPoint(positions=R_Q17,velocities=[-0.52874,-0.43293,0.23985,0.2072,0,0.52874,],time_from_start=rospy.Duration(1.6)),
JointTrajectoryPoint(positions=R_Q18,velocities=[-0.60048,-0.47848,0.25254,0.21652,0,0.60048,],time_from_start=rospy.Duration(1.7)),
JointTrajectoryPoint(positions=R_Q19,velocities=[-0.60048,-0.47848,0.25254,0.21652,0,0.60048,],time_from_start=rospy.Duration(1.8)),
JointTrajectoryPoint(positions=R_Q20,velocities=[-0.60048,-0.47848,0.25254,0.21652,0,0.60048,],time_from_start=rospy.Duration(1.9)),
JointTrajectoryPoint(positions=R_Q21,velocities=[-0.52874,-0.43293,0.23985,0.2072,0,0.52874,],time_from_start=rospy.Duration(2)),
JointTrajectoryPoint(positions=R_Q22,velocities=[0.39417,-0.37559,0.13376,0.25113,0,-0.39417,],time_from_start=rospy.Duration(2.1)),
JointTrajectoryPoint(positions=R_Q23,velocities=[0.43116,-0.40884,0.13752,0.26512,0,-0.43116,],time_from_start=rospy.Duration(2.2)),
JointTrajectoryPoint(positions=R_Q24,velocities=[0.43116,-0.40884,0.13752,0.26512,0,-0.43116,],time_from_start=rospy.Duration(2.3)),
JointTrajectoryPoint(positions=R_Q25,velocities=[0.43116,-0.40884,0.13752,0.26512,0,-0.43116,],time_from_start=rospy.Duration(2.4)),
JointTrajectoryPoint(positions=R_Q26,velocities=[0.39417,-0.37559,0.13376,0.25113,0,-0.39417,],time_from_start=rospy.Duration(2.5)),
JointTrajectoryPoint(positions=R_Q27,velocities=[0.70655,-0.16754,0.038507,0.13031,0,-0.70655,],time_from_start=rospy.Duration(2.6)),
JointTrajectoryPoint(positions=R_Q28,velocities=[0.85043,-0.17353,0.038807,0.13387,0,-0.85043,],time_from_start=rospy.Duration(2.7)),
JointTrajectoryPoint(positions=R_Q29,velocities=[0.85043,-0.17353,0.038807,0.13387,0,-0.85043,],time_from_start=rospy.Duration(2.8)),
JointTrajectoryPoint(positions=R_Q30,velocities=[0.85043,-0.17353,0.038807,0.13387,0,-0.85043,],time_from_start=rospy.Duration(2.9)),
JointTrajectoryPoint(positions=R_Q31,velocities=[0.70655,-0.16754,0.038507,0.13031,0,-0.70655,],time_from_start=rospy.Duration(3)),
JointTrajectoryPoint(positions=R_Q32,velocities=[0.89809,0.16754,-0.038507,-0.13031,0,-0.89809,],time_from_start=rospy.Duration(3.1)),
JointTrajectoryPoint(positions=R_Q33,velocities=[1.1708,0.17353,-0.038807,-0.13387,0,-1.1708,],time_from_start=rospy.Duration(3.2)),
JointTrajectoryPoint(positions=R_Q34,velocities=[1.1708,0.17353,-0.038807,-0.13387,0,-1.1708,],time_from_start=rospy.Duration(3.3)),
JointTrajectoryPoint(positions=R_Q35,velocities=[1.1708,0.17353,-0.038807,-0.13387,0,-1.1708,],time_from_start=rospy.Duration(3.4)),
JointTrajectoryPoint(positions=R_Q36,velocities=[0.89809,0.16754,-0.038507,-0.13031,0,-0.89809,],time_from_start=rospy.Duration(3.5)),
JointTrajectoryPoint(positions=R_Q37,velocities=[0.76492,0.37559,-0.13376,-0.25113,0,-0.76492,],time_from_start=rospy.Duration(3.6)),
JointTrajectoryPoint(positions=R_Q38,velocities=[0.94112,0.40884,-0.13752,-0.26512,0,-0.94112,],time_from_start=rospy.Duration(3.7)),
JointTrajectoryPoint(positions=R_Q39,velocities=[0.94112,0.40884,-0.13752,-0.26512,0,-0.94112,],time_from_start=rospy.Duration(3.8)),
JointTrajectoryPoint(positions=R_Q40,velocities=[0.94112,0.40884,-0.13752,-0.26512,0,-0.94112,],time_from_start=rospy.Duration(3.9)),
JointTrajectoryPoint(positions=R_Q41,velocities=[0.76492,0.37559,-0.13376,-0.25113,0,-0.76492,],time_from_start=rospy.Duration(4)),
JointTrajectoryPoint(positions=R_Q42,velocities=[0,0,0,0,0,0,],time_from_start=rospy.Duration(4.1)),
JointTrajectoryPoint(positions=R_Q43,velocities=[0,0,0,0,0,0,],time_from_start=rospy.Duration(4.2)),
JointTrajectoryPoint(positions=R_Q44,velocities=[0,0,0,0,0,0,],time_from_start=rospy.Duration(4.3)),
JointTrajectoryPoint(positions=R_Q45,velocities=[0,0,0,0,0,0,],time_from_start=rospy.Duration(4.4)),
JointTrajectoryPoint(positions=R_Q46,velocities=[0,0,0,0,0,0,],time_from_start=rospy.Duration(4.5)),
JointTrajectoryPoint(positions=R_Q47,velocities=[-0.21193,0.43293,-0.23985,-0.2072,0,0.21193,],time_from_start=rospy.Duration(4.6)),
JointTrajectoryPoint(positions=R_Q48,velocities=[-0.22171,0.47848,-0.25254,-0.21652,0,0.22171,],time_from_start=rospy.Duration(4.7)),
JointTrajectoryPoint(positions=R_Q49,velocities=[-0.22171,0.47848,-0.25254,-0.21652,0,0.22171,],time_from_start=rospy.Duration(4.8)),
JointTrajectoryPoint(positions=R_Q50,velocities=[-0.22171,0.47848,-0.25254,-0.21652,0,0.22171,],time_from_start=rospy.Duration(4.9)),
JointTrajectoryPoint(positions=R_Q51,velocities=[-0.21193,0.43293,-0.23985,-0.2072,0,0.21193,],time_from_start=rospy.Duration(5)),
JointTrajectoryPoint(positions=R_Q52,velocities=[0.27549,0.64409,-0.55215,-0.11148,0,-0.27549,],time_from_start=rospy.Duration(5.1)),
JointTrajectoryPoint(positions=R_Q53,velocities=[0.29251,0.75857,-0.63148,-0.11407,0,-0.29251,],time_from_start=rospy.Duration(5.2)),
JointTrajectoryPoint(positions=R_Q54,velocities=[0.29251,0.75857,-0.63148,-0.11407,0,-0.29251,],time_from_start=rospy.Duration(5.3)),
JointTrajectoryPoint(positions=R_Q55,velocities=[0.29251,0.75857,-0.63148,-0.11407,0,-0.29251,],time_from_start=rospy.Duration(5.4)),
JointTrajectoryPoint(positions=R_Q56,velocities=[0.27549,0.64409,-0.55215,-0.11148,0,-0.27549,],time_from_start=rospy.Duration(5.5)),
JointTrajectoryPoint(positions=R_Q57,velocities=[-0.39082,-0.24927,0.24111,0.0087119,0,0.39082,],time_from_start=rospy.Duration(5.6)),
JointTrajectoryPoint(positions=R_Q58,velocities=[-0.42712,-0.26304,0.25394,0.008727,0,0.42712,],time_from_start=rospy.Duration(5.7)),
JointTrajectoryPoint(positions=R_Q59,velocities=[-0.42712,-0.26304,0.25394,0.008727,0,0.42712,],time_from_start=rospy.Duration(5.8)),
JointTrajectoryPoint(positions=R_Q60,velocities=[-0.42712,-0.26304,0.25394,0.008727,0,0.42712,],time_from_start=rospy.Duration(5.9)),
JointTrajectoryPoint(positions=R_Q61,velocities=[-0.39082,-0.24927,0.24111,0.0087119,0,0.39082,],time_from_start=rospy.Duration(6)),
JointTrajectoryPoint(positions=R_Q62,velocities=[-0.45018,-0.17363,0.14734,0.027316,0,0.45018,],time_from_start=rospy.Duration(6.1)),
JointTrajectoryPoint(positions=R_Q63,velocities=[-0.49989,-0.18008,0.15193,0.027466,0,0.49989,],time_from_start=rospy.Duration(6.2)),
JointTrajectoryPoint(positions=R_Q64,velocities=[-0.49989,-0.18008,0.15193,0.027466,0,0.49989,],time_from_start=rospy.Duration(6.3)),
JointTrajectoryPoint(positions=R_Q65,velocities=[-0.49989,-0.18008,0.15193,0.027466,0,0.49989,],time_from_start=rospy.Duration(6.4)),
JointTrajectoryPoint(positions=R_Q66,velocities=[-0.45018,-0.17363,0.14734,0.027316,0,0.45018,],time_from_start=rospy.Duration(6.5)),
JointTrajectoryPoint(positions=R_Q67,velocities=[-0.47343,-0.063844,0.049868,0.014149,0,0.47343,],time_from_start=rospy.Duration(6.6)),
JointTrajectoryPoint(positions=R_Q68,velocities=[-0.52913,-0.064677,0.050372,0.014189,0,0.52913,],time_from_start=rospy.Duration(6.7)),
JointTrajectoryPoint(positions=R_Q69,velocities=[-0.52913,-0.064677,0.050372,0.014189,0,0.52913,],time_from_start=rospy.Duration(6.8)),
JointTrajectoryPoint(positions=R_Q70,velocities=[-0.52913,-0.064677,0.050372,0.014189,0,0.52913,],time_from_start=rospy.Duration(6.9)),
JointTrajectoryPoint(positions=R_Q71,velocities=[-0.47343,-0.063844,0.049868,0.014149,0,0.47343,],time_from_start=rospy.Duration(7)),
JointTrajectoryPoint(positions=R_Q72,velocities=[-0.44413,0.063844,-0.049868,-0.014149,0,0.44413,],time_from_start=rospy.Duration(7.1)),
JointTrajectoryPoint(positions=R_Q73,velocities=[-0.49236,0.064677,-0.050372,-0.014189,0,0.49236,],time_from_start=rospy.Duration(7.2)),
JointTrajectoryPoint(positions=R_Q74,velocities=[-0.49236,0.064677,-0.050372,-0.014189,0,0.49236,],time_from_start=rospy.Duration(7.3)),
JointTrajectoryPoint(positions=R_Q75,velocities=[-0.49236,0.064677,-0.050372,-0.014189,0,0.49236,],time_from_start=rospy.Duration(7.4)),
JointTrajectoryPoint(positions=R_Q76,velocities=[-0.44413,0.063844,-0.049868,-0.014149,0,0.44413,],time_from_start=rospy.Duration(7.5)),
JointTrajectoryPoint(positions=R_Q77,velocities=[0,0,0,0,0,0,],time_from_start=rospy.Duration(7.6)),
JointTrajectoryPoint(positions=R_Q78,velocities=[0,0,0,0,0,0,],time_from_start=rospy.Duration(7.7)),
JointTrajectoryPoint(positions=R_Q79,velocities=[0,0,0,0,0,0,],time_from_start=rospy.Duration(7.8)),
JointTrajectoryPoint(positions=R_Q80,velocities=[0,0,0,0,0,0,],time_from_start=rospy.Duration(7.9)),
JointTrajectoryPoint(positions=R_Q81,velocities=[0,0,0,0,0,0,],time_from_start=rospy.Duration(8)),
JointTrajectoryPoint(positions=R_Q82,velocities=[0,0,0,0,0,0,],time_from_start=rospy.Duration(8.1)),
JointTrajectoryPoint(positions=R_Q83,velocities=[0,0,0,0,0,0,],time_from_start=rospy.Duration(8.2)),
JointTrajectoryPoint(positions=R_Q84,velocities=[0,0,0,0,0,0,],time_from_start=rospy.Duration(8.3)),
JointTrajectoryPoint(positions=R_Q85,velocities=[0,0,0,0,0,0,],time_from_start=rospy.Duration(8.4)),
JointTrajectoryPoint(positions=R_Q86,velocities=[0,0,0,0,0,0,],time_from_start=rospy.Duration(8.5)),
JointTrajectoryPoint(positions=R_Q87,velocities=[0,0,0,0,0,0,],time_from_start=rospy.Duration(8.6)),
JointTrajectoryPoint(positions=R_Q88,velocities=[0,0,0,0,0,0,],time_from_start=rospy.Duration(8.7)),
JointTrajectoryPoint(positions=R_Q89,velocities=[0,0,0,0,0,0,],time_from_start=rospy.Duration(8.8)),
JointTrajectoryPoint(positions=R_Q90,velocities=[0,0,0,0,0,0,],time_from_start=rospy.Duration(8.9)),
JointTrajectoryPoint(positions=R_Q91,velocities=[0,0,0,0,0,0,],time_from_start=rospy.Duration(9)),
JointTrajectoryPoint(positions=R_Q92,velocities=[0,0,0,0,0,0,],time_from_start=rospy.Duration(9.1)),
JointTrajectoryPoint(positions=R_Q93,velocities=[0,0,0,0,0,0,],time_from_start=rospy.Duration(9.2)),
JointTrajectoryPoint(positions=R_Q94,velocities=[0,0,0,0,0,0,],time_from_start=rospy.Duration(9.3)),
JointTrajectoryPoint(positions=R_Q95,velocities=[0,0,0,0,0,0,],time_from_start=rospy.Duration(9.4)),
JointTrajectoryPoint(positions=R_Q96,velocities=[0,0,0,0,0,0,],time_from_start=rospy.Duration(9.5)),
JointTrajectoryPoint(positions=R_Q97,velocities=[0,0,0,0,0,0,],time_from_start=rospy.Duration(9.6)),
JointTrajectoryPoint(positions=R_Q98,velocities=[0,0,0,0,0,0,],time_from_start=rospy.Duration(9.7)),
JointTrajectoryPoint(positions=R_Q99,velocities=[0,0,0,0,0,0,],time_from_start=rospy.Duration(9.8)),
JointTrajectoryPoint(positions=R_Q100,velocities=[0,0,0,0,0,0,],time_from_start=rospy.Duration(9.9)),
JointTrajectoryPoint(positions=R_Q101,velocities=[0,0,0,0,0,0,],time_from_start=rospy.Duration(10)),
JointTrajectoryPoint(positions=R_Q102,velocities=[0,0,0,0,0,0,],time_from_start=rospy.Duration(10.1)),
JointTrajectoryPoint(positions=R_Q103,velocities=[0,0,0,0,0,0,],time_from_start=rospy.Duration(10.2)),
JointTrajectoryPoint(positions=R_Q104,velocities=[0,0,0,0,0,0,],time_from_start=rospy.Duration(10.3)),
JointTrajectoryPoint(positions=R_Q105,velocities=[0,0,0,0,0,0,],time_from_start=rospy.Duration(10.4)),
JointTrajectoryPoint(positions=R_Q106,velocities=[0,0,0,0,0,0,],time_from_start=rospy.Duration(10.5)),
JointTrajectoryPoint(positions=R_Q107,velocities=[0,0,0,0,0,0,],time_from_start=rospy.Duration(10.6)),
JointTrajectoryPoint(positions=R_Q108,velocities=[0,0,0,0,0,0,],time_from_start=rospy.Duration(10.7)),
JointTrajectoryPoint(positions=R_Q109,velocities=[0,0,0,0,0,0,],time_from_start=rospy.Duration(10.8)),
JointTrajectoryPoint(positions=R_Q110,velocities=[0,0,0,0,0,0,],time_from_start=rospy.Duration(10.9)),
JointTrajectoryPoint(positions=R_Q111,velocities=[0,0,0,0,0,0,],time_from_start=rospy.Duration(11)),
JointTrajectoryPoint(positions=R_Q112,velocities=[0,0,0,0,0,0,],time_from_start=rospy.Duration(11.1))]


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
