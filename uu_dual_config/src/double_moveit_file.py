#!/usr/bin/env python
# encoding: utf-8 
import rospy, sys
import moveit_commander
import threading
import multiprocessing 
#from control_msgs.msg import GripperCommand
class MoveItDemo:
 def __init__(self):
 # Initialize the move_group API
  moveit_commander.roscpp_initialize(sys.argv)

 # Initialize the ROS node
  rospy.init_node('moveit_demo', anonymous=True)

  dual_arms = moveit_commander.MoveGroupCommander('dual_arms')
  
  # 零位时机器人底座指向左侧．左为机器人i，右为j
  f = open("/home/petori/catkin_ws/src/universal_robot/uu_dual_config/src/simpleCarry.txt")
  #w = open("/home/yangyifan/桌面/e.txt","w")
  for line in f:
      a = line
      tmp = a.strip()
      test = tmp[1:-1].split(",")
      b = test[0:14]
      print b
      joint_positions = map(lambda x : float(x), b)
      dual_arms.set_joint_value_target(joint_positions)

      dual_traj = dual_arms.plan()

      dual_arms.execute(dual_traj)
      #dual_arms.go()

      rospy.sleep(1)
  f.close()


  moveit_commander.roscpp_shutdown()

 # Exit the script
  moveit_commander.os._exit(0)
 
if __name__ == "__main__":
  MoveItDemo()
