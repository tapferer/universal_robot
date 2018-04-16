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

 # Set three basic gripper openings
 #GRIPPER_OPEN = [0.04]
 #GRIPPER_CLOSED = [-0.03]
 #GRIPPER_NEUTRAL = [0.01]

 # Connect to the right_arm move group
  dual_arms = moveit_commander.MoveGroupCommander('dual_arms')
  #left_arm = moveit_commander.MoveGroupCommander('left_arm')
 # Connect to the right_gripper move group
 #right_gripper = moveit_commander.MoveGroupCommander('right_gripper')

 # Get the name of the end-effector link
  #right_end_effector_link = right_arm.get_end_effector_link()
  #left_end_effector_link = left_arm.get_end_effector_link()
 # Display the name of the end_effector link
  #rospy.loginfo("The right end effector link is: " + str(right_end_effector_link))
  #rospy.loginfo("The left end effector link is: " + str(left_end_effector_link))
 # Start the arm in the "resting" configuration stored in the SRDF file
  #right_arm.set_named_target("up")
  #left_arm.set_named_target("up")
 # Plan a trajectory to the goal configuration
  #right_traj = right_arm.plan()
  #left_traj = left_arm.plan()
 # Execute the planned trajectory
  #right_arm.execute(right_traj)
  #left_arm.execute(left_traj)
 # Pause for a moment
  #rospy.sleep(3)
  
 # Set the gripper to a neural position using a joint value target
 #right_gripper.set_joint_value_target(GRIPPER_NEUTRAL)

 # Plan and execute a trajectory to the goal configuration
 #right_gripper.go()
 #rospy.sleep(1)

  f = open("/home/petori/catkin_ws/src/universal_robot/uu_dual_config/src/d.txt")
  #w = open("/home/yangyifan/桌面/e.txt","w")
  for line in f:
      a = line
      tmp = a.strip()
      test = tmp[1:-1].split(",")
      b = test[0:14]
      print b
      joint_positions = map(lambda x : float(x), b)
      dual_arms.set_joint_value_target(joint_positions)
      #left_arm.go()
      #right_arm.go()
      #t1 = threading.Thread(target = left_arm.go, args=())
      #t2 = threading.Thread(target = right_arm.go, args=())
      dual_traj = dual_arms.plan()
      #print left_traj
      #print right_traj
      #w.write(str(left_traj))
      #w.write(str(right_traj))
      #print left_traj 
      #p1 = multiprocessing.Process(target=left_arm.execute(left_traj))  
      #p2 = multiprocessing.Process(target=right_arm.execute(right_traj))
      #p1.start()
      #p2.start()
      #p1.join()
      #p2.join()  
      dual_arms.execute(dual_traj)
      #threads = []
      #t2 = threading.Thread(target = right_arm.execute, args=(right_traj,))
      #threads.append(t2)
      #t1 = threading.Thread(target = left_arm.execute, args=(left_traj,))
      #threads.append(t1)
      #for t in threads:
      #    t.start()
      #for t in threads:
      #    t.join()
      #t1.start()
      #t2.start()
      #t1.join()
      #t2.join()
      rospy.sleep(1)
  f.close()


 # Set target joint values for the arm: joints are in the order they
 # appear in the kinematic tree.
  #right_joint_positions1 = [0.5,-0.6,0.7,-0.8,0.9,1.0]
  #left_joint_positions1 = [0, -1, -0.5, -1.57, 0, 0]

 # Set the arm's goal configuration to the be the joint positions
  #right_arm.set_joint_value_target(right_joint_positions1)
  #left_arm.set_joint_value_target(left_joint_positions1)

 # Plan a trajectory to the goal configuration
  #right_traj = right_arm.plan()
  #left_traj = left_arm.plan()
 # Execute the planned trajectory
  #threading.Thread(target = right_arm.execute(right_traj)).start()
  #threading.Thread(target = left_arm.execute(left_traj)).start()
 # Plan and execute a trajectory to the goal configuration
  #right_arm.go()
  #left_arm.go()
 # Pause for a moment
  #rospy.sleep(1)

  #right_joint_positions2 = [0, -1, -0.5, -1.57, 0, 0]
  #left_joint_positions2 = [0.5,-0.6,0.7,-0.8,0.9,1.0]

 # Set the arm's goal configuration to the be the joint positions
  #right_arm.set_joint_value_target(right_joint_positions2)
  #left_arm.set_joint_value_target(left_joint_positions2)

 # Plan a trajectory to the goal configuration
  #right_traj = right_arm.plan()
  #left_traj = left_arm.plan()
 # Execute the planned trajectory
  #threading.Thread(target = right_arm.execute(right_traj)).start()
  #threading.Thread(target = left_arm.execute(left_traj)).start()
 # Plan and execute a trajectory to the goal configuration
  #right_arm.go()
  #left_arm.go()
 # Pause for a moment
  #rospy.sleep(3)
 # Cleanly shut down MoveIt!
  moveit_commander.roscpp_shutdown()

 # Exit the script
  moveit_commander.os._exit(0)
 
if __name__ == "__main__":
  MoveItDemo()
