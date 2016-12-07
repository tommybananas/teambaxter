#calibrates gripper

import rospy
import baxter_interface

#intialize ros node
rospy.init_node('calibrate_gripper')
#create gripper object
gripper = baxter_interface.Gripper('right')
#calibrate the gripper
gripper.calibrate()
