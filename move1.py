#has Baxter move 1 block from a known location to a programmed location

import rospy
import baxter_interface

#intialize ros node
rospy.init_node('move1')

#create a limb object
limb = baxter_interface.Limb('right')

#create gripper object
gripper = baxter_interface.Gripper('right')

#create waypoints
way1 = {'right_s0': 0.32368723306117647, 'right_s1': 0.07952651071899713, 'right_w0': -1.4982426805673166, 'right_w1': 1.2299213013496175, 'right_w2': -2.50106116255417, 'right_e0': 1.2238932505083666, 'right_e1': 1.0985515565377113}
way2 = {'right_s0': 0.19582845510236946, 'right_s1': 0.022654800762392596, 'right_w0': -1.5639842981050212, 'right_w1': 1.1814892516743656, 'right_w2': -2.265499979744934, 'right_e0': 1.174202850039528, 'right_e1': 1.4599695008099192}


#move to first waypoint
limb.move_to_joint_positions(way1)
rospy.sleep(2.0)

#close gripper
gripper.command_position(0)
rospy.sleep(2.0)

#move to second waypoint
limb.move_to_joint_positions(way2)
rospy.sleep(2.0)

#open gripper
gripper.command_position(100)
rospy.sleep(2.0)

#return to first waypoint
limb.move_to_joint_positions(way1)

