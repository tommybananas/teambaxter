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
#block location
way1 = {'right_s0': 0.34106751246938366, 'right_s1': 0.09189324383731295, 'right_w0': -1.4734364796964792, 'right_w1': 1.1861410968339896, 'right_w2': -2.5200054785482053, 'right_e0': 1.1770541491784299, 'right_e1': 1.0639893032381078}
#above initial placement
way2 = {'right_s0': 0.3008771824385669, 'right_s1': -0.010549367776385592, 'right_w0': -1.4393283758072442, 'right_w1': 1.2704429640810644, 'right_w2': -2.4803883608609025, 'right_e0': 1.2336359311247687, 'right_e1': 1.1656744428068397}
#above final position
way3 = {'right_s0': 0.19046033346915456, 'right_s1': -0.062027418813302794, 'right_w0': -1.4844356152264875, 'right_w1': 1.2233629588904844, 'right_w2': -2.2840524450160666, 'right_e0': 1.2077157917125343, 'right_e1': 1.4771195322623298}
#final block placement
way4 = {'right_s0': 0.22597342259000514, 'right_s1': 0.03767901640227857, 'right_w0': -1.543675480308317, 'right_w1': 1.1386580792280978, 'right_w2': -2.3042438553533686, 'right_e0': 1.1314508338686378, 'right_e1': 1.391527843540645}

#move to first waypoint
limb.move_to_joint_positions(way1)
rospy.sleep(2.0)

#close gripper
gripper.command_position(0)
rospy.sleep(2.0)

#move to second waypoint
limb.move_to_joint_positions(way2)
rospy.sleep(2.0)

#move to third waypoint
limb.move_to_joint_positions(way3)
rospy.sleep(2.0)

#move to fourth waypoint
limb.move_to_joint_positions(way4)
rospy.sleep(2.0)

#open gripper
gripper.command_position(100)
rospy.sleep(2.0)

#return to first waypoint
limb.move_to_joint_positions(way1)

