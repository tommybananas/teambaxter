'''
This file imports all the necessary files and calls funtions to make Baxter
sort blocks by color
'''

#predefined libraries
import rospy
import baxter_interface
import struct
import sys
import time

from std_msgs.msg import String
from sensor_msgs.msg import Image

from geometry_msgs.msg import (
		PoseStamped,
		Pose,
		Point,
		Quaternion,
)

from baxter_core_msgs.srv import (
		SolvePositionIK,
		SolvePositionIKRequest,
)

#OpenCV
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

#our custom modules
import ik
import get_image
from blockVision import BlockVision

global img, limb
img = None
# img = np.zeros((600,960,3), np.uint8)


## 
 # callback used by any image subscriber.
 # sets the global variable img
 # @param data	the image data
def callback(data):
	global img

	if img is None:
		try:
			img = CvBridge().imgmsg_to_cv2(data, "bgr8")
			# cv2.imshow("img", img)
			# cv2.waitKey(33)
		except CvBridgeError as e:
			pass  

##
 # wait until we get most recent image
def get_img():
	global img

	img = None
	while img is None:
		pass
	time.sleep(.33)


def get_closest_block(color, showImage=False):
	get_img()
	vision = BlockVision(img.copy())
	vision.granularity = 3
	vision.showThreshold = False
	vision.debugColors = False
	vision.findBlocks(color)
	vision.drawAllCenters()
	if showImage:
		vision.showImage()  
	y,x,p = vision.blocks[color][0]
	return (x, y, p)

##
 # go to the location specified by x, y, z coordinates
def go_to(x, y, z):
	pos = ik.calc(x, y, z)
	limb.move_to_joint_positions(pos)


def main():
	global limb

	'''initialize the system'''
	#intialize ros node
	rospy.init_node('main_driver')

	# subscribe to right hand camera
	rh_sub = rospy.Subscriber("/cameras/right_hand_camera/image", Image, callback)

	#create a limb object
	limb = baxter_interface.Limb('right')

	#create gripper object and calibrate
	gripper = baxter_interface.Gripper("right")
	gripper.calibrate()

	#move to first waypoint
	start = (0.6, 0.0, 0.0)
	table_h = -.11
	drop_x = 527
	drop_y = 230

	go_to(*start)

	'''Find a block location and color'''
	x, y, p = get_closest_block("green", False)

	'''Move to the block'''
	calc_x = lambda y: (y-300.0)/( (600.0/(7*.0254)) )
	calc_y = lambda x: (x-480.0)/( (960.0/(18*.0254)) )

	move_x = start[0] + calc_x(y)
	move_y = calc_y(x)

	print "\nx, movement ", move_x, "\ny movement ", move_y
	go_to(move_x, move_y, start[2])

	'''Check if we are ontop of the block, otherwise move towards block'''
	x, y, p = get_closest_block("green")
	while abs(x-drop_x) >= 100.0 or abs(y-drop_y) >= 100.0:
		print("not there yet", x, y)
		if x-drop_x < -100.0:
			move_y -= .02
		if x-drop_x > 100.0:
			move_y += .02
		if y-drop_y < -100.0:
			move_x += .02
		if y-drop_y > 100.0:
			move_x -= .02

		go_to(move_x, move_y, start[2])
		x, y, p = get_closest_block("green", True)

	x, y, p = get_closest_block("green", True)
	print("x:", x, "y:", y)

	return

	# Move to right on top of the block
	x, y, p = get_closest_block("green", False)

	move_x = start[0] + calc_x(y) + 0.013
	move_y = calc_y(x) + .025

	go_to(move_x, move_y, start[2])
	x, y, p = get_closest_block("green", True)
	print x, y, x-480, y-300
	print("done")
	return

	'''Pick up block'''
	go_to(move_x, move_y, table_h)
	gripper.close()
	rospy.sleep(1)
	go_to(move_x, move_y, 0)
	time.sleep(1)
	go_to(move_x, move_y, table_h)
	time.sleep(1)
	gripper.open()
	go_to(move_x, move_y, 0)
	go_to(*start)

	'''Place the block based on color'''

	'''Go back to the start'''

if __name__ == "__main__":
	main()