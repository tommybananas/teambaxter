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
	go_to(*start)

	'''Take an image at the starting location'''
	'''Find a block location and color'''
	get_img()
	# cv2.imshow("img", img)
	vision = BlockVision(img.copy())
	vision.granularity = 3
	vision.showThreshold = False
	vision.debugColors = False
	vision.findBlocks("red")
	vision.drawAllCenters()
	for color, blocks in vision.blocks.iteritems():
			for y,x,p in blocks:
					print color+' block found at '+str(y)+', '+str(x)+' ('+str(len(p))+' points)'
	vision.showImage()  
	y,x,p = vision.blocks["red"][0]
	print x,y

	'''Move to the block'''
	calc_x = lambda y: (y-300.0)/( (600.0/(7*.0254)) )
	calc_y = lambda x: (x-480.0)/( (960.0/(18*.0254)) )

	move_x = .6 + calc_x(y) #+ 0.048
	move_y = calc_y(x) #- 0.018

	print "\nx, movement ", move_x, "\ny movement ", move_y
	go_to(move_x, move_y, start[2])

	get_img()
	# cv2.imshow("img", img)
	vision = BlockVision(img.copy())
	vision.granularity = 3
	vision.showThreshold = False
	vision.debugColors = False
	vision.findBlocks("red")
	vision.drawAllCenters()
	for color, blocks in vision.blocks.iteritems():
			for y,x,p in blocks:
					print color+' block found at '+str(y)+', '+str(x)+' ('+str(len(p))+' points)'
	vision.showImage()  
	y,x,p = vision.blocks["red"][0]
	print x,y

	'''Pick up block'''
	# go_to(move_x, move_y, -.11)
	# gripper.close()
	# rospy.sleep(1)
	# go_to(move_x, move_y, 0)
	# time.sleep(1)
	# go_to(move_x, move_y, -.11)
	# time.sleep(1)
	# gripper.open()
	# go_to(move_x, move_y, 0)
	# go_to(*start)

	'''Place the block based on color'''

	'''Go back to the start'''

if __name__ == "__main__":
	main()