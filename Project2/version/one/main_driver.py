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

global img
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
			cv2.imshow("img", img)
			cv2.waitKey(33)
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

def main():
	#intialize ros node
	rospy.init_node('main_driver')

	# subscribe to right hand camera
	rh_sub = rospy.Subscriber("/cameras/right_hand_camera/image", Image, callback)
	# rospy.spin()

	'''The first step is moving Baxter to our default starting location
	and calibrating the gripper'''
	#create a limb object
	limb = baxter_interface.Limb('right')

	#move to first waypoint
	p0 = ik.calc(0.60, 0.00, 0.00)
	limb.move_to_joint_positions(p0)

	#create gripper object
	#calibrate gripper

	'''Take an image at the starting location'''
	'''Find a block location and color'''
	get_img()
	cv2.imshow("img", img)
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

	'''Pick up the block'''
	center_x = 300
	center_y = 480
	block_x = y
	block_y = x
	move_x_px = block_x - center_x
	move_y_px = block_y - center_y
	size_y_m = 18*.0254
	size_x_m = 7*.0254
	corr_factor = 1.0
	ppmx = (600.0/size_x_m)*corr_factor
	ppmy = (960.0/size_y_m)*corr_factor
	move_x_m = move_x_px/ppmx
	move_y_m = move_y_px/ppmy

	print "\nx movement ", move_x_m, "\ny movement ", move_y_m
	block_pos = ik.calc(.6+move_x_m, 0+move_y_m, 0)
	limb.move_to_joint_positions(block_pos)

	'''Place the block based on color'''

	'''Go back to the start'''

if __name__ == "__main__":
	main()