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

	zs = [-0.06,0.0,0.1,0.2]
	xys = [(0.50,0.0),(0.55,0.0),(0.60,0.0),(0.65,0.0),(0.70,0.0),
	(0.50,0.1),(0.55,0.1),(0.60,0.1),(0.65,0.1),(0.70,0.1),
	(0.50,-0.1),(0.55,-0.1),(0.60,-0.1),(0.65,-0.1),(0.70,-0.1)]

	for z in zs:
		for xy in xys:
			x,y = xy
			p = (x,y,z)
			print p
			go_to(*p)
			time.sleep(1.5)
			get_img()
			cv2.imwrite('img_debug/img_'+str(x)+'_'+str(y)+'_'+str(z)+'_.png',img.copy())
		

	return

	

if __name__ == "__main__":
	main()