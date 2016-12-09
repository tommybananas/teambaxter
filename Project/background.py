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

# img = np.zeros((600,960,3), np.uint8)
background = cv2.imread('background2.png', 0)

def find_centers(img):
	gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
	diff = cv2.subtract(background, gray)
	out = np.absolute(diff)
	_, out = cv2.threshold(out, 15, 255, cv2.THRESH_BINARY)

	kernel = np.ones((5,5), np.uint8)
	out = cv2.erode(out, kernel, iterations=4)
	out = cv2.dilate(out, kernel, iterations=4)

	return out
	

## 
 # callback used by any image subscriber.
 # sets the global variable img
 # @param data  the image data
def callback(data):
	img = CvBridge().imgmsg_to_cv2(data, "bgr8")
	out = find_centers(img)

	cv2.imshow('img', out)
	k = cv2.waitKey(33)
	if k & 0xff is 27:
		sys.exit()

def main():
	'''initialize the system'''
	#intialize ros node
	rospy.init_node('main_driver')

	# subscribe to right hand camera
	rh_sub = rospy.Subscriber("/cameras/right_hand_camera/image", Image, callback)
	rospy.spin()

if __name__ == '__main__':
	main()
