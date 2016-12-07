#Takes image from right hand camera and displays it using opencv
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class image_converter:

	def __init__(self):
		self.cv_image = np.zeros((600,960,3), np.uint8)
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/cameras/right_hand_camera/image",Image,self.callback)

	def callback(self,data):
		try:
			self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
			cv2.imshow("cont", self.cv_image)
			cv2.waitKey(33)
		except CvBridgeError as e:
			print(e)

		# cv2.imwrite('cam.png',self.cv_image)


def main(args):
	ic = image_converter()
	rospy.init_node('image_converter', anonymous=True)
	rospy.spin()
	# cv2.imshow("Image Window", ic.cv_image)
	# cv2.waitKey(0)
	cv2.destroyAllWindows()


if __name__ == '__main__':
		main(sys.argv)
