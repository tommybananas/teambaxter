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
    except CvBridgeError as e:
      print(e)


def get_image():
  ic = image_converter()
  return ic.cv_image
