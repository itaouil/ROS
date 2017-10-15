#!/usr/bin/env python

from __future__ import division
import cv2
import numpy as np
import rospy
import sys

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class colourIdentifier():

	def __init__(self):
		self.bridge = CvBridge()
		self.image_listener = rospy.Subscriber('camera/rgb/image_raw', Image, self.callback)
		self.sensitivity = 20
		self.hsv_lower_green = np.array([60 - self.sensitivity, 100, 100])
		self.hsv_upper_green = np.array([60 + self.sensitivity, 255, 255])

	def callback(self, data):
		try:
			# Convert ROS image type to MAT format
			cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')

			# Change rgb format to hsv
			hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

			# Apply mask to image
			mask = cv2.inRange(hsv, self.hsv_lower_green, self.hsv_upper_green)

			# Bitwise and with original image
			res = cv2.bitwise_and(cv_image, cv_image, mask=mask)

		except Exception as CvBridgeError:
			print('Error during image conversion: ', CvBridgeError)

		# Display image feed
		cv2.namedWindow('Camera_Feed')
		cv2.imshow('Camera_Feed', res)
		cv2.waitKey(5)

def main(args):
	# Initialise node
	rospy.init_node('image_converted', anonymous=True)

	# colourIdentifier instance
	cI = colourIdentifier()

	# Spin the node
	try:
		rospy.spin()

	except KeyboardInterrupt:
		print('Shutting down node')
		cv2.destroyAllWindows()


# Check if the node is executing in the main path
if __name__ == '__main__':
	main(sys.argv)
