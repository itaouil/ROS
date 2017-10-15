#!/usr/bin/env python

from __future__ import division
import cv2
import numpy as np
import rospy
import sys
import math

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class colourIdentifier():

	def __init__(self):
		self.bridge = CvBridge()
		self.image_listener = rospy.Subscriber('camera/rgb/image_raw', Image, self.callback)

		self.red_sensitivity = 5
		self.blue_sensitivity = 10
		self.green_sensitivity = 10

		# Green upper and lower bound
		self.hsv_lower_green = np.array([60 - self.green_sensitivity, 100, 100])
		self.hsv_upper_green = np.array([60 + self.green_sensitivity, 255, 255])

		# Blue upper and lower bound
		self.hsv_lower_blue = np.array([110 - self.blue_sensitivity, 50, 50])
		self.hsv_upper_blue = np.array([130 + self.blue_sensitivity, 255, 255])

		# Red upper and lower bound
		self.hsv_lower_red = np.array([5 - self.red_sensitivity, 50, 50])
		self.hsv_upper_red = np.array([15 + self.red_sensitivity, 255, 255])

		# Colors detection flags
		self.red_flag = 0
		self.green_flag = 0
		self.blue_flag = 0

	def callback(self, data):

			try:
				# Convert ROS image type to MAT format
				cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')

				# Change rgb format to hsv
				hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

				# Green mask
				green_mask = cv2.inRange(hsv, self.hsv_lower_green, self.hsv_upper_green)

				# Blue mask
				blue_mask = cv2.inRange(hsv, self.hsv_lower_blue, self.hsv_upper_blue)

				# Red mask
				red_mask = cv2.inRange(hsv, self.hsv_lower_red, self.hsv_upper_red)

				# Intermediate mask
				int_mask = cv2.bitwise_or(green_mask, blue_mask)

				# Final mask
				mask = cv2.bitwise_or(int_mask, red_mask)

				# Find contours
				contours, hierarchy = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

				# Apply mask to original image
				res = cv2.bitwise_and(cv_image, cv_image, mask=mask)

				# Draw all contours
				cv2.drawContours(res, contours, -1, (255,255,255), 3)

				# Find images color in view
				redpixelpoints = cv2.findNonZero(red_mask)
				greenpixelpoints = cv2.findNonZero(green_mask)
				bluepixelpoints = cv2.findNonZero(blue_mask)

				# Colors presence
				self.red_flag = 1 if redpixelpoints != None  else 0
				self.green_flag = 1 if greenpixelpoints != None else 0
				self.blue_flag = 1 if bluepixelpoints != None else 0
				print("Colors presence [red, green, blue]: ", [self.red_flag, self.green_flag, self.blue_flag])

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
