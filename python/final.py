#!/usr/bin/env python
# This final piece fo skeleton code will be centred around gettign the students to follow a colour and stop upon sight of another one.

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
		self.pub = rospy.Publisher('mobile_base/commands/velocity', Twist)

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

		self.x = 640
		self.y = 480

		self.flag = True

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
			contours = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[-2]

			# Apply mask to original image
			res = cv2.bitwise_and(cv_image, cv_image, mask=mask)

			# Velocity
			velocity = Twist()

			# Draw all contours
			# cv2.drawContours(res, contours, -1, (255,255,255), 3)
			maxArea = 0

			if len(contours) == 0:
				velocity.angular.z = 0.2
				self.pub.publish(velocity)

			for cnt in contours:

				# Get contour area
				cnt_area = cv2.contourArea(cnt)

				if cnt_area > 1400:

					# Get biggest area
					maxArea = cv2.contourArea(cnt) if cv2.contourArea > maxArea else maxArea

					# Get contour centroids
					M = cv2.moments(cnt)
					cx = int(M['m10']/M['m00'])
					cy = int(M['m01']/M['m00'])
					radius = int((cnt_area/3.14)**0.5)

					# Draw contours
					cv2.circle(res, (cx, cy), radius, (255,255,255), 3)

					# Change color flag
					redpixelpoints = cv2.findNonZero(red_mask)
					greenpixelpoints = cv2.findNonZero(green_mask)
					bluepixelpoints = cv2.findNonZero(blue_mask)

					self.red_flag = 1 if redpixelpoints != None  else 0
					self.green_flag = 1 if greenpixelpoints != None else 0
					self.blue_flag = 1 if bluepixelpoints != None else 0
					print("[red, green, blue]: ", [self.red_flag, self.green_flag, self.blue_flag])

				# If red keep tracking
				if self.green_flag == 1 and self.red_flag == 0:

					print(cnt_area)

					# Center red
					if self.x - cx > 330:
						velocity.angular.z = 0.1
					elif self.x - cx < 300:
						velocity.angular.z = -0.1
					else:
						velocity.angular.z = 0

						if cnt_area > 70000:
							velocity.linear.x = -0.2

						if cnt_area < 20000:
							velocity.linear.x = 0.2

				elif self.green_flag == 1 and self.red_flag == 1:
					velocity.linear.x = 0
					velocity.angular.z = 0

				else:
					velocity.angular.z = 0.2

				self.pub.publish(velocity)

		except Exception as CvBridgeError:
			print('Error during image conversion: ', CvBridgeError)

		# Display image feed
		cv2.namedWindow('Camera_Feed')
		cv2.imshow('Camera_Feed', res)
		cv2.waitKey(5)

		#Check if the area of the shape you want is big enough to be considered
		# If it is then change the flag for that colour to be True(1)
		# if colour_max_area > #<What do you think is a suitable area?>:
			# draw a circle on the contour you're identifying as a blue object as well
			# cv2.circle(<image>, (<center x>,<center y>), <radius>, <colour (rgb tuple)>, <thickness (defaults to 1)>)
			# Then alter the values of any flags

		# #Check if a flag has been set for the stop message
		# if self.colour1_flag == 1:
		# 	if (colour_max_area) > ****:
		# 		# Too close to object, need to move backwards
		# 		# linear = positive
		# 		# angular = radius of minimum enclosing circle
		# 	else if (colour_max_area) < ****:
		# 		# Too far away from object, need to move forwards
		# 		# linear = positive
		# 		# angular = radius of minimum enclosing circle
		#
		# 	self.<publisher_name>.publish(<Move>)

		# Be sure to do this for the other colour as well
		#Show the resultant images you have created. You can show all of them or just the end result if you wish to.

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
