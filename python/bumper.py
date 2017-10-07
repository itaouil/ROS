#!/usr/bin/env python
# Check ROS license, please !!!!!!!

# Import ROS and Python modules
import rospy
import math
import time
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent

# Linear velocity changer
def bump(data):
    # Check if sensor triggered
    if data.state == BumperEvent.PRESSED:
        rospy.signal_shutdown("I bumped, sorry !")

# Square publisher
def publisher():

    print("Starting")

    # Create node
    rospy.init_node('Square', anonymous=True)

    # Subscribe to bumper event
    sub = rospy.Subscriber("/mobile_base/events/bumper", BumperEvent, bump)

    # Create publisher to topic
    pub = rospy.Publisher('mobile_base/commands/velocity', Twist)

    # Loop frequence + sleep
    rate = rospy.Rate(10) #10hz

    # Create 3 dimensional vector
    desired_velocity = Twist()

    # Keep squaring turtle !
    while not rospy.is_shutdown():

        # Linear part
        desired_velocity.linear.x = 0.4
        desired_velocity.angular.z = 0
        for x in range(30):
            pub.publish(desired_velocity)
            rate.sleep()

        # Rotation
        desired_velocity.linear.x = 0
        desired_velocity.angular.z = 90 * (math.pi / 180)

        # Time of rotation
        start_time = time.time()
        for x in range(30):
            if (time.time() - start_time < 1):
                pub.publish(desired_velocity)
            rate.sleep()

if __name__ == "__main__":
    try:
        publisher()

    except rospy.ROSInterruptException:
        pass
