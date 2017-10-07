#!/usr/bin/env python
# Please check ROS license !!!!!

# Import ROS and Twis type packages
import rospy
from geometry_msgs.msg import Twist

# Circle publisher
def publisher():

    # Publish to 'mobile_base/commands/velocity' topic
    pub = rospy.Publisher('mobile_base/commands/velocity', Twist)

    # Create a node
    rospy.init_node('Walker', anonymous=True)

    # Loop rate + sleep
    rate = rospy.Rate(10) #10hz

    # Create Twist datatype (vector with three dimensions)
    desired_velocity = Twist()

    # Assign a linear and angular velocity
    desired_velocity.linear.x = 0.4 # Forward with 0.2 m/sec.
    desired_velocity.angular.z = 0.4

    # Keep rotating baby
    while not rospy.is_shutdown():
        pub.publish(desired_velocity)
        rate.sleep()

# Execute
if __name__ == "__main__":
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
