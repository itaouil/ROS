// Include ROS and String packages
#include "ros/ros.h"
#include "std_msgs/String.h"

// Chatter callback
void chatterCallback(const std_msgs::String::ConstPtr& msg) {

    // Print to console received message
    ROS_INFO("I heard: [%s]", msg->data.c_str());

}

// Main listener function
int main(int argc, char **argv) {

    // Create listener node
    ros::init(argc, argv, "listener")

    // Node communication handler
    ros::NodeHandle n;

    // Create subscriber
    ros::Subscriber chatter_sub = n.subscribe("chatter", 1000, chatterCallback);

    // Pumping callbacks (force them to get done)
    ros::spin();

    return 0;

}
