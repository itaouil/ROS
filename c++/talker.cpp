// Include ROS and Datatypes packages
#include "ros/ros.h"
#include "std_msgs/String.h"

// Useful for String Manipulation
#include <sstream>

// Main talker function
int main(int argc, char **argv) {

    // Create node (process)
    ros::init(argc, argv, "talker");

    // Node communication handler
    ros::NodeHandle n;

    // Create publisher
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000)

    // Sleeping rate
    ros::Rate loop_rate(10);

    // Counter
    int count = 0;

    // Publish messages
    while (ros::ok()) {

        // Data object
        std_msgs::String msg;

        // Populate message
        std::stringstream ss;
        ss << "Hello World " << count;
        msg.data = ss.str();

        // Logger
        ROS_INFO("%s", msg.data.c_str());

        // Send message
        chatter_pub.publish(msg)

        // To have in case of subscription (callbacks)
        ros.spinOnce();

        loop_rate.sleep();
        ++count;

    }

    return 0
}
