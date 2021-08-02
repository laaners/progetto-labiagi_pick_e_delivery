#include "ros/ros.h"

int main(int argc, char* argv[]) {
  // This must be called before anything else ROS-related
  ros::init(argc, argv, "vision_node");

  // Create a ROS node handle
  ros::NodeHandle nh;

  ROS_INFO("Hello, World!");

  // Don't exit the program.
  ros::spin();
}


/*
#include "ros/ros.h"
#include "std_msgs/String.h"

void chatterCallback(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv) {
    ROS_INFO("Nodo partito!");
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("base_scan", 1000, chatterCallback);

    //ros::spin();
 
    while (ros::ok()) {
        ros::spinOnce();    
    }

    return 0;
}
*/