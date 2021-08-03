#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

void subCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
	ROS_INFO("[angle_min, angle_max, angle_increment, time_increment]: [%f,%f,%f,%f]", msg->angle_min, msg->angle_max, msg->angle_increment, msg->time_increment);

//    std_msgs/Header header
//  uint32 seq
//  time stamp
//  string frame_id
//float32 angle_min
//float32 angle_max
//float32 angle_increment
//float32 time_increment
//float32 scan_time
//float32 range_min
//float32 range_max
//float32[] ranges
//float32[] intensities
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "LaserLocationNode");

    ros::NodeHandle n;

    ROS_INFO("Hello, World!");

    //cmd_vel è un topic, geometry_msgs/Twist è il tipo del messaggio
    ros::Publisher pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    ros::Subscriber sub = n.subscribe("base_scan", 1000, subCallback);

    // Don't exit the program.
    while (ros::ok()) {
        geometry_msgs::Twist msg;
        msg.linear.x = 10;
        msg.angular.z = 5;    
        pub.publish(msg);
        ros::spinOnce();
//        loop_rate.sleep();
    }
}