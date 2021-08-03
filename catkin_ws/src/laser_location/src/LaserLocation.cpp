#include "ros/ros.h"
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

tf2_ros::Buffer tfBuffer;

//rosrun tf view_frames

//void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
void laserCallback(const sensor_msgs::LaserScan& msg) {
    ROS_INFO("I read laser_scan");

    //check if transform odom->laser_frame possible
    int transform_ok;
    transform_ok = tfBuffer.canTransform("odom", "laser_frame", ros::Time(0));
    ROS_INFO("Transform possible, [%i]", transform_ok);

    if(transform_ok != 0) {
        geometry_msgs::TransformStamped transformStamped;
        transformStamped = tfBuffer.lookupTransform("odom", "laser_frame", ros::Time(0));
        double yaw = 2*acos(transformStamped.transform.rotation.w);
        ROS_INFO("x: %f, y: %f, yaw: %f", transformStamped.transform.translation.x, transformStamped.transform.translation.y, yaw);
    }
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "LaserLocationNode");
    ros::NodeHandle n;
    ros::Rate loop_rate(100);

    //tf tutorial
    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::Rate rate(10.0);

    ROS_INFO("Hello, World!");

    //cmd_vel è un topic, geometry_msgs/Twist è il tipo del messaggio
    ros::Publisher pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    ros::Subscriber sub = n.subscribe("scan", 1000, laserCallback);

    int count = 0;
    while(ros::ok()) {
//        ROS_INFO("%d\n", count);

        geometry_msgs::Twist msg;
        msg.linear.x = 10;
        msg.angular.z = 5;    
        pub.publish(msg);

        ros::spinOnce();
        loop_rate .sleep();
//      ros::Duration(T).sleep();
        ++count;
    }

    return 0;
}