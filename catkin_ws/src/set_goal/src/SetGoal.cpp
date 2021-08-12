#include "ros/ros.h"
#include <vector>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_msgs/TFMessage.h>
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"


std::vector<float> target_position(2,0);
std::vector<float> old_position(2,0);
std::vector<float> current_position(2,0);
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
    ros::init(argc, argv, "SetGoalNode");
    ros::NodeHandle n;
    ros::Rate loop_rate(100);

    //tf tutorial
    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::Rate rate(10.0);

    ROS_INFO("Hello, World!");

    //cmd_vel è un topic, geometry_msgs/Twist è il tipo del messaggio
    ros::Publisher pub = n.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1000);
//    ros::Subscriber sub = n.subscribe("scan", 1000, laserCallback);

    int count = 0;
    while(ros::ok()) {
//        ROS_INFO("%d\n", count);

        geometry_msgs::PoseStamped msg;
        msg.pose.position.x = count*2;
        msg.pose.position.y = count*2;
        msg.pose.position.z = count*2;
        pub.publish(msg);

/*

  position: 
    x: 7.12994766235
    y: 4.42517566681
    z: 0.0
  orientation: 
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0

*/

        ros::spinOnce();
        loop_rate.sleep();
//      ros::Duration(T).sleep();
        ++count;
    }

    return 0;
}