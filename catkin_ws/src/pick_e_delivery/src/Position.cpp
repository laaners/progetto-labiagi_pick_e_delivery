#include "ros/ros.h"
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include "std_msgs/String.h"
#include <tf2_msgs/TFMessage.h>
#include "pick_e_delivery/Pose.h"

tf2_ros::Buffer tfBuffer;
ros::Publisher pub;

void positionCallBack(const tf2_msgs::TFMessage& tf) {
    int transform_ok;
    transform_ok = tfBuffer.canTransform("map", "base_link", ros::Time(0));

    if(transform_ok != 0) {
        geometry_msgs::TransformStamped transformStamped;
        transformStamped = tfBuffer.lookupTransform("map", "base_link", ros::Time(0));
        double yaw = 2*acos(transformStamped.transform.rotation.w);
        ROS_INFO("x: %f, y: %f, yaw: %f", transformStamped.transform.translation.x, transformStamped.transform.translation.y, yaw);

        pick_e_delivery::Pose msg;
        msg.x = transformStamped.transform.translation.x;
        msg.y = transformStamped.transform.translation.y;  
        msg.yaw = yaw;  
        pub.publish(msg);

    }
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "PositionNode");
    ros::NodeHandle n;
    ros::Rate loop_rate(100);

    //tf tutorial
    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::Rate rate(10.0);

    ros::Subscriber sub_tf = n.subscribe("tf", 1000, positionCallBack);
    pub = n.advertise<pick_e_delivery::Pose>("/pick_e_delivery/Pose", 1000);


    int count = 0;
    while(ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }

    return 0;
}