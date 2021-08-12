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
#include "set_goal/NewGoal.h"


std::vector<float> target_position(2,0);
std::vector<float> old_position(2,0);
std::vector<float> current_position(2,0);

geometry_msgs::PoseStamped new_goal_msg;
tf2_ros::Buffer tfBuffer;

size_t n = 10;
int message_published = 0;
int cruising = 0;

//rosrun tf view_frames

void setGoalCallBack(const set_goal::NewGoal& new_goal) {
    ROS_INFO("I read laser_scan");

    //sto impostando i parametri del messaggio del tipo geometry_msgs/PoseStamped
    /*
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
geometry_msgs/Pose pose
  geometry_msgs/Point position
    float64 x
    float64 y
    float64 z
  geometry_msgs/Quaternion orientation
    float64 x
    float64 y
    float64 z
    float64 w*/

    new_goal_msg.header.seq = n;
    n++;

    new_goal_msg.header.stamp = ros::Time::now();
    new_goal_msg.header.frame_id = "map";

    new_goal_msg.pose.position.x = new_goal.x;
    new_goal_msg.pose.position.y = new_goal.y;
    new_goal_msg.pose.position.z = 0;

    new_goal_msg.pose.orientation.x = 0;
    new_goal_msg.pose.orientation.y = 0;
    new_goal_msg.pose.orientation.z = 0;
    new_goal_msg.pose.orientation.w = new_goal.theta;
 
    message_published = 1;
    cruising = 1;

    //save goal position
    target_position[0] = new_goal.x;
    target_position[1] = new_goal.y;
}

void positionCallBack(const tf2_msgs::TFMessage& tf) {
    //check if transform odom->laser_frame possible
    int transform_ok;
    transform_ok = tfBuffer.canTransform("map", "base_link", ros::Time(0));
    //ROS_INFO("Transform possible, [%i]", transform_ok);

    if(transform_ok != 0) {
        geometry_msgs::TransformStamped transformStamped;
        transformStamped = tfBuffer.lookupTransform("map", "base_link", ros::Time(0));

        //old_position[0] = current_position[0];
        //old_position[1] = current_position[1];

        current_position[0] = transformStamped.transform.translation.x;
        current_position[1] = transformStamped.transform.translation.y;
    }
}

void check1_CallBack(const ros::TimerEvent& event) {
    if(cruising != 0) {
        ROS_INFO("Controllo se il robot sta navigando");
        float distance;
        distance = sqrt(pow(current_position[0]-old_position[0],2)+pow(current_position[1]-old_position[1],2));
        if(distance < 0.8) {
            ROS_INFO("Sono bloccato");
        }
        distance = sqrt(pow(current_position[0]-target_position[0],2)+pow(current_position[1]-target_position[1],2));
        if(distance < 1.5) {
            ROS_INFO("Sono arrivato a destinazione");
            cruising = 0;
        }
    }
}

void check2_CallBack(const ros::TimerEvent& event) {
    if(cruising != 0) {
        ROS_INFO("Il robot non sta navigando, controllo se è passato troppo tempo");
        float distance;
        distance = sqrt(pow(current_position[0]-target_position[0],2)+pow(current_position[1]-target_position[1],2));
        if(distance > 0.5) {
            ROS_INFO("Timeout! Non riesco a raggiungere il goal");
        }
    }
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "SetGoalNode");
    ros::NodeHandle n;
    ros::Rate loop_rate(100);

    ros::Publisher pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1000);

    //tf tutorial
    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::Rate rate(10.0);

    ros::Subscriber sub = n.subscribe("New_Goal", 1000, setGoalCallBack);

    //devo sapere la posizione del robot nel tempo
    ros::Subscriber sub_tf = n.subscribe("tf", 1000, positionCallBack);

    ros::Timer timer1 = n.createTimer(ros::Duration(0.5),check1_CallBack);
    ros::Timer timer2 = n.createTimer(ros::Duration(50),check2_CallBack);

    int count = 0;
    while(ros::ok()) {
        if(message_published != 0) {
            ROS_INFO("Pubblico nuovo messaggio");
            pub.publish(new_goal_msg);
            message_published = 0;
        }


        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }

    return 0;
}