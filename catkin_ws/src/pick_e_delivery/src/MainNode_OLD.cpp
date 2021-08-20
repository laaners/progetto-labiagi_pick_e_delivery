#include "ros/ros.h"
#include <string.h>
#include <vector>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_msgs/TFMessage.h>
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "pick_e_delivery/NewGoal.h"
#include "pick_e_delivery/Pose.h"

std::vector<float> target_position(2,0);
std::vector<float> old_position(2,0);
std::vector<float> current_position(2,0);
std::vector<float> caller_position(3,0);

geometry_msgs::PoseStamped new_goal_msg;
tf2_ros::Buffer tfBuffer;

typedef enum {FREE,PICK,DELIVERY,GOBACK} STATUS;

size_t n = 10;
int message_published = 0;
int cruising = 0;
int status = FREE;
char status_msg[1024];

float waitPackT = 5.0;
float waitFetchT = 5.0;

ros::Publisher pub_goal;
ros::Publisher pub_pos;
//rosrun tf view_frames

ros::Timer waitPackTimer;
ros::Timer waitFetchTimer;

void setGoalCallBack(const pick_e_delivery::NewGoal& new_goal) {
    //ricevo dal server le coordinate di un nuovo goal, ma che azione devo fare?
    if(new_goal.status == PICK && status != FREE) {
        //il robot è ancora in missione
        ROS_INFO("Il robot è ancora in missione");
        return;
    }
    else if(new_goal.status == FREE && status == DELIVERY) {
        //Il destinatario ci ha comunicato che ha prelevato il pacco
        status = FREE;
        sprintf(status_msg,"Il robot è pronto a ricevere un nuovo ordine!");
        return;
    }
    else if(new_goal.status == FREE && status == GOBACK) {
        //Il client ci ha comunicato che si è ripreso il pacco
        status = FREE;
        sprintf(status_msg,"Il robot è pronto a ricevere un nuovo ordine!");
        return;
    }
    else if(new_goal.status == PICK && status == FREE) {
        caller_position[0] = new_goal.x;
        caller_position[1] = new_goal.y;
        caller_position[2] = new_goal.theta;
        sprintf(status_msg,"Il robot si sta dirigendo verso il mittente per prelevare il pacco");
    }
    else if(new_goal.status == DELIVERY && status == PICK) {
        sprintf(status_msg,"Il robot si sta dirigendo verso il destinatario per consegnare il pacco");
    }
    else {        
        return;
    }

    status = new_goal.status;

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

    if(message_published != 0) {
        ROS_INFO("Pubblico nuovo messaggio");
        pub_goal.publish(new_goal_msg);
        message_published = 0;
    }
}

void positionCallBack(const tf2_msgs::TFMessage& tf) {
    //check if transform odom->laser_frame possible
    int transform_ok;
    transform_ok = tfBuffer.canTransform("map", "base_link", ros::Time(0));

    if(transform_ok != 0) {
        geometry_msgs::TransformStamped transformStamped;
        transformStamped = tfBuffer.lookupTransform("map", "base_link", ros::Time(0));

        //old_position[0] = current_position[0];
        //old_position[1] = current_position[1];

        current_position[0] = transformStamped.transform.translation.x;
        current_position[1] = transformStamped.transform.translation.y;

        double yaw = 2*acos(transformStamped.transform.rotation.w);
        ROS_INFO("x: %f, y: %f, yaw: %f, status: %d, status_msg: %s", transformStamped.transform.translation.x, transformStamped.transform.translation.y, yaw, status, status_msg);
        pick_e_delivery::Pose msg;
        msg.x = transformStamped.transform.translation.x;
        msg.y = transformStamped.transform.translation.y;  
        msg.yaw = yaw;
        msg.status = status;
        msg.status_msg = status_msg;
        pub_pos.publish(msg);

    }
}

void waitPackCallBack(const ros::TimerEvent& event) {
    switch(status) {
        case PICK: {
            status = FREE;
            sprintf(status_msg,"Il robot non ha ricevuto nessun pacco, ora è in FREE");
            break;
        }
        case DELIVERY: {
            sprintf(status_msg,"Il robot è in DELIVERY, tutto corretto");
//            sprintf(status_msg,"Il robot si sta dirigendo verso il destinatario per consegnare il pacco");
            break;
        }
        default: {
            ROS_INFO("ERRORE, stato non riconosciuto");
            break;
        }
    }
    waitPackTimer.stop();
}

void waitFetchCallBack(const ros::TimerEvent& event) {
    switch(status) {
        case DELIVERY: {
            status = GOBACK;
            sprintf(status_msg,"Nessuno è venuto a prelevare il pacco, torno indietro e diventerò FREE");

            new_goal_msg.header.seq = n;
            n++;

            new_goal_msg.header.stamp = ros::Time::now();
            new_goal_msg.header.frame_id = "map";

            new_goal_msg.pose.position.x = caller_position[0];
            new_goal_msg.pose.position.y = caller_position[1];
            new_goal_msg.pose.position.z = 0;

            new_goal_msg.pose.orientation.x = 0;
            new_goal_msg.pose.orientation.y = 0;
            new_goal_msg.pose.orientation.z = 0;
            new_goal_msg.pose.orientation.w = caller_position[2];

            message_published = 1;
            cruising = 1;

            //save goal position
            target_position[0] = caller_position[0];
            target_position[1] = caller_position[1];

            if(message_published != 0) {
                ROS_INFO("Pubblico nuovo messaggio");
                pub_goal.publish(new_goal_msg);
                message_published = 0;
            }
            break;
        }
        default: {
            ROS_INFO("ERRORE, stato non riconosciuto");
            break;
        }
    }
    waitFetchTimer.stop();
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
            sprintf(status_msg,"Sono arrivato a destinazione!");
            cruising = 0;

            switch(status) {
                case PICK: {
                    waitPackTimer.start();
                    break;
                }
                case DELIVERY: {
                    waitFetchTimer.start();
                    break;
                }
                case GOBACK: {
                    //status = FREE;
                    sprintf(status_msg,"Sono da chi mi aveva originariamente chiamato, nessuno è venuto a prelevare il suo pacco, liberami!");
                    break;
                }
                default: {
                    ROS_INFO("ERRORE, stato non riconosciuto");
                    break;
                }
            }

        }
    }
}

void check2_CallBack(const ros::TimerEvent& event) {
    if(cruising != 0) {
        ROS_INFO("Il robot non sta navigando, controllo se è passato troppo tempo");
        float distance;
        distance = sqrt(pow(current_position[0]-target_position[0],2)+pow(current_position[1]-target_position[1],2));
        if(distance > 0.5) {
            sprintf(status_msg,"Non riesco a raggiungere il goal");
            ROS_INFO("Timeout! Non riesco a raggiungere il goal");
        }
    }
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "MainNode");
    ros::NodeHandle n;
    ros::Rate loop_rate(100);

    pub_goal = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1000);
    pub_pos = n.advertise<pick_e_delivery::Pose>("/pick_e_delivery/Pose", 1000);

    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::Rate rate(10.0);

    ros::Subscriber sub = n.subscribe("New_Goal", 1000, setGoalCallBack);

    //devo sapere la posizione del robot nel tempo
    ros::Subscriber sub_tf = n.subscribe("tf", 1000, positionCallBack);

    ros::Timer timer1 = n.createTimer(ros::Duration(0.5),check1_CallBack);
    ros::Timer timer2 = n.createTimer(ros::Duration(50),check2_CallBack);

    waitPackTimer = n.createTimer(ros::Duration(waitPackT), waitPackCallBack);
    waitPackTimer.stop();

    waitFetchTimer = n.createTimer(ros::Duration(waitFetchT), waitFetchCallBack);
    waitFetchTimer.stop();


    sprintf(status_msg,"Il robot è pronto a ricevere un nuovo ordine!");

    int count = 0;
    while(ros::ok()) {
        /*
        if(message_published != 0) {
            ROS_INFO("Pubblico nuovo messaggio");
            status = PICK;
            pub_goal.publish(new_goal_msg);
            message_published = 0;
        }
        */


        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }

    return 0;
}