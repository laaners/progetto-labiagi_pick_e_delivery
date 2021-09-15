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
#include "pick_e_delivery/Timeout.h"
#include "pick_e_delivery/setTooLongInterval.h"
#include "pick_e_delivery/setWaitPackInterval.h"

std::vector<float> target_position(2,0);
std::vector<float> current_position(2,0);
std::vector<float> old_position(2,0);
std::vector<float> caller_position(3,0);

tf2_ros::Buffer tfBuffer;

typedef enum {FREE,PICK,AT_SRC,DELIVERY,AT_DST,GOBACK} STATUS;

size_t n = 10;
int message_published = 0;
int cruising = 0;
int blocked = 0;
int status = FREE;
std::string status_msg;
std::string sender;
std::string receiver;

int stuckv[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int i = 0;
float waitPackT = 30;
float tooLongT = 50;

ros::Publisher pub_goal;
ros::Publisher pub_pos;
ros::Publisher pub_timeout;
//{ isBlocked, notBlocked, tooLongPick, tooLongDelivery, noPack}
//rosrun tf view_frames

ros::Timer waitPackTimer;
ros::Timer tooLongTimer;

//FUNZIONI AUSILIARIE==========================================================================================================================================================

void pubNewGoal(float x, float y, float theta) {
    geometry_msgs::PoseStamped new_goal_msg;
    
    new_goal_msg.header.seq = n;
    n++;

    new_goal_msg.header.stamp = ros::Time::now();
    new_goal_msg.header.frame_id = "map";

    new_goal_msg.pose.position.x = x;
    new_goal_msg.pose.position.y = y;
    new_goal_msg.pose.position.z = 0;

    new_goal_msg.pose.orientation.x = 0;
    new_goal_msg.pose.orientation.y = 0;
    new_goal_msg.pose.orientation.z = 0;
    new_goal_msg.pose.orientation.w = theta;

    message_published = 1;
    cruising = 1;

    //save goal position
    target_position[0] = x;
    target_position[1] = y;

    if(message_published != 0) {
        ROS_INFO("Pubblico nuovo messaggio");
        pub_goal.publish(new_goal_msg);
        message_published = 0;
    }
}

void pubNewTimeout(std::string event) {
    pick_e_delivery::Timeout new_timeout_msg;
    new_timeout_msg.event = event;
    pub_timeout.publish(new_timeout_msg);
}

//SUBSCRIBERS CALLBACK==========================================================================================================================================================

void setGoalCallBack(const pick_e_delivery::NewGoal& new_goal) {
    //ricevo dall'utente le coordinate di un nuovo goal, ma che azione devo fare?
//FREE------------------------------------------------------------------------------------------------------------------------------------
    if(status == FREE && new_goal.command == PICK) {
        //Il robot ha ricevuto un comando per prelevare il pacco ed è libero: inizia a muoversi
        caller_position[0] = new_goal.x;
        caller_position[1] = new_goal.y;
        caller_position[2] = new_goal.theta;
        status = PICK;
        sender = new_goal.user;    
        pubNewGoal(new_goal.x,new_goal.y,new_goal.theta);
        tooLongTimer.start();
        status_msg = "Il robot si sta dirigendo verso il mittente per prelevare il pacco";
    }
//PICK------------------------------------------------------------------------------------------------------------------------------------
    else if(status == PICK && new_goal.command == FREE) {
        //L'utente vuole liberare il robot, è in PICK quindi non ha nessun pacco
        status = FREE;
        pubNewGoal(current_position[0],current_position[1],caller_position[2]);
        status_msg = "Il mittente ha liberato il robot";
    }
//DELIVERY------------------------------------------------------------------------------------------------------------------------------------
    else if(status == DELIVERY && new_goal.command == GOBACK) {
        //L'utente vuole liberare il robot, il robot dovrà tornare indietro
        status = GOBACK;
        receiver = "none";
        pubNewGoal(caller_position[0],caller_position[1],caller_position[2]);
        status_msg = "Il mittente rivuole il pacco indietro";
    }
//AT_DST------------------------------------------------------------------------------------------------------------------------------------
    else if(status == AT_DST && new_goal.command == FREE) {
        //Il destinatario ci ha comunicato che ha prelevato il pacco, il robot quindi è ora libero
        status = FREE;
        status_msg = "Il robot è pronto a ricevere un nuovo ordine!";
    }
    else if(status == AT_DST && new_goal.command == GOBACK) {
        //L'utente vuole liberare il robot, il robot dovrà tornare indietro
        status = GOBACK;
        receiver = "none";
        pubNewGoal(caller_position[0],caller_position[1],caller_position[2]);
        status_msg = "Il mittente rivuole il pacco indietro";
    }
//AT_SRC------------------------------------------------------------------------------------------------------------------------------------
    else if(status == AT_SRC && new_goal.command == DELIVERY) {
        //L'utente ha comunicato al robot che ha caricato il pacco e ora si dirigerà verso la destinazione
        status = DELIVERY;
        receiver = new_goal.user;
        pubNewGoal(new_goal.x,new_goal.y,new_goal.theta);
        tooLongTimer.start();
        waitPackTimer.stop();
        status_msg = "Il robot si sta dirigendo verso il destinatario per consegnare il pacco";
    }
    else if(status == AT_SRC && new_goal.command == FREE) {
        //Il client ci ha comunicato che si è ripreso il pacco
        status = FREE;
        waitPackTimer.stop();
        status_msg = "Il robot è pronto a ricevere un nuovo ordine!";
    }
//Comandi non validi dall'utente------------------------------------------------------------------------------------------------------------------------------------
    else if(status != FREE && new_goal.command == PICK) {
        //Qualcuno ha dato un nuovo incarico al robot, ma è ancora in missione
        ROS_INFO("Il robot è ancora in missione");
    }
    return;
}

void positionCallBack(const tf2_msgs::TFMessage& tf) {
    //check if transform odom->laser_frame possible
    int transform_ok;
    transform_ok = tfBuffer.canTransform("map", "base_link", ros::Time(0));
    if(transform_ok != 0) {
        geometry_msgs::TransformStamped transformStamped;
        transformStamped = tfBuffer.lookupTransform("map", "base_link", ros::Time(0));

        old_position[0] = current_position[0];
        old_position[1] = current_position[1];

        current_position[0] = transformStamped.transform.translation.x;
        current_position[1] = transformStamped.transform.translation.y;

        if((old_position == current_position) && status != FREE) stuckv[i] = 1;
        else stuckv[i] = 0;
        i = (i+1)%10;

        double yaw = 2*acos(transformStamped.transform.rotation.w);
        //ROS_INFO("x: %f, y: %f, yaw: %f, status: %d, status_msg: %s", transformStamped.transform.translation.x, transformStamped.transform.translation.y, yaw, status, status_msg);
        pick_e_delivery::Pose new_pose_msg;
        new_pose_msg.x = transformStamped.transform.translation.x;
        new_pose_msg.y = transformStamped.transform.translation.y;  
        new_pose_msg.yaw = yaw;
        new_pose_msg.status = status;
        new_pose_msg.status_msg = status_msg;
        new_pose_msg.sender = sender;
        new_pose_msg.receiver = receiver;
        pub_pos.publish(new_pose_msg);
    }
}

//TIMERS CALLBACK==========================================================================================================================================================

void waitPackCallBack(const ros::TimerEvent& event) {
    switch(status) {
        case AT_SRC: {
            status = FREE;
            status_msg = "Il robot non ha ricevuto nessun pacco, ora è in FREE";
            pubNewTimeout("noPack");
            break;
        }
        default: {
            ROS_INFO("ERRORE, stato non riconosciuto 168");
            break;
        }
    }
    waitPackTimer.stop();
}

void isCruisingCallBack(const ros::TimerEvent& event) {
    //check if FREE, then reset sender, receiver and stuckv
    if(status == FREE) {
        sender = "none";
        receiver = "none";
    }

    if(cruising != 0) {
        ROS_INFO("Controllo se il robot sta navigando");
        float distance;
        distance = sqrt(pow(current_position[0]-target_position[0],2)+pow(current_position[1]-target_position[1],2));
        if(distance < 1.0) {
            ROS_INFO("Sono arrivato a destinazione");
            cruising = 0;
            switch(status) {
                case PICK: {
                    tooLongTimer.stop();
                    status = AT_SRC;
                    status_msg = "Sono arrivato a destinazione, attendo che qualcuno metta un pacco da consegnare!";
                    waitPackTimer.start();
                    break;
                }
                case DELIVERY: {
                    tooLongTimer.stop();
                    status = AT_DST;
                    status_msg = "Sono arrivato a destinazione, attendo che mi qualcuno prenda il pacco!";
                    break;
                }
                case GOBACK: {
                    status = AT_SRC;
                    status_msg = "Sono da chi mi aveva originariamente chiamato, nessuno è venuto a prelevare il suo pacco, liberami!";
                    break;
                }
                default: {
                    ROS_INFO("ERRORE, stato non riconosciuto 206");
                    break;
                }
            }
        }
        else {
            if((status == DELIVERY) || (status == PICK) || (status == GOBACK)) {
                int stuck = 0, j = 0;
                for(j = 0; j < 10; j++) {
                    stuck += stuckv[j];
                }
                if(stuck == 10) {
                    ROS_INFO("Sono bloccato");
                    status_msg = "Servizio non disponibile";
                    pubNewTimeout("isBlocked");
                    blocked = 1;
                }
                else {
                    status_msg = "Il robot sta navigando";
                    pubNewTimeout("notBlocked");
                    blocked = 0;
                }
            }
        }
    }
}

void tooLongCallBack(const ros::TimerEvent& event) {
    if(cruising != 0) {
        ROS_INFO("Il robot non sta navigando, controllo se è passato troppo tempo");
        float distance;
        distance = sqrt(pow(current_position[0]-target_position[0],2)+pow(current_position[1]-target_position[1],2));
        if(distance > 0.5) {
            ROS_INFO("Timeout! Non riesco a raggiungere il goal");
            switch(status) {
                case PICK: {
                    status = FREE;
                    status_msg = "Non riesco a raggiungere chi mi ha chiamato, mi dispiace";
                    pubNewGoal(current_position[0],current_position[1],caller_position[2]);
                    if(!blocked) {
                        pubNewTimeout("tooLongPick");
                    }
                    break;
                }
                case DELIVERY: {
                    status = GOBACK;
                    status_msg = "Non riesco a raggiungere il goal, torno indietro";
                    pubNewGoal(caller_position[0],caller_position[1],caller_position[2]);
                    if(!blocked) {
                        pubNewTimeout("tooLongDelivery");
                    }
                    break;
                }
                default: {
                    ROS_INFO("ERRORE, stato non riconosciuto 236");
                    break;
                }
            }
        }
    }
}

//SERVICES CALLBACK==========================================================================================================================================================

bool setTooLongInterval(pick_e_delivery::setTooLongInterval::Request  &req, pick_e_delivery::setTooLongInterval::Response &res) {
  ROS_INFO("Periodo ricevuto: x=%f", req.period);
  if((float)req.period > 0.0) {
    tooLongT = req.period;
    tooLongTimer.setPeriod(ros::Duration(tooLongT));
    tooLongTimer.stop();
    ROS_INFO("Periodo settato");
  }
  return true;
}

bool setWaitPackInterval(pick_e_delivery::setWaitPackInterval::Request  &req, pick_e_delivery::setWaitPackInterval::Response &res) {
  ROS_INFO("Periodo ricevuto: x=%f", req.period);
  if((float)req.period > 0.0) {
    waitPackT = req.period;
    waitPackTimer.setPeriod(ros::Duration(waitPackT));
    waitPackTimer.stop();
    ROS_INFO("Periodo settato");
  }
  return true;
}

//MAIN==========================================================================================================================================================

int main(int argc, char* argv[]) {
    if(argc == 3) {
        waitPackT = atof(argv[1]);
        tooLongT = atof(argv[2]);
    }
    sender = "none";
    receiver = "none";

    ros::init(argc, argv, "MainNode");
    ros::NodeHandle n;
    ros::Rate loop_rate(100);

    ROS_INFO("MainNode partito");

//PUBLISHERS------------------------------------------------------------------
    //devo pubblicare un nuovo goal
    pub_goal = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1000);

    //devo comunicare la posizione e lo stato del robot nel tempo
    pub_pos = n.advertise<pick_e_delivery::Pose>("/pick_e_delivery/Pose", 1000);

    //devo comunicare eventi di timeout
    pub_timeout = n.advertise<pick_e_delivery::Timeout>("/pick_e_delivery/Timeout",1000);

//SUBSCRIBERS------------------------------------------------------------------
    //devo sapere la posizione del robot nel tempo
    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::Rate rate(10.0);
    ros::Subscriber sub_tf = n.subscribe("tf", 1000, positionCallBack);

    //aspetto un goal
    ros::Subscriber sub = n.subscribe("/pick_e_delivery/NewGoal", 1000, setGoalCallBack);

//TIMERS------------------------------------------------------------------
    //sta navigando?
    ros::Timer isCruising = n.createTimer(ros::Duration(0.5),isCruisingCallBack);

    //è passato troppo tempo, il robot non ce la fa più
    tooLongTimer = n.createTimer(ros::Duration(tooLongT),tooLongCallBack);
    tooLongTimer.stop();

    //il robot aspetta che gli venga dato un pacco
    waitPackTimer = n.createTimer(ros::Duration(waitPackT), waitPackCallBack);
    waitPackTimer.stop();

//SERVICES------------------------------------------------------------------
    //cambiare periodo di tooLongTimer
    ros::ServiceServer srvTooLong = n.advertiseService("pick_e_delivery/setTooLongInterval", setTooLongInterval);

    //cambiare periodo di waitPackTimer
    ros::ServiceServer srvWaitPack = n.advertiseService("pick_e_delivery/setWaitPackInterval", setWaitPackInterval);

//MAIN LOOP------------------------------------------------------------------
    status_msg = "Il robot è pronto a ricevere un nuovo ordine!";

    int count = 0;
    while(ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }

    return 0;
}