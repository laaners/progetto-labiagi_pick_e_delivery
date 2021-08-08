#include "Eigen/Geometry"
#include "srrg_geometry/geometry2d.h"
#include "srrg_geometry/geometry3d.h"
#include "srrg_pcl/point_types.h"
#include "tf/transform_listener.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <srrg2_navigation_2d_msgs/PathFollowerStatus.h>
#include <srrg2_navigation_2d_ros/tf_helpers.h>

using namespace srrg2_core;
using namespace std;

ros::Publisher cmd_vel_publisher;
ros::Publisher status_publisher;
std::shared_ptr<tf::TransformListener> listener;
std::list<Eigen::Isometry2f, Eigen::aligned_allocator<Eigen::Isometry2f>> path_poses;
srrg2_navigation_2d_msgs::PathFollowerStatus status_msg;

// hardcoded parameters
float pure_rotation_threshold     = M_PI / 4;
float rotation_reach_threshold    = M_PI / 16;
float translation_reach_threshold = 0.5;
float rv_gain                     = 1;
float tv_gain                     = 1;
std::string map_frame_id          = "map";
std::string base_link_frame_id    = "base_link";
std::string cmd_vel_topic         = "/cmd_vel";
std::string path_topic            = "path";
std::string status_topic          = "/path_follower_status";

// ia nice logging :)
size_t total_path_size = 0;

bool computeControl(geometry_msgs::Twist& twist,
                    const Eigen::Vector3f& delta,
                    bool finalize = false) {
  twist.linear.x  = 0;
  twist.linear.y  = 0;
  twist.linear.z  = 0;
  twist.angular.x = 0;
  twist.angular.y = 0;
  twist.angular.z = 0;

  Eigen::Vector2f t = delta.head<2>();
  float theta       = delta.z();
  // goal reached condition
  // handle goal behind the target with pure rotation
  float theta_to_point = atan2(t.y(), t.x());
  if (fabs(theta_to_point) > pure_rotation_threshold && t.norm() > translation_reach_threshold) {
    twist.angular.z   = rv_gain * theta_to_point;
    status_msg.status = "initial_turning";
    return true;
  }

  if (t.norm() > translation_reach_threshold) {
    twist.linear.x    = t.norm() * tv_gain;
    twist.angular.z   = twist.linear.x * 2 * t.y() / t.squaredNorm();
    status_msg.status = "cruising";
    return true;
  }

  if (finalize && fabs(theta) > rotation_reach_threshold) {
    twist.angular.z   = rv_gain * theta;
    status_msg.status = "finalizing";
    return true;
  }

  return false;
}

void pathCallback(const nav_msgs::Path& path) {
  // ia cache and log
  total_path_size = path.poses.size();
  std::cerr << "Got path -- path size [ " << total_path_size << " ]\n";
  path_poses.clear();
  for (size_t i = 0; i < path.poses.size(); ++i) {
    Isometry2f iso;
    getIsometry(iso, path.poses[i].pose);
    path_poses.push_back(iso);
  }
  Isometry2f robot_pose;
  if (!getTfTransform(robot_pose, *listener, map_frame_id, base_link_frame_id, ros::Time(0))) {
    std::cerr << "No tf found!!!" << std::endl;
    return;
  }
  Vector3f robot_pose_2d = geometry2d::t2v(robot_pose);
  bool control           = true;
  // bb sfrondo la path
  do {
    geometry_msgs::Twist twist;
    const Isometry2f& target_in_global = path_poses.front();
    Vector3f target                    = geometry2d::t2v(robot_pose.inverse() * target_in_global);
    control                            = computeControl(twist, target, path_poses.size() == 1);
    if (!control && !path_poses.empty()) {
      path_poses.pop_front();
    }
  } while (!control && !path_poses.empty());
  cerr << "path poses after cut: " << path_poses.size() << endl;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "path_follower");
  ros::NodeHandle nh;
  ros::NodeHandle _nh("~");
  _nh.getParam("pure_rotation_threshold", pure_rotation_threshold);
  _nh.getParam("rotation_reach_threshold", rotation_reach_threshold);
  _nh.getParam("translation_reach_threshold", translation_reach_threshold);
  _nh.getParam("tv_gain", tv_gain);
  _nh.getParam("rv_gain", rv_gain);
  _nh.getParam("map_frame_id", map_frame_id);
  _nh.getParam("base_link_frame_id", base_link_frame_id);
  _nh.getParam("cmd_vel_topic", cmd_vel_topic);
  _nh.getParam("status_topic", status_topic);
  _nh.getParam("path_topic", path_topic);
  std::cerr << argv[0] << ": running with these parameters" << std::endl;
  std::cerr << "_pure_rotation_threshold:=" << pure_rotation_threshold << std::endl;
  std::cerr << "_rotation_reach_threshold:=" << rotation_reach_threshold << std::endl;
  std::cerr << "_translation_reach_threshold:=" << translation_reach_threshold << std::endl;
  std::cerr << "_tv_gain:=" << tv_gain << std::endl;
  std::cerr << "_rv_gain:=" << rv_gain << std::endl;
  std::cerr << "_map_frame_id:=" << map_frame_id << std::endl;
  std::cerr << "_base_link_frame_id:=" << base_link_frame_id << std::endl;
  std::cerr << "_cmd_vel_topic:=" << cmd_vel_topic << std::endl;
  std::cerr << "_path_topic:=" << path_topic << std::endl;
  std::cerr << "_status_topic:=" << status_topic << std::endl;

  cmd_vel_publisher = nh.advertise<geometry_msgs::Twist>(cmd_vel_topic, 10);
  status_publisher  = nh.advertise<srrg2_navigation_2d_msgs::PathFollowerStatus>(status_topic, 10);
  ros::Subscriber path_subscriber = nh.subscribe(path_topic, 10, pathCallback);
  ros::Rate loop_rate(50);
  listener.reset(new tf::TransformListener);
  while (ros::ok()) {
    // ia nice logging to understand something :)
    if (total_path_size) {
      size_t elapsed_path_poses = total_path_size - path_poses.size();
      int elapsed_path_percentage =
        std::ceil(((float) (elapsed_path_poses) / (float) (total_path_size)) * 100);
      std::flush(std::cerr);
    }

    if (!path_poses.empty()) {
      const Isometry2f& target_in_global = path_poses.front();
      Isometry2f robot_pose;
      if (!getTfTransform(robot_pose, *listener, map_frame_id, base_link_frame_id, ros::Time(0))) {
        std::cerr << "No tf found!!!" << std::endl;
        break;
      }
      status_msg.header.frame_id    = base_link_frame_id;
      status_msg.header.stamp       = ros::Time::now();
      Vector3f robot_pose_2d        = geometry2d::t2v(robot_pose);
      Vector3f target               = geometry2d::t2v(robot_pose.inverse() * target_in_global);
      status_msg.local_target_2d[0] = target.x();
      status_msg.local_target_2d[1] = target.y();
      status_msg.local_target_2d[2] = target.z();
      status_msg.robot_pose_2d[0]   = robot_pose_2d.x();
      status_msg.robot_pose_2d[1]   = robot_pose_2d.y();
      status_msg.robot_pose_2d[2]   = robot_pose_2d.z();

      geometry_msgs::Twist twist;
      if (computeControl(twist, target, path_poses.size() == 1)) {
        cmd_vel_publisher.publish(twist);
        status_msg.cmd_vel = twist;
      } else {
        path_poses.pop_front();
      }
      status_msg.num_steps_to_goal = path_poses.size();
      if (path_poses.empty()) {
        status_msg.status = "goal_reached";
      }
      status_publisher.publish(status_msg);
    } else {
      // ia reset status
      status_msg.status = "initial_turning";
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
}
