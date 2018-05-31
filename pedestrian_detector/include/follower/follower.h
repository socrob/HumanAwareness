#ifndef PEDEST0RIAN_DETECTOR_FOLLOWER_H
#define PEDESTRIAN_DETECTOR_FOLLOWER_H

#include <string>
#include <vector>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

// Messages
#include <std_msgs/String.h>
#include <std_msgs/UInt8MultiArray.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>

// Services
#include <std_srvs/Empty.h>
#include <nav_msgs/GetPlan.h>

// Actions
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>


class Follower
{
 public:
  Follower(); //constructor
  ~Follower(); //distructor
  
  void setup();
  void loop();

  void filteredPersonPositionCallback(const geometry_msgs::PointStampedConstPtr& msg);
  void robotPoseCallBack(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);
  
  void updateNavigationGoal();
  void updateHeadPosition();

  void getParams();
  void clearCostmaps();

  
 private:
  ros::NodeHandle nh_;

  // Publishers
  ros::Publisher pub_pose_;
  ros::Publisher pub_pose_in_;
  ros::Publisher pub_head_rot_;
  ros::Publisher trajectory_publisher_;
  ros::Publisher current_target_pose_publisher_;
  ros::Publisher next_target_pose_publisher_;

  // Subscribers
  ros::Subscriber filtered_person_position_subscriber_;
  ros::Subscriber robot_position_subscriber_;

  // Service Clients
  ros::ServiceClient plan_client_;
  ros::ServiceClient clear_client_;
  
  // Main following state
  bool following_enabled_; // TODO enable/disable with event topics

  // Type of navigation. Set by default through parameters. Can be updated dinamically to implement different navigation stategies.
  // TODO change dinamically (state machine?)
  std::string navigation_type_, navigation_stack_;

  // flag used to know when we have received a callback
  bool new_person_position_received_;

  // Position of the person from the Bayesian filter
  geometry_msgs::PointStamped filtered_person_position_;

  // Position of the person in base_link.
  geometry_msgs::PointStamped relative_person_position_;
  
  // Person's path, from the beginning untill the last received person's filtered position
  std::vector<geometry_msgs::PointStamped> complete_person_path_;
  nav_msgs::Path complete_person_trajectory_;

  // Position in the person's path used as target for navigation
  long unsigned int path_target_pointer_;
  
  // for storing the arguments that will be read from param server
  std::vector<std::string> script_arguments_;

  // out topics
  //std_msgs::String stop_;
  //std_msgs::String current_robot_position_;

  // position of the head
  double head_pos; 

  tf::TransformListener* listener_;

  bool isthereaPath(geometry_msgs::PoseStamped goal);
  
  //std::vector<geometry_msgs::PoseStamped> personPoses_;

  geometry_msgs::PoseStamped keptPose_;

  geometry_msgs::Point keptPoint_;
  
  geometry_msgs::PoseStamped current_robot_position_;

  geometry_msgs::PoseStamped goal_;
  geometry_msgs::Point pointGoal_;
  
  bool path_;

  //TF stuff
  static tf::TransformBroadcaster br_;
  tf::Transform transform;
  tf::Quaternion q;
  
};

#endif // PEDESTRIAN_DETECTOR_FOLLOWER_H
