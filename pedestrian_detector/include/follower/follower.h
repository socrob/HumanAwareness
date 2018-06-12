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
#include <std_msgs/Float64.h>
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
#include <scout_msgs/NavigationByTargetAction.h>


class Follower
{
 public:
  Follower(); //constructor
  ~Follower(); //distructor
  
  void reset();
  void loop();

  void eventInCallback(const std_msgs::String& msg);
  void PoiPositionCallback(const geometry_msgs::PointStampedConstPtr& msg);
  void robotPoseCallBack(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);

  void doneCb(const actionlib::SimpleClientGoalState& state, const scout_msgs::NavigationByTargetResultConstPtr& result);
  void activeCb();
  void feedbackCb(const scout_msgs::NavigationByTargetFeedbackConstPtr& feedback);

  void initialiseNavigationGoal();
  void stopNavigation();
  void updateTrajectory();
  void updateNavigationGoal();
  void updateHeadPosition();

  void getParams();
  void clearCostmaps();

  
 private:
  ros::NodeHandle nh_;

  // Publishers
  ros::Publisher navigation_goal_publisher_;
  ros::Publisher head_position_publisher_;
  ros::Publisher trajectory_publisher_;
  ros::Publisher residual_trajectory_publisher_;
  ros::Publisher target_pose_publisher_;
  ros::Publisher next_target_pose_publisher_;
  ros::Publisher head_camera_position_publisher_;

  // Subscribers
  ros::Subscriber event_in_subscriber_;
  ros::Subscriber poi_position_subscriber_;
  ros::Subscriber robot_position_subscriber_;

  // Service Clients
  ros::ServiceClient move_base_plan_service_client_;
  ros::ServiceClient move_base_clear_costmap_service_client_;
  
  // Action Clients
  actionlib::SimpleActionClient<scout_msgs::NavigationByTargetAction> rod_action_client_;
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_goal_action_client_;


  // Main following state
  bool following_enabled_; // TODO enable/disable with event topics
  bool initialise_navigation_;
  bool update_trajectory_;
  bool update_navigation_goal_;
  bool stop_navigation_;

  // Configuration
  // Type of navigation. Set by default through parameters.
  std::string navigation_type_, navigation_stack_;
  std::string fixed_frame_;
  double path_minimum_distance_;
  double target_pose_minimum_distance_;
  double person_pose_minimum_distance_;
  double head_camera_position_;

  // Position of the person from the Bayesian filter, and of the person in base_link.
  geometry_msgs::PoseStamped robot_pose_;
  geometry_msgs::PointStamped poi_position_;
  geometry_msgs::PointStamped relative_poi_position_;
  
  // Person's path, from the beginning untill the last received person's filtered position
  // std::vector<geometry_msgs::PointStamped> complete_person_path_;
  nav_msgs::Path poi_trajectory_;
  geometry_msgs::PoseStamped  target_pose_;

  // The residual trajectory is always the part of the trajectory from the first pose that may be set as target, to the last pose of the complete trajectory.
  // Once a pose of the residual trajectory is set as target, the poses precedent to that one should not be part of the residual anymore.
  nav_msgs::Path residual_trajectory_;

  // Position in the person's path used as target for navigation (Temporary, change to better indexing of trajectories)
  long unsigned int current_pose_pointer_;

  tf::TransformListener* listener_;

  bool isthereaPath(geometry_msgs::PoseStamped goal);
  void broadcastPoseToTF(geometry_msgs::PoseStamped p, std::string target_frame);

  // geometry_msgs::PoseStamped goal_;
  
};

#endif // PEDESTRIAN_DETECTOR_FOLLOWER_H
