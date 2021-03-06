#ifndef PEDEST0RIAN_DETECTOR_FOLLOWER_H
#define PEDESTRIAN_DETECTOR_FOLLOWER_H

#include <stdlib.h>
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
  ~Follower(); //destructor
  
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

  void clearCostmaps();

  
 private:
  ros::NodeHandle nh_;

  // Publishers
  ros::Publisher event_out_publisher_;
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


  // State from external events
  ros::Time start_stamp_;
  bool are_subscribers_created_;
  bool update_navigation_goal_;
  bool poi_position_received_;
  bool initialise_navigation_;
  bool update_trajectory_;
  bool following_enabled_;
  bool stop_navigation_;
  
  // State from internal events
  ros::Time path_completed_stamp_;
  bool is_poi_in_starting_position_;
  bool is_path_completed_;
  bool is_poi_stopped_;
  bool is_poi_tracked_;
  bool is_poi_close_; 
  bool is_poi_lost_;
  bool e_success_;
  bool e_failure_;

  // Configuration
  std::string navigation_type_, navigation_stack_;
  std::string fixed_frame_;
  double path_minimum_distance_;
  double target_pose_minimum_distance_;
  double person_pose_minimum_distance_;
  double head_camera_position_;
  double poi_moving_radius_;
  ros::Duration success_timeout_;
  ros::Duration poi_lost_timeout_;
  ros::Duration poi_stopped_timeout_;
  ros::Duration poi_tracking_timeout_;
  
  // Position of the person from the Bayesian filter, and of the person in base_link.
  geometry_msgs::PoseStamped  robot_pose_;
  geometry_msgs::PointStamped poi_position_;
  geometry_msgs::PointStamped relative_poi_position_;
  geometry_msgs::PointStamped poi_starting_position_;
  geometry_msgs::PointStamped poi_moving_position_;
  
  // The path of the person of interest.
  nav_msgs::Path poi_trajectory_;
  geometry_msgs::PoseStamped  target_pose_;
  
  // The residual trajectory is always the part of the trajectory from the target pose to the last pose of the complete trajectory.
  nav_msgs::Path residual_trajectory_;
  
  // Position in the person's path used as target for navigation (Temporary, change to better indexing of trajectories)
  long unsigned int current_pose_pointer_;
  
  tf::TransformListener* listener_;
  
  void createSubscribers();
  void destroySubscribers();
  void updateActionGoal(bool target_following);
  bool isPathCompleted();
  bool isPoiMoving();
  bool isthereaPath(geometry_msgs::PoseStamped goal);
  void broadcastPoseToTF(geometry_msgs::PoseStamped p, std::string target_frame);
  
};

#endif // PEDESTRIAN_DETECTOR_FOLLOWER_H
