#include <follower/follower.h>
#include <string>
#include <vector>


double euclidean_2d_distance(geometry_msgs::Point p1, geometry_msgs::Point p2) {
  return std::sqrt(std::pow(p2.x-p1.x, 2) + std::pow(p2.y-p1.y, 2));
}

Follower::Follower(): 
    nh_(ros::NodeHandle()),
    rod_action_client_("/navigation_by_target", true),
    move_base_goal_action_client_("/null_move_base", true),
    update_navigation_goal_(false),
    update_trajectory_(false) {
  
  ros::NodeHandle private_nh = ros::NodeHandle("~");
  
  if(!private_nh.param<std::string>("default_navigation_type", navigation_type_, "path_following"))
    ROS_WARN("Parameter default_navigation_type not set. Using default value [%s].", navigation_type_.c_str());
  
  if(!private_nh.param<std::string>("default_navigation_stack", navigation_stack_, "rod"))
    ROS_WARN("Parameter default_navigation_stack not set. Using default value [%s].", navigation_stack_.c_str());
  
  if(!private_nh.param<std::string>("fixed_frame", fixed_frame_, "/odom"))
    ROS_WARN("Parameter fixed_frame not set. Using default value [%s].", fixed_frame_.c_str());
  
  if(!private_nh.param<double>("path_minimum_distance", path_minimum_distance_, 0.5))
    ROS_WARN("Parameter path_minimum_distance not set. Using default value [%f].", path_minimum_distance_);
  
  if(!private_nh.param<double>("target_pose_minimum_distance", target_pose_minimum_distance_, 0.5))
    ROS_WARN("Parameter target_pose_minimum_distance not set. Using default value [%f].", target_pose_minimum_distance_);
  
  if(!private_nh.param<double>("person_pose_minimum_distance", person_pose_minimum_distance_, 0.5))
    ROS_WARN("Parameter person_pose_minimum_distance not set. Using default value [%f].", person_pose_minimum_distance_);
  
  if(!private_nh.param<double>("default_head_camera_position", head_camera_position_, 1.75))
    ROS_WARN("Parameter default_head_camera_position not set. Using default value [%f].", head_camera_position_);
  
  if(!private_nh.param<double>("poi_moving_radius", poi_moving_radius_, 1.0))
    ROS_WARN("Parameter poi_moving_radius_ not set. Using default value [%f].", poi_moving_radius_);
  
  double poi_lost_timeout_double;
  if(!private_nh.param<double>("poi_lost_timeout", poi_lost_timeout_double, 10.0))
    ROS_WARN("Parameter poi_lost_timeout not set. Using default value [%f].", poi_lost_timeout_double);
  poi_lost_timeout_ = ros::Duration(poi_lost_timeout_double);
  
  double poi_stopped_timeout_double;
  if(!private_nh.param<double>("poi_stopped_timeout", poi_stopped_timeout_double, 8.0))
    ROS_WARN("Parameter poi_stopped_timeout not set. Using default value [%f].", poi_stopped_timeout_double);
  poi_stopped_timeout_ = ros::Duration(poi_stopped_timeout_double);
  
  double poi_tracking_timeout_double;
  if(!private_nh.param<double>("poi_tracking_timeout", poi_tracking_timeout_double, 1.0))
    ROS_WARN("Parameter poi_tracking_timeout not set. Using default value [%f].", poi_tracking_timeout_double);
  poi_tracking_timeout_ = ros::Duration(poi_tracking_timeout_double);
  
  double success_timeout_double;
  if(!private_nh.param<double>("success_timeout", success_timeout_double, 8.0))
    ROS_WARN("Parameter success_timeout not set. Using default value [%f].", success_timeout_double);
  success_timeout_ = ros::Duration(success_timeout_double);
  
  
  if(poi_stopped_timeout_double > poi_lost_timeout_double) ROS_WARN("poi_stopped_timeout > poi_lost_timeout NOT ADVISABLE");
  
  // Subscribers
  listener_ = new (tf::TransformListener);
  event_in_subscriber_ = private_nh.subscribe("event_in", 1, &Follower::eventInCallback, this);
  
  // Publishers
  event_out_publisher_ = private_nh.advertise<std_msgs::String>("event_out", 10);
  navigation_goal_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 2);
  head_position_publisher_ = nh_.advertise<std_msgs::UInt8MultiArray>("/cmd_head", 2);
  head_camera_position_publisher_ = nh_.advertise<std_msgs::Float64>("/head_camera_position_controller/command", 1);
  trajectory_publisher_ = nh_.advertise<nav_msgs::Path>("person_trajectory", 2);
  residual_trajectory_publisher_ = nh_.advertise<nav_msgs::Path>("residual_trajectory", 2);
  target_pose_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("current_navigation_target", 2);
  next_target_pose_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("next_navigation_target", 2);
  
  // Service Clients
  move_base_plan_service_client_ = nh_.serviceClient<nav_msgs::GetPlan>("/move_base/GlobalPlanner/make_plan");
  move_base_clear_costmap_service_client_ = nh_.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
  
  // States from external events
  following_enabled_ = false;
  initialise_navigation_ = false;
  stop_navigation_ = false;
  are_subscribers_created_ = false;
  poi_position_received_ = false;
  
  current_pose_pointer_ = 0;
  
  // State from internal events
  bool is_path_completed_ = true;
  bool is_poi_in_starting_position_ = false;
  bool is_poi_close_ = false;
  bool is_poi_tracked_ = false;
  bool is_poi_lost_ = false;
  bool is_poi_stopped_ = false;
  
  ROS_INFO("\n is_poi_tracked: %s \n is_poi_close: %s \n is_poi_lost: %s  \n is_path_following_completed: %s \n is_poi_in_starting_position: %s \n is_poi_stopped: %s \n ",
        is_poi_tracked_?"T":"F",
        is_poi_close_?"T":"F",
        is_poi_lost_?"T":"F",
        is_path_completed_?"T":"F",
        is_poi_in_starting_position_?"T":"F",
        is_poi_stopped_?"T":"F"
        );
}

Follower::~Follower(){
  poi_position_subscriber_.shutdown();
  navigation_goal_publisher_.shutdown();
}


void Follower::createSubscribers(){
  if(!are_subscribers_created_){    
    poi_position_subscriber_ = nh_.subscribe("person_position", 1, &Follower::PoiPositionCallback, this);
    robot_position_subscriber_ = nh_.subscribe("/amcl_pose", 1, &Follower::robotPoseCallBack, this);
    are_subscribers_created_ = true;
  } else {
    ROS_ERROR("Subscribers already created");
  }
}

void Follower::destroySubscribers(){
  if(are_subscribers_created_){    
    poi_position_subscriber_.shutdown();
    robot_position_subscriber_.shutdown();
    are_subscribers_created_ = false;
  } else {
    ROS_ERROR("Subscribers already destroyed");
  }
}

void Follower::eventInCallback(const std_msgs::String& msg){
  
  if(msg.data == "e_start"){
  
    if(!following_enabled_){
    
      ROS_INFO("\nSTART EVENT RECEIVED: Starting following\n\n");
      
      createSubscribers();
      start_stamp_ = ros::Time::now();
      following_enabled_ = true;
      initialise_navigation_ = true;
      
    } else {
      ROS_ERROR("\nALREADY STARTED\n\n");
    }
  } else if (msg.data == "e_track_poi") {

      navigation_stack_ = "None";

      if (!following_enabled_) {

          ROS_INFO("\nSTART EVENT RECEIVED: Starting tracking person of interest (navigation stack set to None)\n\n");

          createSubscribers();
          start_stamp_ = ros::Time::now();
          following_enabled_ = true;
          initialise_navigation_ = false;

      } else {
          ROS_ERROR("\nALREADY STARTED\n\n");
      }
  }

  if(msg.data == "e_stop"){
  
    if(following_enabled_){
      
      ROS_INFO("\nSTOP EVENT RECEIVED: Stopping following\n\n");
      
      destroySubscribers();
      following_enabled_ = false;
      stop_navigation_ = true;
      
    } else {
      ROS_INFO("\nALREADY STOPPED\n\n");
    }
  }
  
}

void Follower::PoiPositionCallback(const geometry_msgs::PointStampedConstPtr& msg){
  poi_position_ = *msg;
  
  // First poi position received
  if(!poi_position_received_){
    poi_moving_position_ = poi_position_;
  }
  
  poi_position_received_ = true;
  update_trajectory_ = true;
  update_navigation_goal_ = true;
}

void Follower::robotPoseCallBack(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg){
  ROS_DEBUG("robotPoseCallBack");
  geometry_msgs::PoseStamped robot_original_frame;
  
  robot_original_frame.header = msg->header;
  robot_original_frame.pose = msg->pose.pose;
  
  try{
    listener_->transformPose(fixed_frame_, robot_original_frame, robot_pose_);
  }catch(tf2::ExtrapolationException &ex){
    ROS_WARN("%s",ex.what());
  } catch (tf::TransformException &ex) {
    ROS_WARN("%s",ex.what());
  }
  
  update_navigation_goal_ = true;

}

// Called once when the goal completes
void Follower::doneCb(const actionlib::SimpleClientGoalState& state,
            const scout_msgs::NavigationByTargetResultConstPtr& result){
  ROS_INFO("ACTION DONE  Finished in state [%s]", state.toString().c_str());
  // ROS_INFO("Answer: %i", *result);
  // ros::shutdown();
}

// Called once when the goal becomes active
void Follower::activeCb(){
  ROS_INFO("ACTION ACTIVE  Goal just went active");
}

// Called every time feedback is received for the goal
void Follower::feedbackCb(const scout_msgs::NavigationByTargetFeedbackConstPtr& feedback){
  ROS_INFO("ACTION FEEDBACK");
}

void Follower::broadcastPoseToTF(geometry_msgs::PoseStamped p, std::string target_frame){
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(p.pose.position.x, p.pose.position.y, p.pose.position.z) );
  transform.setRotation(tf::Quaternion(p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w));
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), p.header.frame_id, target_frame));
}

// DO NOT USE; FUCKS EVERYTHING UP FOR SOME REASON
bool Follower::isPoiMoving(){
  
  if(poi_trajectory_.poses.size() == 0){
    return false;
  }
  
  
  // Find the latest pose in the trajectory with distance greater or equal to poi_moving_radius_. 
  geometry_msgs::PoseStamped& last_pose = poi_trajectory_.poses[poi_trajectory_.poses.size()-1];
  geometry_msgs::PoseStamped& candidate_pose = poi_trajectory_.poses[poi_trajectory_.poses.size()-1];
  bool found_candidate_pose = false;
  
  for(int i = poi_trajectory_.poses.size()-1; i > 0; i--){
    geometry_msgs::PoseStamped& current_pose = poi_trajectory_.poses[i];
    
    if(euclidean_2d_distance(current_pose.pose.position, last_pose.pose.position) >= poi_moving_radius_){
      candidate_pose = current_pose;
      found_candidate_pose = true;
      break;
    }
    
  }
  
  // If such pose was found, and that pose was received before timeout, poi is moving
  if(!found_candidate_pose) return false;
  
  ros::Time now = ros::Time::now();
    
  return (now - candidate_pose.header.stamp) > poi_stopped_timeout_;

}

bool Follower::isPathCompleted(){
  
  long unsigned int poi_trajectory_size = poi_trajectory_.poses.size();
  
  if(poi_trajectory_size == 0){
    return false;
  }
  
  if(current_pose_pointer_ >= poi_trajectory_size){
    ROS_ERROR("current_pose_pointer_ >= poi_trajectory_size");
    return true;
  }
  
  double target_pose_distance = euclidean_2d_distance(robot_pose_.pose.position, target_pose_.pose.position);
  return (current_pose_pointer_ == poi_trajectory_.poses.size()-1) && (target_pose_distance < target_pose_minimum_distance_);
  
}

void Follower::initialiseNavigationGoal(){

  ROS_INFO("INIT NAVIGATION");
  
  if (navigation_stack_ == "rod"){
    ROS_INFO("Initialising the goal for navigation [rod]");
    
    // Send a goal to the action
    scout_msgs::NavigationByTargetGoal goal;
    goal.target_frame = "/people_following/current_target_pose";
    goal.heading_frame = "/people_following/current_target_pose";
    goal.proximity = 0.1;
    goal.follow_target = true;
    
    rod_action_client_.sendGoal(goal, 
          boost::bind(&Follower::doneCb, this, _1, _2), 
          boost::bind(&Follower::activeCb, this), 
          boost::bind(&Follower::feedbackCb, this, _1));
    
  }
}

void Follower::updateActionGoal(bool target_following = false){

  ROS_INFO("UPDATE NAVIGATION");
  
  if (navigation_stack_ == "rod"){
    ROS_INFO("Updating the goal for navigation [rod]");
    
    rod_action_client_.cancelGoal();
    
    // Send a goal to the action
    scout_msgs::NavigationByTargetGoal goal;
    
    // TODO: base turns sideways due to localisation?
    // -> happened even when localised
    
    if(target_following){
      goal.target_frame = "/people_following/poi_pose";
      goal.heading_frame = "/people_following/poi_pose";
      goal.proximity = person_pose_minimum_distance_;
    } else {
      goal.target_frame = "/people_following/current_target_pose";
      goal.heading_frame = "/people_following/current_target_pose";
      goal.proximity = 0.1;
    }
    
    goal.follow_target = true;
    
    rod_action_client_.sendGoal(goal, 
          boost::bind(&Follower::doneCb, this, _1, _2), 
          boost::bind(&Follower::activeCb, this), 
          boost::bind(&Follower::feedbackCb, this, _1));
    
  }
}

void Follower::stopNavigation(){

  ROS_INFO("Stopping navigation for stack [%s]", navigation_stack_.c_str());
  
  if (navigation_stack_ == "move_base_simple") {
    navigation_goal_publisher_.publish(robot_pose_);
  }
  
  if (navigation_stack_ == "move_base" && (move_base_goal_action_client_.getState() == actionlib::SimpleClientGoalState::PENDING || move_base_goal_action_client_.getState() == actionlib::SimpleClientGoalState::ACTIVE)) {
    move_base_goal_action_client_.cancelGoal();
  }
  
  if (navigation_stack_ == "rod" && (rod_action_client_.getState() == actionlib::SimpleClientGoalState::PENDING || rod_action_client_.getState() == actionlib::SimpleClientGoalState::ACTIVE)) {
    rod_action_client_.cancelGoal();
  }
  
  exit(0);
  
}

void Follower::updateTrajectory(){
  
  ROS_DEBUG("Absolute person position in frame [%s]: %2.3f, %2.3f",  poi_position_.header.frame_id.c_str(), poi_position_.point.x, poi_position_.point.y);
  
  std::string f = poi_position_.header.frame_id;
  if(f != fixed_frame_){
    ROS_WARN("Person position not in fixed frame");
    ROS_WARN("Absolute person position in frame [%s]: %2.3f, %2.3f",  poi_position_.header.frame_id.c_str(), poi_position_.point.x, poi_position_.point.y);
  }
  
  // Obtain the person's position in the robot's frame
  try{
    listener_->transformPoint("/base_link", poi_position_, relative_poi_position_);
    ROS_DEBUG("Relative person position in frame [%s]: %2.3f, %2.3f",  relative_poi_position_.header.frame_id.c_str(), relative_poi_position_.point.x, relative_poi_position_.point.y);  
  }catch(tf2::ExtrapolationException &ex){
    ROS_DEBUG("%s",ex.what());
  }catch (tf::TransformException &ex) {
    ROS_DEBUG("%s",ex.what());
  }
  
  // Update path and trajectory only when the person moves some distance
  double path_distance;
  bool path_empty = poi_trajectory_.poses.size() == 0;
  
  if(poi_trajectory_.poses.size() >= 1){
    geometry_msgs::PoseStamped last_trajectory_position = poi_trajectory_.poses[poi_trajectory_.poses.size()-1];
    path_distance = euclidean_2d_distance(poi_position_.point, last_trajectory_position.pose.position);
  }
  
  // This is only executed once, the first time the position of the person of interest is received
  if(path_empty && following_enabled_ && !stop_navigation_ && !initialise_navigation_){
    initialise_navigation_ = true;
    poi_starting_position_ = poi_position_;
  }
  
  if(path_empty || path_distance >= path_minimum_distance_){
  
    // Update the person's trajectory (that contains poses instead of points)
    geometry_msgs::PoseStamped poi_pose;
    
    poi_pose.header = poi_position_.header;
    poi_pose.pose.position = poi_position_.point;
    poi_pose.pose.orientation = tf::createQuaternionMsgFromYaw(atan2(-robot_pose_.pose.position.y + poi_position_.point.y, -robot_pose_.pose.position.x + poi_position_.point.x));
    
    poi_trajectory_.header = poi_position_.header;
    poi_trajectory_.poses.push_back(poi_pose);
    
    // set the pose of the previous Person Of Interest so that it points to the last one (poi_pose)
    if(poi_trajectory_.poses.size() >= 2) {
      geometry_msgs::PoseStamped& prev_poi_pose = poi_trajectory_.poses[poi_trajectory_.poses.size() - 2];
      prev_poi_pose.pose.orientation = tf::createQuaternionMsgFromYaw(atan2(poi_position_.point.y - prev_poi_pose.pose.position.y, poi_position_.point.x - prev_poi_pose.pose.position.x));
    }
    
    // Publish complete person path
    trajectory_publisher_.publish(poi_trajectory_);
  
  }

}

void Follower::updateNavigationGoal(){
  
  // TODO warning: robot_pose_ may be available only after the robot moves
  
  double poi_distance = euclidean_2d_distance(robot_pose_.pose.position, poi_position_.point);
  bool prev_is_poi_close_ = is_poi_close_;
  
  is_poi_close_ = is_poi_tracked_ && poi_distance < person_pose_minimum_distance_ + path_minimum_distance_;
  
  if( prev_is_poi_close_ != is_poi_close_ ){
    updateActionGoal(is_poi_close_);
  }
  
  geometry_msgs::PoseStamped poi_pose;
  poi_pose.header = poi_position_.header;
  poi_pose.pose.position = poi_position_.point;
  poi_pose.pose.orientation = tf::createQuaternionMsgFromYaw(atan2(robot_pose_.pose.position.y - poi_position_.point.y, robot_pose_.pose.position.x - poi_position_.point.x));
  
  broadcastPoseToTF(poi_pose, "/people_following/poi_pose");
  
  
  // target_pose_ specify the pose currently selected as goal for navigation.
  
  // The residual trajectory starts from current target and ends with the last position of the complete trajectory
  // Update the residual trajectory
  long unsigned int poi_trajectory_size = poi_trajectory_.poses.size();
  
  if(poi_trajectory_size == 0){
    ROS_DEBUG("EMPTY PATH");
    return;
  }
  
  // update current target pose and residual trajectory
  if(current_pose_pointer_ >= poi_trajectory_.poses.size()){
    ROS_ERROR("current_pose_pointer_ >= poi_trajectory_.poses.size()");
    return;
  }
  
  target_pose_ = poi_trajectory_.poses[current_pose_pointer_];
  residual_trajectory_.poses = std::vector<geometry_msgs::PoseStamped>(poi_trajectory_.poses.begin()+current_pose_pointer_, poi_trajectory_.poses.end());
  residual_trajectory_.header = poi_trajectory_.header;
  residual_trajectory_publisher_.publish(residual_trajectory_);
  // TODO get rid of integer pointer, use adressess
  // for(auto residual_pose : residual_trajectory_.poses)
  
  geometry_msgs::PoseStamped new_target_pose = target_pose_;
  next_target_pose_publisher_.publish(robot_pose_);
  double target_pose_distance = euclidean_2d_distance(robot_pose_.pose.position, target_pose_.pose.position);
  bool closer_target_found = false;
  unsigned long int new_current_pose_pointer;
  
  for(long unsigned int residual_pose_index = current_pose_pointer_; residual_pose_index < poi_trajectory_.poses.size(); residual_pose_index++){
    auto residual_pose = poi_trajectory_.poses[residual_pose_index];
    double residual_pose_distance = euclidean_2d_distance(robot_pose_.pose.position, residual_pose.pose.position);
    if(residual_pose_distance < target_pose_distance){
      closer_target_found = true;
      new_target_pose = residual_pose;
      new_current_pose_pointer = residual_pose_index;
    }
  }
  
  if(closer_target_found){
  
    geometry_msgs::PoseStamped goal;
    
    target_pose_ = new_target_pose;
    current_pose_pointer_ = new_current_pose_pointer;
    
    goal.header = poi_position_.header;
    goal.pose = target_pose_.pose;
    
    if (navigation_stack_ == "move_base_simple") {
      
      navigation_goal_publisher_.publish(goal);
    
    } else if(navigation_stack_ == "move_base"){
      
      move_base_msgs::MoveBaseGoal move_base_goal;
      move_base_goal.target_pose = goal;
      
      ROS_INFO("Sending move_base_goal");
      move_base_goal_action_client_.sendGoal(move_base_goal);
      
      if(move_base_goal_action_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        ROS_INFO("SimpleClientGoalState::SUCCEEDED");
      } else {
        ROS_WARN("SimpleClientGoalState FAILED");
      }
      
    }
    
    ROS_INFO("new trajectory navigation target: %lu\\%lu\t", current_pose_pointer_, poi_trajectory_.poses.size());
  
  }else{
    // Skip the next residual pose if it is close enough, unless it is the last
    if(target_pose_distance < target_pose_minimum_distance_){
      if(current_pose_pointer_ < poi_trajectory_.poses.size()-1) current_pose_pointer_++;
    }
  }
  
  // The path following is completed when the last pose is set as target and is close enough to be considered as reached
  bool perv_is_path_completed_ = is_path_completed_;
  is_path_completed_ = (current_pose_pointer_ == poi_trajectory_.poses.size()-1) && (target_pose_distance < target_pose_minimum_distance_);
  
  if(!perv_is_path_completed_ && is_path_completed_){
    path_completed_stamp_ = ros::Time::now();
  }
  
  target_pose_publisher_.publish(target_pose_);
  broadcastPoseToTF(target_pose_, "/people_following/current_target_pose");
  
}

void Follower::updateHeadPosition(){
  
  std_msgs::Float64 head_camera_position_msg;
  head_camera_position_msg.data = head_camera_position_;
  head_camera_position_publisher_.publish(head_camera_position_msg);
//  ROS_INFO("Head camera position set to %f", head_camera_position_msg.data);
  
  geometry_msgs::Point p, p_recent;
  
//  geometry_msgs::PointStamped poi_position_recent = poi_position_;
//  poi_position_recent.header.stamp = ros::Time::now();
  
  // TODO check if its better to not update after poi untracked
  
  // Obtain the person's position in the robot's frame
//  try{
//      listener_->transformPoint("/base_link", ros::Time(0), poi_position_, poi_position_.header.frame_id, relative_poi_position_);
//      p = relative_poi_position_.point;
//      ROS_INFO("Relative person position in frame [%s]: %2.3f, %2.3f",  relative_poi_position_.header.frame_id.c_str(), relative_poi_position_.point.x, relative_poi_position_.point.y);
//  }catch(tf2::ExtrapolationException &ex){
//      ROS_WARN("%s",ex.what());
//  }catch (tf::TransformException &ex) {
//      ROS_WARN("%s",ex.what());
//  }
  
  
  
  geometry_msgs::PointStamped poi_position_recent = poi_position_;
  poi_position_recent.header.stamp = ros::Time::now();
  
  // TODO check if its better to not update after poi untracked
  
  // Obtain the person's position in the robot's frame
  try{
      listener_->transformPoint("/base_link", ros::Time(0), poi_position_, poi_position_.header.frame_id, poi_position_recent);
      p_recent = poi_position_recent.point;
      ROS_DEBUG("Follower::updateHeadPosition(): Relative person position in frame [%s]: %2.3f, %2.3f",  poi_position_recent.header.frame_id.c_str(), poi_position_recent.point.x, poi_position_recent.point.y);
  }catch(tf2::ExtrapolationException &ex){
      ROS_WARN("%s",ex.what());
  }catch (tf::TransformException &ex) {
      ROS_WARN("%s",ex.what());
  }
  
  
  p = relative_poi_position_.point;

  tf::StampedTransform transform;
  double roll, pitch, yaw;
  
  try{ 
    listener_->lookupTransform("/base_link", "/head_link", ros::Time(0), transform);
  }catch (tf::TransformException &ex) {
    ROS_WARN("%s",ex.what());
  }
  tf::Matrix3x3(transform.getBasis()).getRPY(roll, pitch, yaw);

  std_msgs::UInt8MultiArray controlMsg;
  controlMsg.data.resize(2);
  
//  double angle_raw = atan2(p.y, p.x);
  double recent_angle_raw = atan2(p_recent.y, p_recent.x);
  double angle;
  
//  ROS_INFO("\n\n\n updateHeadPosition                      angle_raw = %2.3f       std::abs(angle_raw - yaw) = %2.3f", angle_raw, std::abs(angle_raw - yaw) );
//  ROS_INFO("angle_raw - recent_angle_raw = %2.3f", angle_raw - recent_angle_raw);
  
  if(!is_poi_tracked_ && is_path_completed_){
    
    angle = 0;
    
  } else if (std::abs(recent_angle_raw - yaw) > 0.1) {
    
    angle = recent_angle_raw;
    if(angle > M_PI/2 - 0.1){
      angle = M_PI/2 - 0.1;
    }else if(angle < -M_PI/2 + 0.1){
      angle = -M_PI/2 + 0.1;
    }
    
  } else if (!is_poi_tracked_) { // TODO: is this better than just return?
    
    angle = recent_angle_raw;
    if(angle > M_PI/2 - 0.1){
      angle = M_PI/2 - 0.1;
    }else if(angle < -M_PI/2 + 0.1){
      angle = -M_PI/2 + 0.1;
    }
  
  } else {
    
    return;
  }
  
  float v = 300.0;
  //Speed Control
  controlMsg.data[1] = v;
  //Position control
  double angle_degrees = 90 - (180*angle)/M_PI;
  controlMsg.data[0] = angle_degrees;
//  }
  head_position_publisher_.publish(controlMsg);
  
}

bool Follower::isthereaPath(geometry_msgs::PoseStamped goal){
  nav_msgs::GetPlan srv;
  srv.request.start = robot_pose_;
  srv.request.goal = goal;
  if (!move_base_plan_service_client_.call(srv)){
    ROS_ERROR("Failed to call service make_plan");
    return false;
  }else{
    if(srv.response.plan.poses.size() > 0){
      std::cout << "plan" << std::endl;
      return true;
    }else{
      std::cout << "no plan" << std::endl;
      return false;
    }
  }
}

void Follower::clearCostmaps(){
  std_srvs::Empty srv;
  if (!move_base_clear_costmap_service_client_.call(srv)){
    ROS_ERROR("Failed to Clear Costmaps");
  }
}

void Follower::reset(){

  ROS_INFO("RESET");
  ROS_INFO("navigation_type: %s", navigation_type_.c_str());
  ROS_INFO("navigation_stack: %s", navigation_stack_.c_str());

  // TODO check for all action clients and cancel goals if needed

  if(navigation_stack_ == "rod"){
    ROS_INFO("Waiting for the rod action server to become available");
    while(ros::ok() && !rod_action_client_.waitForServer(ros::Duration(3.0))){
      ROS_WARN("Still waiting for the rod action server to become available");
    }
  } else if (navigation_stack_ == "move_base"){
    ROS_INFO("Waiting for the move_base action server to become available");
    while(ros::ok() && !move_base_goal_action_client_.waitForServer(ros::Duration(3.0))){
      ROS_WARN("Still waiting for the move_base action server to become available");
    }
  }
  ROS_INFO("Action server started.");
  
}


void Follower::loop(){
  
  if(stop_navigation_ || e_failure_){
    stopNavigation();
    stop_navigation_ = false;
  }

  if(following_enabled_ && !stop_navigation_ && !e_failure_){

    // TODO if rod navigation stack with path_following is unable to navigate, switch to move_base with person following
    
    if(update_trajectory_) {
      updateTrajectory();
      update_trajectory_ = false;
    }
    
    if(update_navigation_goal_){
      updateNavigationGoal();
      update_navigation_goal_ = false;
    }

    updateHeadPosition();

    if(initialise_navigation_){
      initialiseNavigationGoal();
      initialise_navigation_ = false;
    }

    // Update internal state variables
    
    bool prev_is_poi_in_starting_position_ = is_poi_in_starting_position_;
    bool prev_is_poi_stopped_ = is_poi_stopped_;
    bool prev_is_poi_lost_ = is_poi_lost_;
    bool prev_is_poi_tracked_ = is_poi_tracked_;
    
    /*
    ros::Time now = ros::Time::now();
    ros::Duration untracked_time = now - poi_position_.header.stamp;
    is_poi_tracked_ = untracked_time < poi_tracking_timeout_;
    
    double distance_from_starting_position = euclidean_2d_distance(poi_starting_position_.point, poi_position_.point);
    is_poi_in_starting_position_ = is_poi_tracked_ && (distance_from_starting_position < poi_moving_radius_);
    
    if(!(is_poi_in_starting_position_ && is_poi_tracked_) && euclidean_2d_distance(poi_moving_position_.point, poi_position_.point) > poi_moving_radius_)
      poi_moving_position_ = poi_position_;

    is_poi_stopped_ = is_poi_tracked_ && !is_poi_in_starting_position_ && (now - poi_moving_position_.header.stamp > poi_stopped_timeout_);
    is_poi_lost_ = !is_poi_stopped_ && !(is_poi_in_starting_position_ && is_poi_tracked_) && is_path_completed_ && !is_poi_tracked_ && (now - path_following_completed_stamp_ > poi_lost_timeout_);
    
    */
    
    std_msgs::String e;
    ros::Time now = ros::Time::now();    
    ros::Duration untracked_time = now - poi_position_.header.stamp;
    ros::Duration stopped_time = now - poi_moving_position_.header.stamp;
    ros::Duration path_completed_time = now - path_completed_stamp_;
    double distance_from_starting_position = euclidean_2d_distance(poi_starting_position_.point, poi_position_.point);
    
    // becomes true the first time the poi position is received, and stays true only until the timeout expires after the last time the position has been received.
    is_poi_tracked_ = poi_position_received_ && (untracked_time < poi_tracking_timeout_);
    
    // Whether the poi is in a circle of radious equal to the parameter poi_moving_radius_. Is false until the poi is tracked, hence until the first time its position is received. 
    is_poi_in_starting_position_ = is_poi_tracked_ && (distance_from_starting_position < poi_moving_radius_);
    
    // Poi is considered stopped only if it is tracked (otherwise it is unknown), if it is not in the starting position (stopped is considered a terminal state of the following), and in the stamp of the last movement is older than poi_stopped_timeout_.
    is_poi_stopped_ = is_poi_tracked_ && !is_poi_in_starting_position_ && (stopped_time > poi_stopped_timeout_);
    
    // Poi is considered lost only if it is untracked, but after its position has been received at least once, if it is not in the starting position, if the robot has completed the path (there is no other way to go) for a time greater than poi_lost_timeout_.
    is_poi_lost_ = poi_position_received_ && !is_poi_tracked_ && !is_poi_in_starting_position_ && is_path_completed_ && (path_completed_time > poi_lost_timeout_);
    
    
    
    // (e_success_ || e_failure_) can only become true once. 0 <= e_success_ + e_failure_ <= 1. Either when the success_timeout_ expires, sendind event e_failure, or when the poi is tracked the first time.
    if(!(e_success_ || e_failure_)){
      if(is_poi_in_starting_position_) {
        e_success_ = true;
        e.data = "e_success";
        event_out_publisher_.publish(e);
      } else if (now - start_stamp_ > success_timeout_) {
        e_failure_ = true;
        e.data = "e_failure";
        event_out_publisher_.publish(e);
        return;
      }
    }
    
//    // When is_poi_in_starting_position_ becomes false
//    if(prev_is_poi_in_starting_position_ && !is_poi_in_starting_position_) {
//      e.data = "e_left_starting_position";
//      event_out_publisher_.publish(e);
//    }
    
    // When is_poi_stopped_ becomes true
    if(!prev_is_poi_stopped_ && is_poi_stopped_) {
      e.data = "e_timeout";
      event_out_publisher_.publish(e);
    }
    
    // When is_poi_lost_ becomes true
    if(!prev_is_poi_lost_ && is_poi_lost_) {
      e.data = "e_lost";
      event_out_publisher_.publish(e);
    }
    
    // When is_poi_tracked_ changes
    if(prev_is_poi_tracked_ != is_poi_tracked_) {
      e.data = is_poi_tracked_ ? "e_tracking_acquired" : "e_tracking_lost";
      event_out_publisher_.publish(e);
    }
    
    if(
      (!(e_success_ || e_failure_) && is_poi_in_starting_position_)||
      (!(e_success_ || e_failure_) && now - start_stamp_ > success_timeout_)||
      (prev_is_poi_in_starting_position_ != is_poi_in_starting_position_ && is_poi_in_starting_position_)||
      (prev_is_poi_stopped_ != is_poi_stopped_ && is_poi_stopped_)||
      (prev_is_poi_lost_ != is_poi_lost_ && is_poi_lost_)||
      (prev_is_poi_tracked_ != is_poi_tracked_)
    ) {
    
        ROS_INFO("\n untracked_time: %f \n is_poi_tracked: %s \n is_poi_close: %s \n is_poi_lost: %s  \n is_path_following_completed: %s \n distance_from_starting_position: %f \n is_poi_in_starting_position: %s \n is_poi_stopped: %s \n ",
            untracked_time.toSec(),
            is_poi_tracked_?"T":"F",
            is_poi_close_?"T":"F",
            is_poi_lost_?"T":"F",
            is_path_completed_?"T":"F",
            distance_from_starting_position,
            is_poi_in_starting_position_?"T":"F",
            is_poi_stopped_?"T":"F"
        );
    
    }
    
    
    
//    ROS_INFO("trajectory navigation target: %lu\\%lu\t", current_pose_pointer_, poi_trajectory_.poses.size());
    /*
    */
    
  }
  
}


int main(int argc, char** argv){
  ros::init(argc, argv, "follower");
  
  Follower follower;

  double node_frequency = 50;
  ros::Rate loop_rate(node_frequency);

  follower.reset();

  while(ros::ok()){
    ros::spinOnce();
    follower.loop();
    loop_rate.sleep();
  }
  
  return 0;
}

