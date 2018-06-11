#include <follower/follower.h>
#include <string>
#include <vector>


double euclidean_distance_2d(geometry_msgs::Point p1, geometry_msgs::Point p2) {
  return std::sqrt(std::pow(p2.x-p1.x, 2) + std::pow(p2.y-p1.y, 2));
}

Follower::Follower(): 
    nh_(ros::NodeHandle()),
    nav_action_client_("/navigation_by_target", true),
    move_base_goal_action_client_("/move_base", true),
    update_trajectory_(false),
    update_navigation_goal_(false){

  ros::NodeHandle private_nh = ros::NodeHandle("~");

  if(!private_nh.param<std::string>("default_navigation_type", navigation_type_, "straight"))
    ROS_WARN("Parameter default_navigation_type not set. Using default value.");

  if(!private_nh.param<std::string>("default_navigation_stack", navigation_stack_, "move_base"))
    ROS_WARN("Parameter default_navigation_stack not set. Using default value.");

  if(!private_nh.param<std::string>("fixed_frame", fixed_frame_, "/odom"))
    ROS_WARN("Parameter fixed_frame not set. Using default value.");
  
  if(!private_nh.param<double>("path_minimum_distance", path_minimum_distance_, 0.5))
    ROS_WARN("Parameter path_minimum_distance not set. Using default value.");
  
  if(!private_nh.param<double>("target_pose_minimum_distance", target_pose_minimum_distance_, 0.5))
    ROS_WARN("Parameter target_pose_minimum_distance not set. Using default value.");
  
  if(!private_nh.param<double>("person_pose_minimum_distance", person_pose_minimum_distance_, 0.5))
    ROS_WARN("Parameter person_pose_minimum_distance not set. Using default value.");
  
  if(!private_nh.param<double>("default_head_camera_position", head_camera_position_, 1.86))
    ROS_WARN("Parameter default_head_camera_position not set. Using default value.");
  
  // subscriptions
  event_in_subscriber_ = nh_.subscribe("follower/event_in", 1, &Follower::eventInCallback, this);
  poi_position_subscriber_ = nh_.subscribe("person_position", 1, &Follower::PoiPositionCallback, this);
  robot_position_subscriber_ = nh_.subscribe("/amcl_pose", 1, &Follower::robotPoseCallBack, this);

  navigation_goal_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 2);
  head_position_publisher_ = nh_.advertise<std_msgs::UInt8MultiArray>("/cmd_head", 2);
  head_camera_position_publisher_ = nh_.advertise<std_msgs::Float64>("/head_camera_position_controller/command", 1);
  
  trajectory_publisher_ = nh_.advertise<nav_msgs::Path>("person_trajectory", 2);
  residual_trajectory_publisher_ = nh_.advertise<nav_msgs::Path>("residual_trajectory", 2);

  target_pose_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("current_navigation_target", 2);
  next_target_pose_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("next_navigation_target", 2);
  
  
  // querying parameters from parameter server TODO
  getParams();
  
  //Services
  move_base_plan_service_client_ = nh_.serviceClient<nav_msgs::GetPlan>("/move_base/GlobalPlanner/make_plan");
  move_base_clear_costmap_service_client_ = nh_.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
  
  listener_ = new (tf::TransformListener);

  // States
  following_enabled_ = true; // TODO false
  initialise_navigation_ = false;
  current_pose_pointer_ = 0;

}

Follower::~Follower(){
  poi_position_subscriber_.shutdown();
  navigation_goal_publisher_.shutdown();
}

void Follower::getParams(){
}

void Follower::eventInCallback(const std_msgs::String& msg){
  if(msg.data == "e_start"){
    following_enabled_ = true;
  }
}

void Follower::PoiPositionCallback(const geometry_msgs::PointStampedConstPtr& msg){
  poi_position_ = *msg;
  update_trajectory_ = true;
  update_navigation_goal_ = true;
}

void Follower::robotPoseCallBack(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg){
  ROS_INFO("robotPoseCallBack");
  geometry_msgs::PoseStamped robot_original_frame, asdasd;

  robot_original_frame.header = msg->header;
  robot_original_frame.pose = msg->pose.pose;

  try{
    listener_->transformPose(fixed_frame_, robot_original_frame, asdasd);
  }catch(tf2::ExtrapolationException &ex){
    ROS_WARN("%s",ex.what());
  } catch (tf::TransformException &ex) {
    ROS_WARN("%s",ex.what());
  }

  robot_pose_ = asdasd;
  
  update_navigation_goal_ = true;
  ROS_INFO("robotPoseCallBack");

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




void Follower::initialiseNavigationGoal(){

  ROS_INFO("INIT NAVIGATION");

  if (navigation_stack_ == "2d_nav"){
    ROS_INFO("Sending goal.");
    // send a goal to the action
    scout_msgs::NavigationByTargetGoal goal;
    goal.target_frame = "/people_following/current_target_pose";
    goal.heading_frame = "/people_following/current_target_pose";
    goal.proximity = person_pose_minimum_distance_;
    goal.follow_target = true;

    nav_action_client_.sendGoal(goal, 
          boost::bind(&Follower::doneCb, this, _1, _2), 
          boost::bind(&Follower::activeCb, this), 
          boost::bind(&Follower::feedbackCb, this, _1));

    //wait for the action to return
    bool finished_before_timeout = nav_action_client_.waitForResult(ros::Duration(2.0));

    if (finished_before_timeout){
      actionlib::SimpleClientGoalState state = nav_action_client_.getState();
      ROS_INFO("Action finished: %s", state.toString().c_str());
    } else {
      ROS_INFO("Action did not finish before the time out (2.0 sec).");
    }
  }


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
    path_distance = euclidean_distance_2d(poi_position_.point, last_trajectory_position.pose.position);
  }

  if(path_empty && following_enabled_ && !initialise_navigation_){
    initialise_navigation_ = true;
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
  double target_pose_distance = euclidean_distance_2d(robot_pose_.pose.position, target_pose_.pose.position);
  bool closer_target_found = false;
  unsigned long int new_current_pose_pointer;

  for(long unsigned int residual_pose_index = current_pose_pointer_; residual_pose_index < poi_trajectory_.poses.size(); residual_pose_index++){
    auto residual_pose = poi_trajectory_.poses[residual_pose_index];
    double residual_pose_distance = euclidean_distance_2d(robot_pose_.pose.position, residual_pose.pose.position);
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

      ROS_INFO("Waiting move base result");
      move_base_goal_action_client_.waitForResult();

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

  target_pose_publisher_.publish(target_pose_);
  broadcastPoseToTF(target_pose_, "/people_following/current_target_pose");

}

void Follower::updateHeadPosition(){

  geometry_msgs::Point p = relative_poi_position_.point;

  tf::StampedTransform transform;
  double roll, pitch, yaw;
  
  try{ 
    listener_->lookupTransform("/base_link", "/head_link", ros::Time(0), transform);
  }catch (tf::TransformException &ex) {
    ROS_DEBUG("%s",ex.what());
  }
  tf::Matrix3x3(transform.getBasis()).getRPY(roll, pitch, yaw);

  std_msgs::UInt8MultiArray controlMsg;
  controlMsg.data.resize(2);
  double angle = atan2(p.y,p.x);
  if(std::abs(angle - yaw) > 0.1){
    if(angle > M_PI/2){
      angle = M_PI/2;
    }else if(angle < -M_PI/2){
      angle = - M_PI/2;
    }
    float v = 1.5*(2 * std::abs(angle - yaw)/M_PI)*100;
    if(v > 100){
      v = 100;
    }

    //Speed Control
    controlMsg.data[1] = v;
    //Position control
    double angle_degrees = 90 - (180*angle)/M_PI;
    controlMsg.data[0] = angle_degrees;
  }
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

  std_msgs::Float64 head_camera_position_msg;
  head_camera_position_msg.data = head_camera_position_;
  head_camera_position_publisher_.publish(head_camera_position_msg);
  ROS_INFO("Head camera position set to %f", head_camera_position_msg.data);

  if(navigation_stack_ == "2d_nav"){
    ROS_INFO("Waiting for the 2d_nav action server to become available");
    while(ros::ok() && !nav_action_client_.waitForServer(ros::Duration(2.0))){
      ROS_WARN("Still waiting for the 2d_nav action server to become available");
    }
  } else if (navigation_stack_ == "move_base"){
    ROS_INFO("Waiting for the move_base action server to become available");
    while(ros::ok() && !move_base_goal_action_client_.waitForServer(ros::Duration(2.0))){
      ROS_WARN("Still waiting for the move_base action server to become available");
    }
  }
  ROS_INFO("Action server started.");
  
}


void Follower::loop(){

  if(following_enabled_){ // TODO enable/disable with event topics (and safety timers?)

    // TODO if person is not tracked (how long?) switch to recovery navigation
    // TODO if navigation stack unable to complete goal, update navigation stack

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


  } else {
    // TODO if navigation in progress, stop navigation
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

