#include <follower/follower.h>
#include <string>
#include <vector>


double euclidean_distance_2d(geometry_msgs::Point p1, geometry_msgs::Point p2) {
  return std::sqrt(std::pow(p2.x-p1.x, 2) + std::pow(p2.y-p1.y, 2));
}

Follower::Follower(): nh_(ros::NodeHandle()), nav_action_client_("/navigation_by_target", true), update_trajectory_(false), update_navigation_goal_(false){

  ros::NodeHandle private_nh = ros::NodeHandle("~");

  if(!private_nh.param<std::string>("default_navigation_type", navigation_type_, "straight"))
    ROS_WARN("Parameter default_navigation_type not set. Using default value.");

  if(!private_nh.param<std::string>("default_navigation_stack", navigation_stack_, "move_base"))
    ROS_WARN("Parameter default_navigation_stack not set. Using default value.");

  if(!private_nh.param<double>("path_minimum_distance", path_minimum_distance_, 0.5))
    ROS_WARN("Parameter path_minimum_distance not set. Using default value.");
  
  if(!private_nh.param<double>("target_pose_minimum_distance", target_pose_minimum_distance_, 0.5))
    ROS_WARN("Parameter target_pose_minimum_distance not set. Using default value.");
  
  if(!private_nh.param<double>("person_pose_minimum_distance", person_pose_minimum_distance_, 0.5))
    ROS_WARN("Parameter person_pose_minimum_distance not set. Using default value.");
  
  // subscriptions
  event_in_subscriber_ = nh_.subscribe("follower/event_in", 1, &Follower::eventInCallback, this);
  filtered_person_position_subscriber_ = nh_.subscribe("person_position", 1, &Follower::filteredPersonPositionCallback, this);
  robot_position_subscriber_ = nh_.subscribe("/amcl_pose", 1, &Follower::robotPoseCallBack, this);

  //pub_event_in_ = nh_.advertise<std_msgs::String>("/move_base_wrapper/event_in",2);
  pub_pose_in_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 2);
  pub_head_rot_ = nh_.advertise<std_msgs::UInt8MultiArray>("/cmd_head", 2);
  
  trajectory_publisher_ = nh_.advertise<nav_msgs::Path>("person_trajectory", 2);
  residual_trajectory_publisher_ = nh_.advertise<nav_msgs::Path>("residual_trajectory", 2);

  current_target_pose_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("current_navigation_target", 2);
  next_target_pose_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("next_navigation_target", 2);
  
  // querying parameters from parameter server TODO
  getParams();
  
  //Services
  plan_client_ = nh_.serviceClient<nav_msgs::GetPlan>("/move_base/GlobalPlanner/make_plan");
  clear_client_ = nh_.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
  
  listener_ = new (tf::TransformListener);

  // States
  following_enabled_ = true; // TODO false
  initialise_navigation_ = false;
  current_pose_pointer_ = 0;

}

Follower::~Follower(){
  filtered_person_position_subscriber_.shutdown();
  pub_pose_in_.shutdown();
}

void Follower::getParams(){
}

void Follower::eventInCallback(const std_msgs::String& msg){
  if(msg.data == "e_start"){
    following_enabled_ = true;
  }
}

void Follower::filteredPersonPositionCallback(const geometry_msgs::PointStampedConstPtr& msg){
  filtered_person_position_ = *msg;
  update_trajectory_ = true;
  update_navigation_goal_ = true;
}

void Follower::robotPoseCallBack(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg){
  current_robot_position_.header = msg->header;
  current_robot_position_.pose = msg->pose.pose;
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
    bool finished_before_timeout = nav_action_client_.waitForResult(ros::Duration(5.0));

    if (finished_before_timeout)
    {
      actionlib::SimpleClientGoalState state = nav_action_client_.getState();
      ROS_INFO("Action finished: %s", state.toString().c_str());
    }
    else{
      ROS_INFO("Action did not finish before the time out.");
    }
  }


}

void Follower::updateTrajectory(){

  ROS_DEBUG("Absolute person position in frame [%s]: %2.3f, %2.3f",  filtered_person_position_.header.frame_id.c_str(), filtered_person_position_.point.x, filtered_person_position_.point.y);

  std::string f = filtered_person_position_.header.frame_id;
  if(f != "/map" && f != "map" && f != "/odom" && f != "odom"){
    ROS_WARN("Person position not in fixed frame");
    ROS_WARN("Absolute person position in frame [%s]: %2.3f, %2.3f",  filtered_person_position_.header.frame_id.c_str(), filtered_person_position_.point.x, filtered_person_position_.point.y);
  }
  
  // Obtain the person's position in the robot's frame
  try{
    listener_->transformPoint("/base_link", filtered_person_position_, relative_person_position_);
    ROS_DEBUG("Relative person position in frame [%s]: %2.3f, %2.3f",  relative_person_position_.header.frame_id.c_str(), relative_person_position_.point.x, relative_person_position_.point.y);  
  }catch(tf2::ExtrapolationException &ex){
    ROS_DEBUG("%s",ex.what());
  }catch (tf::TransformException &ex) {
    ROS_DEBUG("%s",ex.what());
  }
  
  // Update path and trajectory only when the person moves some distance

  double path_distance;
  bool path_empty = complete_person_path_.size() == 0;

  if(complete_person_path_.size() >= 1){
    geometry_msgs::PointStamped last_path_position = complete_person_path_[complete_person_path_.size()-1];
    path_distance = std::sqrt(std::pow(filtered_person_position_.point.x-last_path_position.point.x, 2) + std::pow(filtered_person_position_.point.y-last_path_position.point.y, 2));
  }

  if(path_empty && following_enabled_ && !initialise_navigation_){
    initialise_navigation_ = true;
  }

  if(path_empty || path_distance >= path_minimum_distance_){

    // Update the person's path
    complete_person_path_.push_back(filtered_person_position_);

    // Only the time that the path contains only one pose, set the current pose target as the first (and only) pose
    // if(complete_person_path_.size() <= 1) current_target_pose_ = complete_person_trajectory_.poses.begin();

    // Update the person's trajectory (that contains poses instead of points)
    geometry_msgs::PoseStamped filtered_person_pose;
    
    filtered_person_pose.header = filtered_person_position_.header;
    filtered_person_pose.pose.position = filtered_person_position_.point;
    filtered_person_pose.pose.orientation.x = 0.0;
    filtered_person_pose.pose.orientation.y = 0.0;
    filtered_person_pose.pose.orientation.z = 0.0;
    filtered_person_pose.pose.orientation.w = 1.0;

    complete_person_trajectory_.header = filtered_person_position_.header;
    complete_person_trajectory_.poses.push_back(filtered_person_pose);

    // TODO move
    // Publish complete person path
    trajectory_publisher_.publish(complete_person_trajectory_);

  }

}

void Follower::updateNavigationGoal(){
  
  // Current_target_pose specify the pose currently selected as goal for navigation.

  // The residual trajectory starts from current target and ends with the last position of the complete trajectory
  // Update the residual trajectory
  long unsigned int complete_trajectory_size = complete_person_trajectory_.poses.size();

  if(complete_trajectory_size == 0){
    ROS_DEBUG("EMPTY PATH");
  } else {
    

    // update current target pose and residual trajectory
    if(current_pose_pointer_ >= complete_person_trajectory_.poses.size()){
      ROS_ERROR("current_pose_pointer_ >= complete_person_trajectory_.poses.size()");
      return;
    }

    current_target_pose_ = complete_person_trajectory_.poses[current_pose_pointer_];
    residual_trajectory_.poses = std::vector<geometry_msgs::PoseStamped>(complete_person_trajectory_.poses.begin()+current_pose_pointer_, complete_person_trajectory_.poses.end());
    residual_trajectory_.header = complete_person_trajectory_.header;
    residual_trajectory_publisher_.publish(residual_trajectory_);


    // TODO get rid of integer pointer, use adressess
    // for(auto residual_pose : residual_trajectory_.poses)

    geometry_msgs::PoseStamped new_target_pose = current_target_pose_;
    double current_target_pose_distance = euclidean_distance_2d(current_robot_position_.pose.position, current_target_pose_.pose.position);
    bool closer_target_found = false;
    unsigned long int new_current_pose_pointer;

    for(long unsigned int residual_pose_index = current_pose_pointer_; residual_pose_index < complete_person_trajectory_.poses.size(); residual_pose_index++){
      auto residual_pose = complete_person_trajectory_.poses[residual_pose_index];
      double residual_pose_distance = euclidean_distance_2d(current_robot_position_.pose.position, residual_pose.pose.position);
      if(residual_pose_distance < current_target_pose_distance){
        closer_target_found = true;
        new_target_pose = residual_pose;
        new_current_pose_pointer = residual_pose_index;
      }
    }

    if(closer_target_found){


      current_target_pose_ = new_target_pose;
      current_pose_pointer_ = new_current_pose_pointer;
      
      goal_.header = filtered_person_position_.header;
      goal_.pose = current_target_pose_.pose;
      // TODO goal_.pose.orientation = tf::createQuaternionMsgFromYaw(atan2(-current_target_pose.pose.position.y + next_target_pose.pose.position.y , -current_target_pose.pose.position.x + next_target_pose.pose.position.x));
      if (navigation_stack_ == "2d_nav_topic") pub_pose_in_.publish(goal_);

      ROS_INFO("new trajectory navigation target: %lu\\%lu\t", current_pose_pointer_, complete_person_trajectory_.poses.size());

    }else{
      // Skip the next residual pose if it is close enough, unless it is the last
      if(current_target_pose_distance < target_pose_minimum_distance_){
        if(current_pose_pointer_ < complete_person_trajectory_.poses.size()-1) current_pose_pointer_++;
      }
    }

    current_target_pose_publisher_.publish(current_target_pose_);
    broadcastPoseToTF(current_target_pose_, "/people_following/current_target_pose");

  }

}

void Follower::updateHeadPosition(){

  geometry_msgs::Point p = relative_person_position_.point;

  tf::StampedTransform transform;
  double roll, pitch, yaw;
  
  try{ 
    listener_->lookupTransform("/base_link", "/head_link", ros::Time(0), transform);
  }catch (tf::TransformException &ex) {
    ROS_ERROR("%s",ex.what());
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
  pub_head_rot_.publish(controlMsg);
}

bool Follower::isthereaPath(geometry_msgs::PoseStamped goal){
  nav_msgs::GetPlan srv;
  srv.request.start = current_robot_position_;
  srv.request.goal = goal;
  if (!plan_client_.call(srv)){
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
  if (!clear_client_.call(srv)){
    ROS_ERROR("Failed to Clear Costmaps");
  }
}

void Follower::setup(){
  ROS_INFO("navigation_type: %s", navigation_type_.c_str());
  ROS_INFO("navigation_stack: %s", navigation_stack_.c_str());

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  nav_action_client_.waitForServer(); //will wait for infinite time
  ROS_INFO("Action server started.");

}


void Follower::loop(){

  // if(follower.update_trajectory_){
  //   follower.clearCostmaps(); // TODO removed, check if needed?
  // }

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
  ros::init(argc, argv, "Follower");
  
  Follower follower;

  double node_frequency = 50;
  ros::Rate loop_rate(node_frequency);

  follower.setup();

  while(ros::ok()){
    
    ros::spinOnce();
    follower.loop();
    loop_rate.sleep();

  }
  
  return 0;
}

