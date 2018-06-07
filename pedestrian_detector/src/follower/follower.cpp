#include <follower/follower.h>
#include <string>
#include <vector>


double euclidean_distance_2d(geometry_msgs::Point p1, geometry_msgs::Point p2) {
  return std::sqrt(std::pow(p2.x-p1.x, 2) + std::pow(p2.y-p1.y, 2));
}

Follower::Follower(): nh_(ros::NodeHandle()), new_person_position_received_(false), nav_action_client_("/navigation_by_target", true){

  ros::NodeHandle private_nh = ros::NodeHandle("~");

  if(!private_nh.param<std::string>("default_navigation_type", navigation_type_, "straight"))
    ROS_WARN("Parameter default_navigation_type not set. Using default value.");


  if(!private_nh.param<std::string>("default_navigation_stack", navigation_stack_, "move_base"))
    ROS_WARN("Parameter default_navigation_stack not set. Using default value.");

  if(!private_nh.param<double>("path_minimum_distance", path_minimum_distance_, 0.5))
    ROS_WARN("Parameter path_minimum_distance not set. Using default value.");
  
  
  // subscriptions
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
  
  // Actions
  // nav_action_client_ = actionlib::SimpleActionClient<scout_msgs::NavigationByTargetAction>("/navigation_by_target", true);


  listener_ = new (tf::TransformListener);

  // States
  following_enabled_ = true; // TODO

  current_pose_pointer_ = 0;

}

Follower::~Follower(){
  filtered_person_position_subscriber_.shutdown();
  //pub_event_in_.shutdown();
  pub_pose_in_.shutdown();
}

void Follower::getParams(){
}

void Follower::filteredPersonPositionCallback(const geometry_msgs::PointStampedConstPtr& msg){
  

  // Save the position as point, in its original frame
  filtered_person_position_ = *msg;

  ROS_INFO("Absolute person position in frame [%s]: %2.3f, %2.3f",  filtered_person_position_.header.frame_id.c_str(), filtered_person_position_.point.x, filtered_person_position_.point.y);

  if(filtered_person_position_.header.frame_id != "/map")
    ROS_WARN("Person position not in fixed frame");
  
  // Obtain the person's position in the robot's frame
  // TODO wrap exception
  try{
    listener_->transformPoint("/base_link", filtered_person_position_, relative_person_position_);
  }catch(tf2::ExtrapolationException &ex){
    ROS_WARN("%s",ex.what());
  }catch (tf::TransformException &ex) {
    ROS_WARN("%s",ex.what());
  }

  ROS_INFO("Relative person position in frame [%s]: %2.3f, %2.3f",  relative_person_position_.header.frame_id.c_str(), relative_person_position_.point.x, relative_person_position_.point.y);  

  // Update path and trajectory only when the person moves some distance

  double path_distance;
  bool path_empty = complete_person_path_.size() == 0;

  if(complete_person_path_.size() >= 1){
    geometry_msgs::PointStamped last_path_position = complete_person_path_[complete_person_path_.size()-1];
    // path_empty = false;
    path_distance = std::sqrt(std::pow(filtered_person_position_.point.x-last_path_position.point.x, 2) + std::pow(filtered_person_position_.point.y-last_path_position.point.y, 2));
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

    complete_person_trajectory_.header = filtered_person_position_.header;
    complete_person_trajectory_.poses.push_back(filtered_person_pose);

    // TODO move
    // Publish complete person path
    trajectory_publisher_.publish(complete_person_trajectory_);

  }

  new_person_position_received_ = true;

}

void Follower::robotPoseCallBack(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg){
  current_robot_position_.header = msg->header;
  current_robot_position_.pose = msg->pose.pose;
}

void Follower::updateNavigationGoal(){

  double x = relative_person_position_.point.x;
  double y = relative_person_position_.point.y;
  double z = relative_person_position_.point.z;

  // Direct navigation
  if(navigation_type_ == "straight"){
    double d = 0.6;
    double final_x = d*x;
    double final_y = d*y;

    goal_.header = filtered_person_position_.header;
    goal_.pose.position.x = final_x;
    goal_.pose.position.y = final_y;
    goal_.pose.position.z = z;
    goal_.pose.orientation = tf::createQuaternionMsgFromYaw(atan2(final_y,final_x));
    
  } else if(navigation_type_ == "path_following"){
    
    ROS_INFO("NAVIGATION: path_following");

    // Current_target_pose specify the pose currently selected as goal for navigation.

    // The residual trajectory is always between the current target and the latest position in the trajectory
    // Update the residual trajectory
    long unsigned int complete_trajectory_size = complete_person_trajectory_.poses.size();
    // std::vector<geometry_msgs::PoseStamped>::iterator last_iter = complete_person_trajectory_.poses.back(); 
    

    if(complete_trajectory_size == 0){
      ROS_INFO("EMPTY PATH");
    } else {
      
      ROS_INFO("current_pose_pointer_: %lu\tcomplete_person_trajectory_ size: %lu\t", current_pose_pointer_, complete_person_trajectory_.poses.size());

      // update current target pose and residual trajectory
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

      for(int residual_pose_index = current_pose_pointer_; residual_pose_index < complete_person_trajectory_.poses.size(); residual_pose_index++){
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

      }

      current_target_pose_publisher_.publish(current_target_pose_);

    }

  } else if(navigation_type_ == "recovery"){
    // TODO look for a person where the other navigation types may have lost the tracking 
  }


  if(navigation_stack_ == "move_base"){
    // Clear the person's trace in the cost map to help the planner find a path that "trails" the person.
    clearCostmaps();
    // TODO check path
  } else if (navigation_stack_ == "2d_nav"){
    // TODO check on the action feedback
  }

  
  // TODO remove unused
  //bool path = isthereaPath(goal_);
  // if (path){
  //   std::cout << "there is path" << std::endl;
  //   event_out_msg_ = goal_;
  //   pointGoal_ = filtered_person_position_.point;
  //   //personPoses_.push_back(goal_) = ;
  //   keptPose_ = goal_;
  //   keptPoint_ = filtered_person_position_.point;
  // }else{
  //   std::cout << "there is no path" << std::endl;
  //   event_out_msg_ = keptPose_;
  //   //pointGoal_ = keptPoint_;
  //   keptPoint_ = filtered_person_position_.point;
  // }
  //pub_event_in_.publish(stop_);
  

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

  if (navigation_stack_ == "2d_nav"){
    ROS_INFO("Action server started, sending goal.");
    // send a goal to the action
    scout_msgs::NavigationByTargetGoal goal;
    goal.target_frame = "/people_following/current_target_pose";
    goal.heading_frame = "/people_following/current_target_pose";
    goal.proximity = 0.5;
    goal.follow_target = true;

    nav_action_client_.sendGoal(goal);

    //wait for the action to return
    bool finished_before_timeout = nav_action_client_.waitForResult(ros::Duration(5.0));

    if (finished_before_timeout)
    {
      actionlib::SimpleClientGoalState state = nav_action_client_.getState();
      ROS_INFO("Action finished: %s",state.toString().c_str());
    }
    else{
      ROS_INFO("Action did not finish before the time out.");
    }
  }



}


void Follower::loop(){

  // if(follower.new_person_position_received_){
  //   follower.clearCostmaps(); // TODO removed, check if needed?
  // }

  if(following_enabled_){ // TODO enable/disable with event topics (and safety timers?)

    // TODO if person is not tracked (how long?) switch to recovery navigation
    // TODO if navigation stack unable to complete goal, update navigation stack

    // Otherwise, update navigation goal and head position. 
    updateNavigationGoal();
    updateHeadPosition();
    if(new_person_position_received_) {
      

      new_person_position_received_ = false;

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

