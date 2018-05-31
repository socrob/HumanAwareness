#include <follower/follower.h>
#include <string>
#include <vector>

Follower::Follower():new_person_position_received_(false), nh_(ros::NodeHandle()){

  nh_.param<std::string>("default_navigation_type", navigation_type_, "straight");
  nh_.param<std::string>("default_navigation_stack", navigation_stack_, "move_base");

  // subscriptions
  filtered_person_position_subscriber_ = nh_.subscribe("person_position",1,&Follower::eventInCallBack,this);
  sub_robot_pose_ = nh_.subscribe("/amcl_pose",1,&Follower::robotPoseCallBack,this);

  //pub_event_in_ = nh_.advertise<std_msgs::String>("/move_base_wrapper/event_in",2);
  pub_pose_in_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",2);
  pub_head_rot_ = nh_.advertise<std_msgs::UInt8MultiArray>("/cmd_head",2);
  
  // querying parameters from parameter server TODO
  getParams();
  
  //Services
  plan_client_ = nh_.serviceClient<nav_msgs::GetPlan>("/move_base/GlobalPlanner/make_plan");
  clear_client_ = nh_.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
  
  //start_.data = "e_start";
  //stop_.data = "e_stop";
  listener_ = new (tf::TransformListener);

  // States
  following_enabled_ = true; // TODO

}

Follower::~Follower(){
  filtered_person_position_subscriber_.shutdown();
  //pub_event_in_.shutdown();
  pub_pose_in_.shutdown();
}

void Follower::getParams(){
}

void Follower::eventInCallBack(const geometry_msgs::PointStampedConstPtr& msg){
  filtered_person_position_ = *msg;

  // TODO wrap exception
  listener_->transformPoint("base_link", filtered_person_position_, relative_person_position_);

  ROS_INFO("Absolute person position in frame [%s]: %2.3f, %2.3f",  filtered_person_position_.header.frame_id.c_str(), filtered_person_position_.point.x, filtered_person_position_.point.y);
  ROS_INFO("Relative person position in frame [%s]: %2.3f, %2.3f",  relative_person_position_.header.frame_id.c_str(), relative_person_position_.point.x, relative_person_position_.point.y);

  new_person_position_received_ = true;

}

void Follower::robotPoseCallBack(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg){
  start_.header = msg->header;
  start_.pose = msg->pose.pose;
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
    // TODO interpolate person trajectory and get point 
  } else if(navigation_type_ == "recovery"){
    // TODO look for a person where the other navigation types may have lost the tracking 
  }


  if(navigation_stack_ == "move_base"){
    // Clear the person's trace in the cost map to help the planner find a path that "trails" the person.
    clearCostmaps();
    // TODO check path
  } else if (navigation_stack_ == "2d_nav"){
    // TODO
  }

  // nav_action update goal
  pub_pose_in_.publish(goal_);

  
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
  srv.request.start = start_;
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

void Follower::loop(){

  // if(follower.new_person_position_received_){
  //   follower.clearCostmaps(); // TODO removed, check if needed?
  // }

  if(following_enabled_){ // TODO enable/disable with event topics (and safety timers?)

    // TODO if person is not tracked (how long?) switch to recovery navigation
    // TODO if navigation stack unable to complete goal, update navigation stack

    // Otherwise, update navigation goal and head position. 
    if(new_person_position_received_) {
      
      updateNavigationGoal();
      updateHeadPosition();

      new_person_position_received_ = false;

    }

  } else {
    // TODO if navigation in progress, stop navigation
  }
}


int main(int argc, char** argv){
  ros::init(argc, argv, "Follower");
  
  // create object of the node class
  Follower follower;
  // setup node frequency
  double node_frequency = 50;

  ros::Rate loop_rate(node_frequency);

  while(ros::ok()){
    
    ros::spinOnce();

    // TODO move everything to loop function
    follower.loop();
    
    loop_rate.sleep();
  }
  
  return 0;
}

