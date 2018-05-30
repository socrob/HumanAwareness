#include <follower/follower.h>
#include <string>
#include <vector>

Follower::Follower():is_event_in_received_(false), nh_(ros::NodeHandle()){
  // subscriptions
  sub_event_in_ = nh_.subscribe("person_position",1,&Follower::eventInCallBack,this);
  sub_robot_pose_ = nh_.subscribe("/amcl_pose",1,&Follower::robotPoseCallBack,this);

  //pub_event_in_ = nh_.advertise<std_msgs::String>("/move_base_wrapper/event_in",2);
  pub_pose_in_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",2);
  pub_head_rot_ = nh_.advertise<std_msgs::UInt8MultiArray>("/cmd_head",2);
  // querying parameters from parameter server
  getParams();
  //Services
  plan_client_ = nh_.serviceClient<nav_msgs::GetPlan>("/move_base/GlobalPlanner/make_plan");
  clear_client_ = nh_.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
  //start_.data = "e_start";
  //stop_.data = "e_stop";
  listener_ = new (tf::TransformListener);
}

Follower::~Follower(){
  sub_event_in_.shutdown();
  //pub_event_in_.shutdown();
  pub_pose_in_.shutdown();
}

void Follower::getParams(){
}

void Follower::eventInCallBack(const geometry_msgs::PointStampedConstPtr& msg){
  person_position_msg_ = *msg;
  is_event_in_received_ = true;
}

void Follower::robotPoseCallBack(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg){
  start_.header = msg->header;
  start_.pose = msg->pose.pose;
}

void Follower::update(){
  // listen to callbacks
  ros::spinOnce();
  
  if(!is_event_in_received_) return;

  // reset flag
  is_event_in_received_ = false;

  Stamped< tf::Point > person_base_link;

  tf::transformPoint	(
    "base_link", // const std::string & 	target_frame
    person_position_msg_, // const Stamped< tf::Point > & 	stamped_in,
    person_base_link // Stamped< tf::Point > & 	stamped_out
)	;


  double x = person_position_msg_.point.x;
  double y = person_position_msg_.point.y;
  double z = person_position_msg_.point.z;
  
  double vx = x;
  double vy = y;
  //double norm_v = abs(sqrt(x*x + y*y));
  //double ux = vx/norm_v;
  //double uy = vy/norm_v;
  double d = 0.6;
  double final_x = d*vx;
  double final_y = d*vy;

  goal_.header = person_position_msg_.header;

  ROS_INFO_STREAM("Received person position in frame [" << goal_.header.frame_id << "] ");

  goal_.pose.position.x = final_x;
  goal_.pose.position.y = final_y;
  goal_.pose.position.z = z;
  goal_.pose.orientation = tf::createQuaternionMsgFromYaw(atan2(final_y,final_x));
  
  //bool path = isthereaPath(goal_);
  
  // if (path){
  //   std::cout << "there is path" << std::endl;
  //   event_out_msg_ = goal_;
  //   pointGoal_ = person_position_msg_.point;
  //   //personPoses_.push_back(goal_) = ;
  //   keptPose_ = goal_;
  //   keptPoint_ = person_position_msg_.point;
  // }else{
  //   std::cout << "there is no path" << std::endl;
  //   event_out_msg_ = keptPose_;
  //   //pointGoal_ = keptPoint_;
  //   keptPoint_ = person_position_msg_.point;
  // }
  
  
  //pub_event_in_.publish(stop_);
  clearCostmaps();
  std::cout << "sending goal" << std::endl;
  //rotateHead(person_position_msg_.point);
  pub_pose_in_.publish(goal_);
  //pub_event_in_.publish(start_);

}

void Follower::update_head(){
  // listen to callbacks
  ros::spinOnce();

  if(!is_event_in_received_) return;
  rotateHead(person_position_msg_.point);
}

void Follower::rotateHead(geometry_msgs::Point p){
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

int main(int argc, char** argv){
  ros::init(argc, argv, "Follower");
  
  // create object of the node class
  Follower follower;
  // setup node frequency
  double node_frequency = 10;

  ros::Rate loop_rate(node_frequency);

  while(ros::ok()){
    if(follower.is_event_in_received_){
      follower.clearCostmaps();
    }
    follower.update();
    follower.update_head();
    loop_rate.sleep();
  }
  
  return 0;
}

