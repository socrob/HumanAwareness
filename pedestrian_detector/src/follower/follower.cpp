#include <follower/follower.h>
#include <string>
#include <vector>

Follower::Follower():nh_("~"),is_event_in_received_(false){
  // subscriptions
  sub_event_in_ = nh_.subscribe("/person_position",1,&Follower::eventInCallBack,this);
  //pub_event_in_ = nh_.advertise<std_msgs::String>("/move_base_wrapper/event_in",2);
  pub_pose_in_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",2);
  pub_head_rot_ = nh_.advertise<std_msgs::UInt8MultiArray>("/cmd_head",2);
  // querying parameters from parameter server
  getParams();
  //start_.data = "e_start";
  //stop_.data = "e_stop";
  listener = new (tf::TransformListener);
  
}
  
Follower::~Follower(){
  sub_event_in_.shutdown();
  //pub_event_in_.shutdown();
  pub_pose_in_.shutdown();
}

void Follower::getParams(){
}

void Follower::eventInCallBack(const geometry_msgs::PointStampedConstPtr& msg){
  event_in_msg_ = *msg;
  is_event_in_received_ = true;
}

void Follower::update(){
  // listen to callbacks
  ros::spinOnce();
  
  if(!is_event_in_received_) return;

  // reset flag
  is_event_in_received_ = false;

  double x = event_in_msg_.point.x;
  double y = event_in_msg_.point.y;
  double z = event_in_msg_.point.z;
  
  double vx = x;
  double vy = y;
  //double norm_v = abs(sqrt(x*x + y*y));
  //double ux = vx/norm_v;
  //double uy = vy/norm_v;
  double d = 0.5;
  double final_x = d*vx;
  double final_y = d*vy;
   
  event_out_msg_.header=event_in_msg_.header;
  event_out_msg_.pose.position.x = final_x;
  event_out_msg_.pose.position.y = final_y;
  event_out_msg_.pose.position.z = z;
  event_out_msg_.pose.orientation = tf::createQuaternionMsgFromYaw(atan2(final_y,final_x));
  rotateHead(event_out_msg_.pose.position);
  //pub_event_in_.publish(stop_);
  pub_pose_in_.publish(event_out_msg_);
  //pub_event_in_.publish(start_);
}

void Follower::rotateHead(geometry_msgs::Point p){
  tf::StampedTransform transform;
  double roll, pitch, yaw;
  
  try{ 
    listener->lookupTransform("/base_link", "/head_link", ros::Time(0), transform);
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

int main(int argc, char** argv){
  ros::init(argc, argv, "Follower");
  
  // create object of the node class
  Follower follower;
  // setup node frequency
  double node_frequency = 10;

  ros::Rate loop_rate(node_frequency);

  while(ros::ok()){
    follower.update();
    loop_rate.sleep();
  }
  
  return 0;
}

