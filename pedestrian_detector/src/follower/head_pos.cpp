#include <ros/ros.h>
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "std_msgs/UInt8MultiArray.h"
#include <string>

int main(int argc, char **argv){

  ros::init(argc, argv, "head_pos");
  ros::NodeHandle n;
  ros::Publisher head_pub = n.advertise<std_msgs::UInt8MultiArray>("/cmd_head",1);

  std_msgs::UInt8MultiArray controlMsg;
  controlMsg.data.resize(2);
  //Speed control
  controlMsg.data[1] = 30.0;
  //Position control                                                                                                                         
  controlMsg.data[0] = 90.0;


  ros::Rate loop_rate(4);
  while (ros::ok()){
    head_pub.publish(controlMsg);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
  
}
