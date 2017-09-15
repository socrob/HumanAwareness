#include <ros/ros.h>
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "sensor_msgs/PointCloud2.h"
#include "vizzy_msgs/PersonInSight.h"
#include "vizzy_msgs/StartFollowing.h"
#include "vizzy_msgs/ChangeRadius.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_listener.h>
#include "pedestrian_detector/BoundingBox.h"
#include "pedestrian_detector/DetectionList.h"
#include <exception>

geometry_msgs::PoseArray mapeople;
geometry_msgs::Point mcenter;
sensor_msgs::PointCloud2 pcloud;
tf::TransformListener* listener;
double kid_height = 0.4;
bool startfollow = false;
double radiusd = 2.5;
void timerCallback(const ros::TimerEvent& event){
  try{
    listener->waitForTransform("/base_link", "/head_camera_rgb_optical_frame",ros::Time(0), ros::Duration(3.0));
  }
  catch (tf::TransformException &ex) {
    ROS_ERROR("%s",ex.what());
  }
  std::cout <<"radius: " << radiusd << std::endl;
}
pcl::PointXYZ findpclpoint(pcl::PointCloud<pcl::PointXYZ> cloud, geometry_msgs::Point crowdpoint){
  pcl::PointXYZ rgbdpoint, aux_pxyz;
  geometry_msgs::Point aux_p;
  int count=0;
  bool notav;
  rgbdpoint.x = 0;
  rgbdpoint.y = 0;
  rgbdpoint.z = 0;
  for(int i = -10; i < 11; i++){
    for(int j = -10; j < 11; j++){
      aux_p.x = crowdpoint.x+i;
      aux_p.y = crowdpoint.y+j;
      notav = false;
      if(aux_p.x >= 0 && aux_p.x <= 640 && aux_p.y >=0 && aux_p.y <= 480){
	try{
	  aux_pxyz = cloud.at(aux_p.x,aux_p.y);
	}catch(std::exception& e){
	  ROS_ERROR("POINT NOT AVAILABLE");
	  notav = true;
	}
	if(std::isnan(aux_pxyz.x) == false && std::isnan(aux_pxyz.y) == false && std::isnan(aux_pxyz.z) == false){
	  rgbdpoint.x = rgbdpoint.x + aux_pxyz.x;
	  rgbdpoint.y = rgbdpoint.y + aux_pxyz.y;
	  rgbdpoint.z = rgbdpoint.z + aux_pxyz.z;
	  if(not notav){
	    count++;
	  }
	}
      }
    }
  }
  rgbdpoint.x = rgbdpoint.x/count;
  rgbdpoint.y = rgbdpoint.y/count;
  rgbdpoint.z = rgbdpoint.z/count;
  return rgbdpoint;
}

void pclCallback(const sensor_msgs::PointCloud2ConstPtr& pcloudmsg){
  //std::cout << "pclCallback" << std::endl;
  
  pcloud = *pcloudmsg;

}

bool inradius(pcl::PointXYZ point){
  geometry_msgs::PointStamped pointin, pointout;
  bool in;
  
  pointin.point.x = point.x;
  pointin.point.y = point.y;
  pointin.point.z = point.z;
  pointin.header.stamp = ros::Time::now();
  pointin.header.frame_id = "/head_camera_rgb_optical_frame";
  
  try{
    //listener.waitForTransform("/asus_object_rgb_optical_frame", "/map",ros::Time(0), ros::Duration(3.0));
    listener->transformPoint("/base_link",ros::Time(0),pointin,"/head_camera_rgb_optical_frame",pointout); 
  }
  catch (tf::TransformException &ex) {
    ROS_ERROR("%s",ex.what());
  }
  
  //float at = atan2(pointout.point.y,pointout.point.x);
  
  float d = sqrt(pow(pointout.point.x,2) + pow(pointout.point.y,2));
  
  if(d < radiusd){
    in = true;
  }else{
    in = false;
  }

  return in;
}

void bbCallback(const pedestrian_detector::DetectionListConstPtr& bpeople){
  //std::cout << "bbCallback" << std::endl;
  mapeople.poses.clear();
  geometry_msgs::Point crowdpoint;
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(pcloud,pcl_pc2);
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromPCLPointCloud2(pcl_pc2,cloud);
  bool org = cloud.isOrganized();
  bool goodp;
  
  if(org){
  
    pcl::PointXYZ rgbdpoint;
    std::vector<pcl::PointXYZ> rgbdpoints;
    std::vector<geometry_msgs::PointStamped> mapoints;
    geometry_msgs::PointStamped asuspoint, mapoint;
    geometry_msgs::Pose maperson;
    int psize = bpeople->headsVector.size();
    for(int i = 0; i < psize; i++){
      float center_x = bpeople->headsVector[i].x + bpeople->headsVector[i].width/2;
      float center_y = bpeople->headsVector[i].y + bpeople->headsVector[i].height/2;
    
      crowdpoint.x = center_x;
      crowdpoint.y = center_y;
      crowdpoint.z = 0;

      rgbdpoint = findpclpoint(cloud,crowdpoint);
      
      goodp = inradius(rgbdpoint);
      if(goodp){
	rgbdpoints.push_back(rgbdpoint);//fazer clear do crowdpoints
      }
    }
    int rgbdsize = rgbdpoints.size(); 
    for(int i = 0; i < rgbdsize ; i++){
      //rgbdpoint = cloud.at(crowdpoints[i].x,crowdpoints[i].y);
    
      asuspoint.point.x = rgbdpoints[i].x;
      asuspoint.point.y = rgbdpoints[i].y;
      asuspoint.point.z = rgbdpoints[i].z;
      asuspoint.header.stamp = ros::Time::now();
      asuspoint.header.frame_id = "/head_camera_rgb_optical_frame";
    
      try{
	listener->transformPoint("/base_link",ros::Time(0),asuspoint,"/head_camera_rgb_optical_frame",mapoint); 
      }catch (tf::TransformException &ex) {
	ROS_ERROR("%s",ex.what());
      }
      mapoints.push_back(mapoint);
      maperson.position = mapoint.point;
      maperson.orientation.x = 0;
      maperson.orientation.y = 0;
      maperson.orientation.z = 0;
      maperson.orientation.w = 1;
      //      std::cout << "person x: " << maperson.position.x << " y: " << maperson.position.y << std::endl;
      mapeople.poses.push_back(maperson);
    } 
    rgbdpoints.clear();
  }
}

bool insightserviceCallback(vizzy_msgs::PersonInSight::Request &req, vizzy_msgs::PersonInSight::Response &res){
  int s = mapeople.poses.size();
  bool person = false;
  if(s > 0){
    person = true;
  }
    
  res.insight = person;
  return true;
}

bool followserviceCallback(vizzy_msgs::StartFollowing::Request &req, vizzy_msgs::StartFollowing::Response &res){
  startfollow = req.start;
  res.follow = startfollow;
  return true;
}

bool changeradiusCallback(vizzy_msgs::ChangeRadius::Request &req, vizzy_msgs::ChangeRadius::Response &res){
  radiusd = req.distance;
  res.change = true;
  return true;
}

int main(int argc, char **argv){

  ros::init(argc, argv, "centralp");
  ros::NodeHandle n;
  listener = new (tf::TransformListener);
  ros::Subscriber bb_sub = n.subscribe("/detections", 1, bbCallback); 
  ros::Subscriber pcl_sub = n.subscribe("/head_camera/depth_registered/points", 1, pclCallback); 
  ros::ServiceServer insightservice = n.advertiseService("PersonInFront", insightserviceCallback);
  ros::ServiceServer followservice = n.advertiseService("FollowNow", followserviceCallback);
  ros::ServiceServer radiusservice = n.advertiseService("DetectRadius", changeradiusCallback);
  ros::Publisher mapeople_pub = n.advertise<geometry_msgs::PoseArray>("mapeople", 1);
  
  ros::Timer timer = n.createTimer(ros::Duration(0.1), timerCallback);

  ros::Rate loop_rate(20);
  while (ros::ok()){
    mapeople.header.stamp = ros::Time::now();
    mapeople.header.frame_id = "/base_link";
    if(startfollow){
      mapeople_pub.publish(mapeople);
    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
