#include <ros/ros.h>
#include <std_msgs/Time.h>
#include <std_msgs/Header.h>
#include <string>
#include <tf/transform_listener.h>
#include "people_msgs/People.h"
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>


bool id = false;
std::string name;
geometry_msgs::PointStamped personposition;

int closerperson(std::vector<people_msgs::Person> m,int s){

  int closer = 0;
  geometry_msgs::PointStamped pointin;
  float dcloser = 0;

  for(int i = 0; i < s; i++){

    pointin.point.x = m[i].position.x;
    pointin.point.y = m[i].position.y;
    pointin.point.z = m[i].position.z;
    float d = sqrt(pow(pointin.point.x,2) + pow(pointin.point.y,2));

    if(i == 0){
      dcloser = d;
    }else{
      if (d < dcloser){
        closer = i;
        dcloser = d;
      }
    }

  }

  return closer;

}

void peopletrackCallback(const people_msgs::People::ConstPtr& msg){
  int s = msg->people.size();
  geometry_msgs::PointStamped personpoint;
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Quaternion q;
  geometry_msgs::Point p;
  float vx, vy, at,personvx,personvy;
  
  if(!id){
    id = true;
    int i = closerperson(msg->people,s);
    name = msg->people[i].name;
    personpoint.point = msg->people[i].position;
    personpoint.header = msg->header;
    personvx = msg->people[i].velocity.x;
    personvy = msg->people[i].velocity.y;
  }else{
    int count = 0;
    for(int i = 0; i < s; i++){
      if(name == msg->people[i].name){
        personpoint.point = msg->people[i].position;
        personpoint.header = msg->header;
        personvx = msg->people[i].velocity.x;
        personvy = msg->people[i].velocity.y;
      }else{
      	count++;
	    }

      if(count==s){
        int j = closerperson(msg->people,s);
        name = msg->people[j].name;
        personpoint.point = msg->people[j].position;
        personpoint.header = msg->header;
        personvx = msg->people[i].velocity.x;
        personvy = msg->people[i].velocity.y;
      }
    }
  }
  
  personposition=personpoint;
  std::cout << "tf pub" << std::endl;

  p.x = personpoint.point.x;
  p.y = personpoint.point.y;
  vx = personvx;
  vy = personvy;

  at = atan2(vy, vx);
  q = tf::createQuaternionFromYaw(at);

  transform.setRotation(q);
  transform.setOrigin( tf::Vector3(p.x, p.y, 0.0));
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "tracked_person"));
//end
}

int main(int argc,char* argv[]){
  ros::init(argc, argv, "after");
  
  ros::NodeHandle n;
  //listener = new (tf::TransformListener);

  ros::Subscriber trackpeople_sub = n.subscribe("people_tracker/people", 1, peopletrackCallback);
  ros::Publisher personpub = n.advertise<geometry_msgs::PointStamped>("person_position",10);
  
  ros::Rate loop(20);
  while(ros::ok()){
    if(personposition.header.frame_id != " "){
      personpub.publish(personposition);
      personposition.header.frame_id =' ';
    }
    ros::spinOnce();
    loop.sleep();
  }
}
