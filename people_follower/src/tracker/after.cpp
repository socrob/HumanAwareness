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
geometry_msgs::PointStamped poi_position_;
ros::Subscriber tracked_people_subscriber_;
bool subscribed_to_input_topics_ = false;

int closestPerson(std::vector<people_msgs::Person> m, int s){

  int closest = 0;
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
        closest = i;
        dcloser = d;
      }
    }
  }

  return closest;

}

void trackedPeopleCallback(const people_msgs::People::ConstPtr &msg){
  int s = msg->people.size();
  geometry_msgs::PointStamped person_point;
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Quaternion q;
  geometry_msgs::Point p;
  float vx, vy, at, personvx = 0.0, personvy = 0.0;
  
  if(!id){
    id = true;
    int i = closestPerson(msg->people, s);
    name = msg->people[i].name;
    person_point.point = msg->people[i].position;
    person_point.header = msg->header;
    personvx = msg->people[i].velocity.x;
    personvy = msg->people[i].velocity.y;
  }else{
    int count = 0;
    for(int i = 0; i < s; i++){
      if(name == msg->people[i].name){
        person_point.point = msg->people[i].position;
        person_point.header = msg->header;
        personvx = msg->people[i].velocity.x;
        personvy = msg->people[i].velocity.y;
      }else{
      	count++;
      }

      if(count==s){
        int j = closestPerson(msg->people, s);
        name = msg->people[j].name;
        person_point.point = msg->people[j].position;
        person_point.header = msg->header;
        personvx = msg->people[i].velocity.x;
        personvy = msg->people[i].velocity.y;
      }
    }
  }
  
  poi_position_=person_point;

  p.x = person_point.point.x;
  p.y = person_point.point.y;
  vx = personvx;
  vy = personvy;

  at = atan2(vy, vx);
  q = tf::createQuaternionFromYaw(at);

  transform.setRotation(q);
  transform.setOrigin( tf::Vector3(p.x, p.y, 0.0));
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "tracked_person"));
//end
}

void createSubscribers(){
  ros::NodeHandle n;
  tracked_people_subscriber_ = n.subscribe("bayes_people_tracker/people", 1, trackedPeopleCallback);
  ROS_INFO("Creating subscribers");
}

void destroySubscribers(){
  tracked_people_subscriber_.shutdown();
  ROS_INFO("Destroying subscribers");
}


int main(int argc,char* argv[]){
  ros::init(argc, argv, "after");
  
  ros::NodeHandle n;
  ros::Rate loop(30);

  ros::Publisher personpub = n.advertise<geometry_msgs::PointStamped>("person_position", 10);

  while(ros::ok()){

    if(personpub.getNumSubscribers() > 0 && !subscribed_to_input_topics_){
      createSubscribers();
      subscribed_to_input_topics_ = true;
    }

    if(personpub.getNumSubscribers() == 0 && subscribed_to_input_topics_){
      destroySubscribers();
      subscribed_to_input_topics_ = false;
    }

    if(poi_position_.header.frame_id != " "){
      personpub.publish(poi_position_);
      poi_position_.header.frame_id =' ';
    }

    ros::spinOnce();
    loop.sleep();
  }
}
