#include <iostream>
#include "ros/ros.h"
#include "sb_msgs/LocalMap.h"
#include "AIMapping.h"

using namespace std;

static const string NODE_NAME = "global_mapping_node";
static const string LOCAL_MAP_SUB_TOPIC = "local_map";


void local_map_callback(const sb_msgs::LocalMap::ConstPtr& msg)
{
  ROS_INFO("\n[global_mapping_node]");
  cout << "pose.x = " << msg->pose.x << endl;
  cout << "pose.y = " << msg->pose.y << endl;
  cout << "pose.theta = " << msg->pose.theta << endl;
//  ROS_INFO("x = %d", msg->pose.x);
//  ROS_INFO("y = %d", msg->pose.y);
//  ROS_INFO("rotation = %d", msg->pose.theta);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, NODE_NAME);

  ros::NodeHandle n;
  
  ros::Subscriber sub = n.subscribe(LOCAL_MAP_SUB_TOPIC, 1000, local_map_callback);

  ros::spin();

  return 0;
}
