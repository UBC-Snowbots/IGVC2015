#include <iostream>
#include "ros/ros.h"
#include "sb_msgs/LocalMap.h"

using namespace std;

static const string NODE_NAME = "global_mapping_node";
static const string LOCAL_MAP_SUB_TOPIC = "local_map";


void local_map_callback(const sb_msgs::LocalMap::ConstPtr& msg)
{
  ROS_INFO("[global_mapping_node]");
  ROS_INFO("x = %d", msg->x);
  ROS_INFO("y = %d", msg->y);
  ROS_INFO("rotation = %d", msg->rotation);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, NODE_NAME);

  ros::NodeHandle n;
  
  ros::Subscriber sub = n.subscribe(LOCAL_MAP_SUB_TOPIC, 1000, local_map_callback);

  ros::spin();

  return 0;
}
