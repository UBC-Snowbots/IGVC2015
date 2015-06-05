#include "ros/ros.h"
#include "Mapping/Mapping.h"

#define NODE_NAME "sb_mapping"

int main(int argc, char **argv)
{
  ros::init(argc, argv, NODE_NAME);
  ros::NodeHandle nh;
  ros::Rate loop_rate(10);
  Mapping mapping(nh);
  
  while(ros::ok())
  {
    mapping.RosUpdate(); // subscribes, processes data, publishes
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  return 0;
}
