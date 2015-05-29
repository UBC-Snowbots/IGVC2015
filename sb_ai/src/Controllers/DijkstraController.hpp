#pragma once

#include "ros/ros.h"
#include <string>
#include <iostream>
#include "sb_ai.h"
#include "sb_msgs/AISimMap.h"
#include "AI/dijkstra/dijkstra_recursive.h"
#include <geometry_msgs/Twist.h>
#include "ControllerBase.hpp"

namespace ai
{
  class DijkstraController: public ai::ControllerBase
  {
  
  private:
	  int * map_ptr;
	  int width; 
	  int height;
	  int start;
	  int goal;
	  geometry_msgs::Twist twist_msg;
	  Dijkstra dijkstras;
	  ros::Subscriber map_sub;
	  int first, fourth;
	  
  public:
	  DijkstraController(ros::NodeHandle& nh);
	  void MapCallback(const sb_msgs::AISimMap::ConstPtr& msg);
	  geometry_msgs::Twist Update();
  };
}

