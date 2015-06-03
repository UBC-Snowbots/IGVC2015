#pragma once
#include "ros/ros.h"
#include <string>
#include <iostream>
#include "AI/D*lite_A/GridWorld.h"
#include <geometry_msgs/Twist.h>
#include "ControllerBase.hpp"

namespace ai
{
	class DSLiteController: public ai::ControllerBase
	{
	private:
	  GridWorld* world;
	  const int SIZE;
	  const int INTERSECTION_RADIUS;
	  const int SCAN_RADIUS;
	  int * realWorld;
	  void scanMap();
	  void execute();
	  
	public:
		DSLiteController(ros::NodeHandle& nh);	  
		//void MapCallback(const sb_msgs::AISimMap::ConstPtr& msg);
		geometry_msgs::Twist Update();
  };

}

