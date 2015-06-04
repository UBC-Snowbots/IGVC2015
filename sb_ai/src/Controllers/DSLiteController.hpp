#pragma once
#include "ros/ros.h"
#include <string>
#include <iostream>
#include "AI/dstarliteA/GridWorld.h"
#include <geometry_msgs/Twist.h>
#include "ControllerBase.hpp"

#define SIZE 20
#define INTERSECTION_RADIUS 2
#define SCAN_RADIUS 4

namespace ai
{


	class DSLiteController: public ai::ControllerBase
	{
	private:
	  GridWorld* world;
	  int* realWorld;	 
	  void scanMap();
	  void execute();
	  
	public:
		DSLiteController(ros::NodeHandle& nh);	  
		//void MapCallback(const sb_msgs::AISimMap::ConstPtr& msg);
		geometry_msgs::Twist Update();
  };

}

