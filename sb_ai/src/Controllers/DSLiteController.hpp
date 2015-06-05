#pragma once
#include "ros/ros.h"
#include "sb_ai.h"
#include <string>
#include <iostream>
#include "AI/dstarliteB/GridWorld.h"
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>  // temp for gps and compass
#include <nav_msgs/OccupancyGrid.h>
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
	  float long_pos, lat_pos; // gps waypoint for current position
	  int x_pos, y_pos; // global map position
	  float orientation;
	  ros::Subscriber map_sub, gps_sub, compass_sub;
	  
	public:
		DSLiteController(ros::NodeHandle& nh);	  
		
		void MapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map);
		void GpsCallback(const std_msgs::String::ConstPtr& gps);
		void CompassCallback(const std_msgs::String::ConstPtr& compass);
		
		geometry_msgs::Twist Update();
  };

}

