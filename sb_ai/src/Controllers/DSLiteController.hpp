#pragma once
#include "ros/ros.h"
#include "sb_ai.h"
#include <string>
#include <iostream>
#include "AI/dstarliteB/GridWorld.h"
#include <geometry_msgs/Twist.h>
#include <sb_msgs/RobotState.h>
#include <sb_msgs/Waypoint.h>
#include <nav_msgs/OccupancyGrid.h>
#include "ControllerBase.hpp"
#include "AI/Utilities/Utilities.h"



namespace ai
{
	class DSLiteController: public ai::ControllerBase
	{
	private:
	
	  GridWorld* world;
	  int* realWorld;	 
	  
	  void scanMap();
	  void execute();
	  
	  // On start state variables
	  int global_width, global_height, global_size; // assigned at beginning
	  int origin_x, origin_y; // assigned at beginning
	  float global_orientation; // assigned after we get gps reading
	  float origin_long, origin_lat; // after we get about 10 gps readings
	  int gps_count, compass_count; // counts for averaging gps/compass data
	  
	  // Current state variables
	  float long_pos, lat_pos; // gps waypoint for current position 
	  int x_pos, y_pos; // current global map position (index)
	  float orientation;  // current global map compass orientation (rad)
	  ros::Subscriber map_sub, gps_sub, compass_sub;
	  
	  // FOR TESTING THE MAP
	  ros::Publisher global_map_pub;
	  nav_msgs::OccupancyGrid global_map;
	  void UpdateGlobalMapTest();
	  
	public:
		DSLiteController(ros::NodeHandle& nh);	  
		
		void MapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map);
		void GpsCallback(const sb_msgs::Waypoint::ConstPtr& gps);
		void CompassCallback(const sb_msgs::RobotState::ConstPtr& compass);
		
		geometry_msgs::Twist Update();
  };

}

