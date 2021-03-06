#pragma once
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sb_msgs/Waypoint.h>

namespace AI_Utilities
{
  struct Location
  {
	  int x;
	  int y;
  };

  static const float MAX_ANG_VEL = 0.3f;
  static const float MAX_LIN_VEL = 0.3f;
  static const float PI = 3.14159265;
  
  geometry_msgs::Twist GetVelocity(Location* next_targets, float elsa_yaw, geometry_msgs::Twist &elsa_command);

  void TransformLocalToGlobal(nav_msgs::OccupancyGrid local, nav_msgs::OccupancyGrid global, float theta);
  
  int GetGlobalIndexX(float gpsOriginLon, float currGpsLon, int globalOriginX, float resolution);

	int GetGlobalIndexY(float gpsOriginLat, float currGpsLat, int globalOriginY, float resolution);

	int ConvertIndexToLocalXCoord(int index, int width);

	int ConvertIndexToLocalYCoord(int index, int width);

	int ConvertXYCoordToIndex(int x, int y, int width);
}


