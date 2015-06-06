#pragma once

#include "stdint.h"
#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "sb_msgs/Waypoint.h"

// hardcoded values
#define GLOBAL_MAP_WIDTH 3000
#define GLOBAL_MAP_HEIGHT 2000
#define GLOBAL_MAP_PADDING 100

class GenerateGlobalMap 
{

private:

	float longLatToMeters = 0.07871 / 0.000001; 

  // FUNCTIONS

	float GetLonMetersDisplacement(sb_msgs::Waypoint gpsOrigin);

	float GetLatMetersDisplacement(sb_msgs::Waypoint gpsOrigin);

	uint8_t ConvertIndexToLocalXCoord(uint8_t index, uint8_t width);

	uint8_t ConvertIndexToLocalYCoord(uint8_t index, uint8_t width);

	uint8_t ConvertXYCoordToIndex(uint8_t x, uint8_t y, uint8_t width);

public:

	GenerateGlobalMap();

	void TransformLocalToGlobal(nav_msgs::OccupancyGrid local, nav_msgs::OccupancyGrid global, float theta);

};
