#include "MapLocalToGlobal.h"
#include "math.h"

GenerateGlobalMap::GenerateGlobalMap()
{  
}


void GenerateGlobalMap::TransformLocalToGlobal(nav_msgs::OccupancyGrid local, nav_msgs::OccupancyGrid global, float theta) 
{
	uint8_t xGlobalVisionCoord;
	uint8_t yGlobalVisionCoord;

  uint32_t localMapSize = local.info.width * local.info.height; 

	// Loop through the vision map
	for (int index = 0; index < localMapSize; index++) {

		xGlobalVisionCoord = cos(theta)
				* ConvertIndexToLocalXCoord(index, local.info.width)
				- sin(theta) * ConvertIndexToLocalYCoord(index, local.info.width)
				+ local.info.origin.position.x;
		yGlobalVisionCoord = sin(theta)
				* ConvertIndexToLocalXCoord(index, local.info.width)
				+ cos(theta) * ConvertIndexToLocalYCoord(index, local.info.width)
				+ local.info.origin.position.y;

		// Update global map with 0/1 to show that an obstacle dne/exists
		global.data[ConvertXYCoordToIndex(xGlobalVisionCoord,
				yGlobalVisionCoord, global.info.width)] =
				local.data[index];
	}
}


float GenerateGlobalMap::GetLonMetersDisplacement(sb_msgs::Waypoint gpsOrigin) 
{
  return gpsOrigin.lon * (longLatToMeters);
}


float GenerateGlobalMap::GetLatMetersDisplacement(sb_msgs::Waypoint gpsOrigin) 
{
  return gpsOrigin.lat * (longLatToMeters);
}


uint8_t GenerateGlobalMap::ConvertIndexToLocalXCoord(uint8_t index, uint8_t width) 
{
	return index % width;
}


uint8_t GenerateGlobalMap::ConvertIndexToLocalYCoord(uint8_t index, uint8_t width) 
{
	return index / width;
}


uint8_t GenerateGlobalMap::ConvertXYCoordToIndex(uint8_t x, uint8_t y, uint8_t width) 
{
	return y * width + x;
}
