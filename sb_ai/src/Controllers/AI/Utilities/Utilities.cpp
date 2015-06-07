#include <cmath>
#include <iostream>
#include "Utilities.h"

namespace AI_Utilities
{
  geometry_msgs::Twist GetVelocity(Location* next_targets, float elsa_yaw, geometry_msgs::Twist &elsa_command)
  {
    elsa_command.linear.x = 0;
    elsa_command.linear.y = 0;
    elsa_command.linear.z = 0;
    elsa_command.angular.x = 0;
    elsa_command.angular.y = 0;
    elsa_command.angular.z = 0;
    
    // Check for null 
    if (!next_targets) return elsa_command; 
    
    float dist_x, dist_y, ang_dist_sign;
    float ang_dist;
    float target_yaw = 0.0f;
    
    dist_x = next_targets[1].x - next_targets[0].x;
    dist_y = next_targets[1].y - next_targets[0].y;

    target_yaw = atan(dist_y/dist_x);
    ang_dist = target_yaw - elsa_yaw; // need to fix negatives
    ang_dist_sign = (int) ang_dist;
    ang_dist_sign /= abs(ang_dist_sign);  // check that this actually works
    ang_dist = abs(ang_dist);

    if (ang_dist > 120.0f)
    {
      elsa_command.angular.z = 0.3 * ang_dist_sign;
      elsa_command.linear.y = 0.1;
    }

    else if (ang_dist > 80.0f)
    {
      elsa_command.angular.z = 0.25 * ang_dist_sign;
      elsa_command.linear.y = 0.15;
    }
    
     else if (ang_dist > 40.0f)
    {
      elsa_command.angular.z = 0.2 * ang_dist_sign;
      elsa_command.linear.y = 0.2;
    }
    
    else if (ang_dist > 10.0f)
    {
      elsa_command.angular.z = 0.15 * ang_dist_sign;
      elsa_command.linear.y = 0.25;
    }
    
    else
    {
      elsa_command.angular.z = 0;
      elsa_command.linear.y = 0.3;
    }
    
    return elsa_command;
  }


  void TransformLocalToGlobal(nav_msgs::OccupancyGrid local, nav_msgs::OccupancyGrid global, float theta) 
  {
	  uint8_t xGlobalVisionCoord;
	  uint8_t yGlobalVisionCoord;

    uint32_t localMapSize = local.info.width * local.info.height; 

	  // Loop through the vision map
	  for (int index = 0; index < localMapSize; index++) 
	  {

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


  int GetGlobalIndexX(float gpsOriginLon, float currGpsLon, int globalOriginX, float resolution) 
  {
  	float longConv = 111412.84 * cos(currGpsLon)
		- 93.5 * cos(3 * currGpsLon) - 0.118 * cos(5 * currGpsLon);
    
    int diff = (currGpsLon - gpsOriginLon) * longConv;
    diff /= resolution;
    diff += globalOriginX;
    return diff;
  }


  int GetGlobalIndexY(float gpsOriginLat, float currGpsLat, int globalOriginY, float resolution) 
  {
  	float latConv = 111132.92 - 559.82 * cos(2 * currGpsLat)
		+ 1.175 * cos(4 * currGpsLat)
		- 0.0023 * cos(6 * currGpsLat);
  
    int diff = (currGpsLat - gpsOriginLat) * latConv;
    diff /= resolution;
    diff += globalOriginY;
    return diff;
  }


  uint8_t ConvertIndexToLocalXCoord(uint8_t index, uint8_t width) 
  {
	  return index % width;
  }


  uint8_t ConvertIndexToLocalYCoord(uint8_t index, uint8_t width) 
  {
	  return index / width;
  }


  uint8_t ConvertXYCoordToIndex(uint8_t x, uint8_t y, uint8_t width) 
  {
	  return y * width + x;
  }

}
                  
