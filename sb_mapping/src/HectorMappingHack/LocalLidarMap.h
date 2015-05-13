#include "ros/ros.h"
#include <string>
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"

#define NODE_NAME "local_lidar_map"
#define LIDAR_SUB "scan"
#define MAPPING_PUB "sb_mapping"

#define MAP_WIDTH 10
#define MAP_HEIGHT 10
#define MAP_RESOLUTION 1.0f

class LocalLidarMap {

private:
  nav_msgs::OccupancyGrid local_map;
  
public:

  nav_msgs::OccupancyGrid * GetLocalMap();
  LocalLidarMap(int width, int height, float resolution);
  void LidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan);  
  
};
