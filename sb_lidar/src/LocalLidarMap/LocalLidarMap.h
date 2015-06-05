#include "ros/ros.h"
#include <string>
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"

#define NODE_NAME "local_lidar_map"
#define LIDAR_SUB "scan"
#define MAPPING_PUB "sb_lidar_map"

// Width of map should be double the height
#define RANGE_M 6
#define MAP_RESOLUTION 0.05f

class LocalLidarMap {

private:
  nav_msgs::OccupancyGrid local_map;
  int map_size, map_width, map_height;
  
public:

  nav_msgs::OccupancyGrid * GetLocalMap();
  LocalLidarMap(int range, float resolution);
  void LidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan);  
  
};
