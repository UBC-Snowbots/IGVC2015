#include "ros/ros.h"
#include <string>
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"

#define NODE_NAME "local_lidar_map"
#define LIDAR_SUB "scan"
#define MAPPING_PUB "sb_mapping"

// Width of map should be double the height
#define MAP_WIDTH 256  
#define MAP_HEIGHT 128
#define MAP_RESOLUTION 0.2f

class LocalLidarMap {

private:
  nav_msgs::OccupancyGrid local_map;
  int map_size, map_width, map_height;
  
public:

  nav_msgs::OccupancyGrid * GetLocalMap();
  LocalLidarMap(int width, int height, float resolution);
  void LidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan);  
  
};
