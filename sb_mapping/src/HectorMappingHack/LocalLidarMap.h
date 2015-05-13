#include "ros/ros.h"
#include <string>
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"

#define NODE_NAME "local_lidar_map"
#define LIDAR_SUB "scan"
#define MAPPING_PUB "sb_mapping"

class LocalLidarMap {

private:

  nav_msgs::OccupancyGrid map;

  //int* map;
  //int map_width, map_height;
  int pivot_x, pivot_y;
  //float theta;
  float map_resolution; // meters per grid index
  
public:
  LocalLidarMap(int width, int height, float resolution);
  void LidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan);  
  
};
