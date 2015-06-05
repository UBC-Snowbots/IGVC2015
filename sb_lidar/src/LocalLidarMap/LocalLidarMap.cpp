#include "LocalLidarMap.h"
#include <cmath>
#define _USE_MATH_DEFINES // for C++
#include <cmath>

LocalLidarMap::LocalLidarMap(int range, float resolution)
{
  // Set map parameters
  map_width = (range * 2) / resolution;
  map_height = range / resolution;
  local_map.info.width = map_width;
  local_map.info.height = map_height;
  local_map.info.resolution = resolution;

  // Initialize map
  map_size = map_width * map_height;
  local_map.data.assign(map_size, 0);
  
  // Set transformations; These should be constant, given the first data
  local_map.info.origin.position.x = map_width / 2;
  local_map.info.origin.position.y = map_height - 1;
  local_map.info.origin.position.z = 0;
  // quaternion
  local_map.info.origin.orientation.x = 0;
  local_map.info.origin.orientation.y = 0;
  local_map.info.origin.orientation.z;
  local_map.info.origin.orientation.w;
}

nav_msgs::OccupancyGrid * LocalLidarMap::GetLocalMap()
{
  return &local_map;
}

void LocalLidarMap::LidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{  
  float x, y;
  float range, theta;
  int scan_rays = scan->ranges.size();
  ROS_INFO("Range size: %i", (int) scan->ranges.size());
  local_map.data.assign(map_size, 0); // clears the occupancy grid
  for (int i = 0; i < scan_rays; i++)
  {
    range = scan->ranges[i];
    
    if (range < scan->range_min || range > scan->range_max || isinf(range) || isnan(range)) {}
    else
    {
      theta = i*scan->angle_increment + scan->angle_min;
      std::cout << "Theta: " << theta << std::endl;
     // std::cout << "Min angle: " << scan->angle_min << std::endl;
      //std::cout << "MAx angle: " << scan->angle_max << std::endl;
  
      x = range * sin(theta);
      x /= MAP_RESOLUTION;
      x += map_width / 2;
      y = range * cos(theta);
      y /= MAP_RESOLUTION;
      int index = (int) y*map_width + x;
      local_map.data[index] = 100;
    }
  }
}


int main(int argc, char **argv)
{
  // Initialize lidar map 
  LocalLidarMap lidar_map(RANGE_M, MAP_RESOLUTION);
  
  // Initialize node and node handler
  ros::init(argc, argv, NODE_NAME);
  ros::NodeHandle n;
  
  // Initialize subscribers and publishers
  ros::Subscriber lidar_sub = n.subscribe(LIDAR_SUB, 10, &LocalLidarMap::LidarCallback, &lidar_map);
  ros::Publisher local_map_pub = n.advertise<nav_msgs::OccupancyGrid>(MAPPING_PUB, 10);
  
  // Specify rate for the while loop below
  ros::Rate loop_rate(10);
  
  while (ros::ok())
  {
    //ROS_INFO("Width: %i, Height: %i", lidar_map.GetLocalMap()->info.width, lidar_map.GetLocalMap()->info.height);
    
    local_map_pub.publish(*lidar_map.GetLocalMap());
    
    ros::spinOnce();
    loop_rate.sleep();
  }  
  
  return 0;
}
