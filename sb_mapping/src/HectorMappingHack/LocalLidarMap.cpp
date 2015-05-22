#include "LocalLidarMap.h"
#include <cmath>

LocalLidarMap::LocalLidarMap(int width, int height, float resolution)
{
  // Set map parameters
  map_width = width;
  map_height = height;
  local_map.info.width = width;
  local_map.info.height = height;
  local_map.info.resolution = resolution;

  // Initialize map
  map_size = width * height;
  local_map.data.assign(map_size, 0);
  
  // Set transformations; These should be constant, given the first data
  local_map.info.origin.position.x = width / 2;
  local_map.info.origin.position.y = height - 1;
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
  // TEST
  ROS_INFO("Min angle: %f, Max angle: %f", scan->angle_min, scan->angle_max);
  ROS_INFO("Min range: %f, Max Range: %f", scan->range_min, scan->range_max);
  
  float x, y;
  float range, theta;
  int scan_rays = scan->ranges.size();
  ROS_INFO("Range size: %i", (int) scan->ranges.size());
  local_map.data.assign(map_size, 0); // clears the occupancy grid
  for (int i = 0; i < scan_rays; i++)
  {
    range = scan->ranges[i];
    ROS_INFO("Range: %f", range);
    
    if (range < scan->range_min || range > scan->range_max || isinf(range) || isnan(range)) {}
    else
    {
      theta = i*scan->angle_increment + scan->angle_min;
      ROS_INFO("Theta: %f", theta);
      x = range * cos(theta);
      y = range * sin(theta);
      x = map_width / 2 + x; 
      //x /= map_width;       // floor
      //y /= map_height;      // floor
      int index = (int) y*map_width + x;
      local_map.data[index] = 100;
      ROS_INFO("Index: %f, %i", y*map_width + x, index);
      ROS_INFO("Map size: %lus", local_map.data.size());
    }
  }
}

int main(int argc, char **argv)
{
  // Initialize lidar map 
  LocalLidarMap lidar_map(MAP_WIDTH, MAP_HEIGHT, MAP_RESOLUTION);
  
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
    ROS_INFO("Width: %i, Height: %i", lidar_map.GetLocalMap()->info.width, lidar_map.GetLocalMap()->info.height);
    
    local_map_pub.publish(*lidar_map.GetLocalMap());
    
    ros::spinOnce();
    loop_rate.sleep();
  }  
  
  return 0;
}
