#include "LocalLidarMap.h"


LocalLidarMap::LocalLidarMap(int width, int height, float resolution)
{
  // Set private variables
  map_width = width;
  map_height = height;
  map_resolution = resolution;
  
  // Initialize map
  int total_map_size = map_width * map_height;
  map = new int[total_map_size];
  for (int i = 0; i < total_map_size; i++)
  {
    map[i] = 0;
  }
}

void LocalLidarMap::LidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  scan->angle_min;
  scan->angle_max;
  scan->angle_increment;
  scan->time_increment;
  scan->scan_time;
  scan->range_min;
  scan->range_max;
  scan->ranges;
  scan->intensities;
}

int main(int argc, char **argv)
{
  LocalLidarMap lidar_map(10, 10, 1.0f);
  
  ros::init(argc, argv, NODE_NAME);
  ros::NodeHandle n;
  
  ros::Subscriber lidar_sub = n.subscribe(LIDAR_SUB, 10, &LocalLidarMap::LidarCallback, &lidar_map);
  ros::Publisher local_map_pub = n.advertise<nav_msgs::OccupancyGrid>(MAPPING_PUB, 10);
  
  ros::Rate loop_rate(10);
  
  while (ros::ok())
  {
  
    ros::spin();
    loop_rate.sleep();
  }  
  
  return 0;
}
