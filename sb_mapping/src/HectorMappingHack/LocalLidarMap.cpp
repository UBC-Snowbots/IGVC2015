#include "LocalLidarMap.h"

LocalLidarMap::LocalLidarMap(int width, int height, float resolution)
{
  // Set map parameters
  local_map.info.width = width;
  local_map.info.height = height;
  local_map.info.resolution = resolution;

  // Initialize map
  int map_size = width * height;
  local_map.data.assign(map_size, 0);
  
  // Set transformations
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
  LocalLidarMap lidar_map(MAP_WIDTH, MAP_HEIGHT, MAP_RESOLUTION);
  
  ros::init(argc, argv, NODE_NAME);
  ros::NodeHandle n;
  
  ros::Subscriber lidar_sub = n.subscribe(LIDAR_SUB, 10, &LocalLidarMap::LidarCallback, &lidar_map);
  ros::Publisher local_map_pub = n.advertise<nav_msgs::OccupancyGrid>(MAPPING_PUB, 10);
  
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
