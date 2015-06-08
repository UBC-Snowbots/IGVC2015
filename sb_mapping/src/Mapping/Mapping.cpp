#include "Mapping.h"

Mapping::Mapping(ros::NodeHandle& nh) 
{
  // Allocate memory for published local map
  local_map.data.assign(MAP_WIDTH * MAP_HEIGHT, 100); // -1
  local_map.info.width = MAP_WIDTH;
  local_map.info.height = MAP_HEIGHT;
  local_map.info.origin.position.x = 0;
  local_map.info.origin.position.y = 0;
  local_map.info.origin.position.z = 0;
  local_map.info.origin.orientation.x = 0;
  local_map.info.origin.orientation.y = 0;
  local_map.info.origin.orientation.z = 0;
  local_map.info.origin.orientation.w = 0;
  local_map.info.resolution = 0.05;
    
  // Initialize subscribers and publishers
  vision_sub = nh.subscribe(VISION_SUB_TOPIC, 10, &Mapping::VisionMapCallback, this);
  lidar_sub = nh.subscribe(LIDAR_SUB_TOPIC, 10, &Mapping::LidarMapCallback, this); 
  ai_pub = nh.advertise<nav_msgs::OccupancyGrid>(AI_PUB_TOPIC, 10);
}


void Mapping::VisionMapCallback(const sensor_msgs::Image::ConstPtr& map)
{
  return;
}


void Mapping::LidarMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map)
{  
  for (int i = 0; i < MAP_WIDTH * MAP_HEIGHT; i++)
  {
    local_map.data[i] = map->data[i];
  }
}


void Mapping::RosUpdate()
{
  ai_pub.publish(local_map);
}

