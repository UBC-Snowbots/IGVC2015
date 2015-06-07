#include "Mapping.h"

Mapping::Mapping(ros::NodeHandle& nh) 
{
  // Allocate memory for published local map
  local_map.data.assign(MAP_WIDTH * MAP_HEIGHT, -1);
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
/*
  int start_x, end_x, start_y, end_y, diff;
  diff = (MAP_WIDTH - l_width)/2;
  start_x = diff;
  end_x = MAP_WIDTH - diff;
  diff = MAP_HEIGHT - l_height;
  start_y = diff;
  end_y = MAP_HEIGHT;
  
  int i, j, j_end;
  j = start_y * MAP_WIDTH + start_x;
  j_end = end_y * MAP_WIDTH + end_x;
  */
  // if (map->data[i] > local_map.data[j]) { local_map.data[j] = map->data[i]; }
  
  for (int i = 0; i < MAP_WIDTH * MAP_HEIGHT; i++)
  {
    local_map.data[i] = map->data[i]; 
  }
}


void Mapping::RosUpdate()
{
  ai_pub.publish(local_map);
}

