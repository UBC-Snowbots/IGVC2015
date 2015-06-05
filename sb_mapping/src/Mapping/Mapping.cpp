#include "Mapping.h"

Mapping::Mapping(ros::NodeHandle& nh) : 
  v_width(200), 
  v_height(150), 
  l_width(100), 
  l_height(76), 
  v_l_offset(20)
{
  map_width = v_width; // max of two , forgot the math fnc
  if (l_height >= v_height + v_l_offset) { map_height = l_height; } // based on assumption lidar displacement is always below vision
  else { map_height = v_height + v_l_offset; } 
    
  // Allocate memory for published local map
  local_map.data.assign(map_width * map_height, -1);
    
  // Initialize subscribers and publishers
  vision_sub = nh.subscribe(VISION_SUB_TOPIC, 10, &Mapping::VisionMapCallback, this);
  lidar_sub = nh.subscribe(LIDAR_SUB_TOPIC, 10, &Mapping::LidarMapCallback, this); 
  ai_pub = nh.advertise<nav_msgs::OccupancyGrid>(AI_PUB_TOPIC, 1);
}


void Mapping::VisionMapCallback(const sensor_msgs::Image::ConstPtr& map)
{
  return;
}


void Mapping::LidarMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map)
{
  int start_x, end_x, start_y, end_y, diff;
  diff = (map_width - l_width)/2;
  start_x = diff;
  end_x = map_width - diff;
  diff = map_height - l_height;
  start_y = diff;
  end_y = map_height;
  
  int i, j, j_end;
  j = start_y * map_width + start_x;
  j_end = end_y * map_width + end_x;
  
  for (i = 0, j; i < l_width * l_height, j < j_end; i++, j++)
  {
    if (local_map.data[j] > 0 && map->data[i] <= 0) {}
    else { local_map.data[j] = map->data[i]; }
  }
}


void Mapping::RosUpdate()
{
  local_map.data.assign(map_width * map_height, -1);
  ai_pub.publish(local_map);
}

