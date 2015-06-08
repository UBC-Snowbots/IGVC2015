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
  int size = map->width * map->height;
  int start_x, start_y, end_x, end_y;
  
  start_x = (MAP_WIDTH/2) - (map->width/2);
  end_x = (MAP_WIDTH/2) + (map->width/2);
  start_y = MAP_HEIGHT - VISION_OFFSET - map->height; 
  end_y = MAP_HEIGHT - VISION_OFFSET;

  // Assign range on x axis
  if (start_x < 0 || end_x >= MAP_WIDTH)
  {
    start_x = 0;
    end_x = MAP_WIDTH;
  }
  
  // Assign range on y axis
  if (start_y < 0 || end_y > MAP_HEIGHT - VISION_OFFSET)
  {
    start_y = 0;
    end_y = MAP_HEIGHT - VISION_OFFSET;
  }
  
  std::cout << "Width: " << map->width << ", Height:" << map->height << std::endl;
  std::cout << "Start_x :" << start_x << ", End_x: " << end_x << std::endl;
    std::cout << "Start_y :" << start_y << ", End_y: " << end_y << std::endl;
  
  // Loop through to fill in local map
  int i = 0;
  int x = 0;
  int y = 0;
  for (y = start_y; y < end_y; y++)
  { 
    for (x = start_x; x < end_x; x++)
    {
      if (map->data[i])
      {
        local_map.data[y*MAP_WIDTH+x] = 100;
      }
      else 
      {
        if (0 > local_map.data[y*MAP_WIDTH+x])
        {
          local_map.data[y*MAP_WIDTH+x] = 0;
        }
      }
      i++;
    }
  }
  local_map.data[0] = 100;
}


void Mapping::LidarMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map)
{  
  for (int i = 0; i < MAP_WIDTH * MAP_HEIGHT; i++)
  {
    if (map->data[i] > local_map.data[i])
    {
      local_map.data[i] = map->data[i];
    }
  }
}


void Mapping::RosUpdate()
{
  ai_pub.publish(local_map);
  local_map.data.assign(MAP_WIDTH * MAP_HEIGHT, -1);
}

