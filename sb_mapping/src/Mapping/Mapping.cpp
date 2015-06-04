#include "Mapping.h"

Mapping::Mapping()
{
    local_width = 200;
    local_height = 200;
    local_size = local_width * local_height;
    final_resolution = 0.5; 
    
    lidar_pos_x = 0.5;
    lidar_pos_y = 1.0;
    vision_pos_x = 0.5;
    vision_pos_y = 0.75;
    
    final_resolution = 0.05; // m/cell
    
    // Hardcode for now
    lidar_width = 60;
    lidar_height = 30;
    
    // Initialize subscribers and publishers
    //vision_sub
    //lidar_sub
    // ai_pub
}


void Mapping::VisionMapCallback(const sensor_msgs::Image::ConstPtr& map)
{
  return;
}


void Mapping::LidarMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map)
{
  int width, height;
  int xi, xf, yi, yi; // ranges to copy to
  width = map->info.width;
  height = map->info.height;
  xi = (local_width/2) - (width/2);
  xf = (local_width/2) + (width/2);
  yi = local_height - height;
  yf = local_height;
  
  
  return;
}


void Mapping::RosUpdate()
{
  std::cout << "Hello ! " << std::endl;
  return;
}

