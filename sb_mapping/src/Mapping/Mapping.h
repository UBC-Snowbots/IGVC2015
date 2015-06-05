#pragma once
#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/Image.h"
#include <string>

#define VISION_SUB_TOPIC "image_bird"
#define LIDAR_SUB_TOPIC "sb_lidar_map"
#define AI_PUB_TOPIC "local_map"

// Hardcoded values
#define MAP_WIDTH 240 
#define MAP_HEIGHT 120

class Mapping
{
  private:
  
    // Local maps
    nav_msgs::OccupancyGrid local_map, vision_map;
    
    // Subscribers, publishers
    ros::Subscriber vision_sub, lidar_sub;
    ros::Publisher ai_pub;
    
  public:
  
    // Initializer: vision/lidar displacement, estimated map size, resolution. 
    Mapping(ros::NodeHandle& nh);
    
    // Callback for processing vision map
    void VisionMapCallback(const sensor_msgs::Image::ConstPtr& map);
    
    // Callback for lidar map 
    void LidarMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map);
    
    // update function for outputting combined local map
    void RosUpdate();
    
};
    

