#pragma once
#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/Image.h"
#include <string>

#define VISION_SUB_TOPIC "visionSub"
#define LIDAR_SUB_TOPIC "lidarSub"
#define AI_PUB_TOPIC "aiPub"

class Mapping
{
  private:
  
    // Map sizes: width and height in terms of array sizes
    int local_width, local_height, local_size;
    int vision_width, vision_height, vision_size;
    int lidar_width, lidar_height, lidar_size;
    float final_resolution; // m/cell
    float lidar_pos_x, lidar_pos_y;
    float vision_pos_x, vision_pos_y;
    
    // Local maps
    nav_msgs::OccupancyGrid local_map, vision_map, lidar_map;
    
    // Subscribers, publishers
    ros::Subscriber vision_sub, lidar_sub;
    ros::Publisher ai_pub;
    
  public:
  
    // Initializer: vision/lidar displacement, estimated map size, resolution. 
    Mapping();
    
    // Callback for processing vision map
    // Converts image into occupancy grid. Also adjust resolution.
    void VisionMapCallback(const sensor_msgs::Image::ConstPtr& map);
    
    // Callback for lidar map 
    void LidarMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map);
    
    // update function for outputting combined local map
    void RosUpdate();
};
    

