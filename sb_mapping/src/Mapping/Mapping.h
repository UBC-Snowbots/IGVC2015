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
  
    // Map variables
    int v_width, v_height;
    int l_width, l_height;
    int v_l_offset; // offset of vision from lidar map
    int map_width, map_height;
    
    // Local maps
    nav_msgs::OccupancyGrid local_map, vision_map, lidar_map;
    
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
    

