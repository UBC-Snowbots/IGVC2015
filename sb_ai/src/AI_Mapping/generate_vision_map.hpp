#pragma once

#include "stdint.h"

#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Pose2D.h"
#include "sensor_msgs/Image.h"

class GenerateVisionMap {

  private:

    uint8_t ConvertIndexToXCoord(uint8 index);

    uint8_t ConvertIndexToYCoord(uint8 index);

    void VisionSubscriberCallback(const sensor_msgs::Image::ConstPtr& imageMsg);

    ros::NodeHandle _n;

    ros::Subscriber _imageSubscriber;

    sensor_msgs::Image _imageMsg;

  public:

    GenerateVisionMap();

    ~GenerateVisionMap();

    nav_msgs::OccupancyGrid LocalToGlobal(const geometry_msgs::Pose2D globalLocationPtr, nav_msgs::OccupancyGrid& localMap);

};
