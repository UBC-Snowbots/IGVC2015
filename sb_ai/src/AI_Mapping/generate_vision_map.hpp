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

    uint8_t ConvertXYCoordToIndex(uint8_t x, uint8_t y, uint8_t width);

    void VisionSubscriberCallback(const sensor_msgs::Image::ConstPtr& imageMsg);
 
    void CompassSubscriberCallback(const int angle);

    ros::NodeHandle _n;

    ros::Subscriber _imageSubscriber;

    sensor_msgs::Image _imageMsg;

    geometry_msgs::Pose2D _poseMsg;

    uint32_t _visionMapSize;

  public:

    GenerateVisionMap();

    ~GenerateVisionMap();

    void UpdateGlobalMapWithVisionData(const geometry_msgs::Pose2D globalLocationPtr, nav_msgs::OccupancyGrid& localMap);

};
