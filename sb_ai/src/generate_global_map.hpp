#pragma once

#include "stdint.h"

#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Pose2D.h"
#include "sensor_msgs/Image.h"

class GenerateGlobalMap {

  private:

    uint8_t ConvertIndexToXCoord(uint8_t index);

    uint8_t ConvertIndexToYCoord(uint8_t index);

    uint8_t ConvertXYCoordToIndex(uint8_t x, uint8_t y, uint8_t width);

    void LocalMapSubscriberCallback(const sensor_msgs::Image::ConstPtr& imageMsg);
 
    void CompassSubscriberCallback(const int angle);

    ros::NodeHandle _n;

    ros::Subscriber _imageSubscriber;

    sensor_msgs::Image _imageMsg;

    geometry_msgs::Pose2D _poseMsg;

    nav_msgs::OccupancyGrid _globalMap;

    uint32_t _localMapSize;

  public:

    GenerateGlobalMap();

    ~GenerateGlobalMap();

    void testDoSomething();

    void TransformLocalToGlobal(const geometry_msgs::Pose2D globalLocationPtr, nav_msgs::OccupancyGrid& localMap);

};
