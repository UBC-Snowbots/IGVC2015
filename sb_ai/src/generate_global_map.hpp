#pragma once

#include "stdint.h"

#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Pose2D.h"
#include "geographic_msgs/GeoPoint.msg"
#include "std_msgs/Float32.msg"
#include "sensor_msgs/Image.h"

class GenerateGlobalMap {

  private:

    uint8_t ConvertIndexToXCoord(uint8_t index);

    uint8_t ConvertIndexToYCoord(uint8_t index);

    uint8_t ConvertXYCoordToIndex(uint8_t x, uint8_t y, uint8_t width);

    void LocalMapSubscriberCallback(const sensor_msgs::Image::ConstPtr& imageMsg);

    void GeoPointSubscriberCallback(const geographic_msgs::GeoPoint::ConstPtr& geoPointMsg);
 
    void AngleSubscriberCallback(const std_msgs::Float32::ConstPtr& angleMsg);

    ros::NodeHandle _n;

    ros::Subscriber _imageSubscriber;

    ros::Subscriber _geoPointSubscriber;

    sensor_msgs::Image _imageMsg;

    nav_msgs::OccupancyGrid _globalMap;

    uint32_t _localMapSize;

  public:

    GenerateGlobalMap();

    ~GenerateGlobalMap();

    void testDoSomething();

    void TransformLocalToGlobal();

};
