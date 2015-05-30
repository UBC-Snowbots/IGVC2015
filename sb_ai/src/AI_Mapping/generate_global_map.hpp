#pragma once

#include "stdint.h"
#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Pose2D.h"
#include "sensor_msgs/Image.h"
#include "sb_msgs/Gps_info.h"
#include "sb_msgs/Waypoint.h"

static const std::string VISION_TOPIC = "vision_topic";
static const std::string WAYPOINT_TOPIC = "warpoint_topic";
static const std::string GPS_INFO_TOPIC = "gps_info_topic";

class GenerateGlobalMap {

  private:

    uint8_t ConvertIndexToXCoord(uint8_t index);

    uint8_t ConvertIndexToYCoord(uint8_t index);

    uint8_t ConvertXYCoordToIndex(uint8_t x, uint8_t y, uint8_t width);

    // Gets the local map from vision
    void LocalMapSubscriberCallback(const sensor_msgs::Image::ConstPtr& imageMsg);

    // Gets the longitude and latitude and converts to global x and y coords
    void WaypointSubscriberCallback(const sb_msgs::Waypoint::ConstPtr& waypointMsg);
 
    // Gets the real world angle
    void GPSInfoSubscriberCallback(const sb_msgs::Gps_info::ConstPtr& gpsInfoMsg);

    ros::NodeHandle _n;

    // Subscriber for the local map from vision
    ros::Subscriber _imageSubscriber;

    // Subscriber for the gps longitude and latitude
    ros::Subscriber _waypointSubscriber;

    // Subscriber for the real world angle
    ros::Subscriber _gpsInfoSubscriber;

    // Stores the local map of the robot
    sensor_msgs::Image _imageMsg;

    // Stores the angle of the robot
    geometry_msgs::Pose2D _poseMsg;

    // Stores the global map of the robot along with it's position in the global map
    nav_msgs::OccupancyGrid _globalMap;
    
    uint32_t _visionMapSize;

    uint32_t _localMapSize;
    
    int _mapSize;

  public:

    GenerateGlobalMap();

    ~GenerateGlobalMap();

    void TransformLocalToGlobal();

    void testDoSomething();


};
