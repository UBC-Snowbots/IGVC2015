#pragma once

#include "stdint.h"
#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "sb_msgs/Gps_info.h"
#include "sb_msgs/Waypoint.h"

static const std::string VISION_TOPIC = "vision_topic";
static const std::string WAYPOINT_TOPIC = "warpoint_topic";
static const std::string GPS_INFO_TOPIC = "gps_info_topic";

class GenerateGlobalMap {

  private:

    uint8_t ConvertIndexToLocalXCoord(uint8_t index);

    uint8_t ConvertIndexToLocalYCoord(uint8_t index);

    uint8_t ConvertXYCoordToIndex(uint8_t x, uint8_t y, uint8_t width);

    // Gets the local map from vision
    void LocalMapSubscriberCallback(const nav_msgs::OccupancyGrid::ConstPtr& localMap);

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
    nav_msgs::OccupancyGrid _localMap;

    // Stores the global map of the robot along with it's position in the global map
    nav_msgs::OccupancyGrid _globalMap;
    
    uint32_t _globalMapSize;

    // Real world angle of robot
    uint8_t _compassAngle;

    // Orientation angle of the global map
    uint8_t _globalMapAngle;

    uint32_t _localMapSize;

  public:

    GenerateGlobalMap();

    ~GenerateGlobalMap();

    void TransformLocalToGlobal();

    void testDoSomething();


};
