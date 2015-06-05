#pragma once

#include "stdint.h"
#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "sb_msgs/Gps_info.h"
#include "sb_msgs/Waypoint.h"

static const std::string VISION_TOPIC = "vision_topic";
static const std::string WAYPOINT_TOPIC = "warpoint_topic";
static const std::string GPS_INFO_TOPIC = "gps_info_topic";
static const std::string GLOBAL_MAP_TOPIC = "global_map_topic";

class GenerateGlobalMap {

private:

	uint8_t ConvertIndexToLocalXCoord(uint8_t index);

	uint8_t ConvertIndexToLocalYCoord(uint8_t index);

	uint8_t ConvertXYCoordToIndex(uint8_t x, uint8_t y, uint8_t width);

	// Gets the local map from vision
	void LocalMapSubscriberCallback(
			const nav_msgs::OccupancyGrid::ConstPtr& localMap);

	// Gets the longitude and latitude and converts to global x and y coords
	void WaypointSubscriberCallback(
			const sb_msgs::Waypoint::ConstPtr& waypointMsg);

	// Gets the real world angle
	void GPSInfoSubscriberCallback(
			const sb_msgs::Gps_info::ConstPtr& gpsInfoMsg);

	void CalculateMeterChangePerLongitude();

	void CalculateMeterChangePerLatitude();

	ros::NodeHandle _n;

	// Subscriber for the local map from vision
	ros::Subscriber _localMapSubscriber;

	// Subscriber for the gps longitude and latitude
	ros::Subscriber _waypointSubscriber;

	// Subscriber for the real world angle
	ros::Subscriber _gpsInfoSubscriber;

	// Publisher for the global map
	ros::Publisher _globalMapPublisher;

	// Stores the local map of the robot and the CURRENT POSITION in the global map
	nav_msgs::OccupancyGrid _localMap;

	// Stores the global map of the robot along with the ORIGIN in the global map
	nav_msgs::OccupancyGrid _globalMap;

	// Origin GPS coords of the robot at the start
	sb_msgs::Waypoint _gpsOrigin;

	// How many meters per degree of longitude
	uint32_t _meterChangePerLongitude;

	// How many meters per degree of latitude
	uint32_t _meterChangePerLatitude;

	// Course width in meters
	uint32_t _courseWidth;

	// Course height in meters
	uint32_t _courseHeight;

	uint32_t _globalMapSize;

	// Real world angle of robot -> same angle as the local map
	uint8_t _compassAngle;

	uint32_t _localMapSize;

public:

	GenerateGlobalMap(sb_msgs::Waypoint gpsOrigin, uint32_t courseWidth,
			uint32_t courseHeight, float mapResolution);

	~GenerateGlobalMap();

	void TransformLocalToGlobal();

	void testDoSomething();

};
