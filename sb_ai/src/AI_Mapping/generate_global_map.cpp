#include "generate_global_map.hpp"
#include "math.h"

uint8_t GLOBAL_MAP_PADDING = 100;

GenerateGlobalMap::GenerateGlobalMap(sb_msgs::Waypoint gpsOrigin, uint32_t courseWidth, uint32_t courseHeight, float mapResolution):
  _gpsOrigin(gpsOrigin) {

  // Calculate the meter change per longitude and latitude
  CalculateMeterChangePerLongitude();
  CalculateMeterChangePerLatitude();

  // Initialize subscribers
  _localMapSubscriber = _n.subscribe(VISION_TOPIC, 1000, &GenerateGlobalMap::LocalMapSubscriberCallback, this);
  _waypointSubscriber = _n.subscribe(WAYPOINT_TOPIC, 1000, &GenerateGlobalMap::WaypointSubscriberCallback, this);
  _gpsInfoSubscriber = _n.subscribe(GPS_INFO_TOPIC, 1000, &GenerateGlobalMap::GPSInfoSubscriberCallback, this);

  // Initialize publishers
  _globalMapPublisher = _n.advertise<nav_msgs::OccupancyGrid>(GLOBAL_MAP_TOPIC, 1000);

  // Initialize global map
  _globalMap.info.width = courseWidth + GLOBAL_MAP_PADDING;
  _globalMap.info.height = courseHeight + GLOBAL_MAP_PADDING;
  _globalMap.info.resolution = mapResolution;
  _globalMapSize = _globalMap.info.width * _globalMap.info.height;
  _globalMap.data.assign(_globalMapSize,0);
  // Assume robot starts somewhere in the middle bottom of the course  
  _globalMap.info.origin.position.x = _globalMap.info.width / 2; 
  _globalMap.info.origin.position.y = _globalMap.info.height * 3 / 4;

  _globalMapAngle = 0; // pretty much useless because the global map is oriented the same way GPS is oriented, so NWSE

}

GenerateGlobalMap::~GenerateGlobalMap(){

}


void GenerateGlobalMap::TransformLocalToGlobal(){

  uint8_t xGlobalVisionCoord;
  uint8_t yGlobalVisionCoord;

  // Loop through the vision map
  for(int index = 0; index < _localMapSize; index++) {

      xGlobalVisionCoord = cos(_compassAngle - _globalMapAngle) * ConvertIndexToLocalXCoord(index) - sin(_compassAngle - _globalMapAngle) * ConvertIndexToLocalYCoord(index) + _localMap.info.origin.position.x;
      yGlobalVisionCoord = sin(_compassAngle - _globalMapAngle) * ConvertIndexToLocalXCoord(index) + cos(_compassAngle - _globalMapAngle) * ConvertIndexToLocalYCoord(index) + _localMap.info.origin.position.y;

    // Update global map with 0/1 to show that an obstacle dne/exists
    _globalMap.data[ConvertXYCoordToIndex(xGlobalVisionCoord, yGlobalVisionCoord, _globalMap.info.width)] = _localMap.data[index];

  }

}
void GenerateGlobalMap::CalculateMeterChangePerLongitude() {

  _meterChangePerLongitude = 111412.84 * cos(_gpsOrigin.lat) - 93.5 * cos(3 * _gpsOrigin.lat) - 0.118 * cos(5 * _gpsOrigin.lat);

}

void GenerateGlobalMap::CalculateMeterChangePerLatitude() {

  _meterChangePerLatitude = 111132.92 - 559.82 * cos(2 * _gpsOrigin.lat) + 1.175 * cos(4 * _gpsOrigin.lat) - 0.0023 * cos(6 * _gpsOrigin.lat);

}

void GenerateGlobalMap::LocalMapSubscriberCallback(const nav_msgs::OccupancyGrid::ConstPtr& localMap){

  ROS_INFO("LocalMapSubscriberCallback doing stuff");
  _localMap.info.height = localMap->info.height; // Number of rows
  _localMap.info.width = localMap->info.width; // Number of columns
  _localMapSize = localMap->info.width * localMap->info.height;

  for(int index = 0; index < _localMapSize; index++){

    _localMap.data[index] = localMap->data[index];

  }

}

uint8_t GenerateGlobalMap::ConvertIndexToLocalXCoord(uint8_t index){

   return index % _localMap.info.width;

}

uint8_t GenerateGlobalMap::ConvertIndexToLocalYCoord(uint8_t index){

   return index / _localMap.info.width;

}

uint8_t GenerateGlobalMap::ConvertXYCoordToIndex(uint8_t x, uint8_t y, uint8_t width) {

  return y * width + x;

}

void GenerateGlobalMap::WaypointSubscriberCallback(const sb_msgs::Waypoint::ConstPtr& waypointMsg){

  ROS_INFO("WaypointSubscriberCallback doing stuff");
  _localMap.info.origin.position.x = waypointMsg->lon; // FIXME CONVERT STUFF TO X AND Y
  _localMap.info.origin.position.y = waypointMsg->lat;

}


void GenerateGlobalMap::GPSInfoSubscriberCallback(const sb_msgs::Gps_info::ConstPtr& gpsInfoMsg){
  
  ROS_INFO("GPSInfoSubscriberCallback doing stuff");
  _compassAngle = gpsInfoMsg->angle;

}

void GenerateGlobalMap::testDoSomething() {

 ROS_INFO("Did something");

}
