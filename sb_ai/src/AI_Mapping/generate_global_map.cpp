#include "generate_global_map.hpp"
#include "math.h"


GenerateGlobalMap::GenerateGlobalMap()
{

  // Initialize subscribers
  _imageSubscriber = _n.subscribe(VISION_TOPIC, 1000, &GenerateGlobalMap::LocalMapSubscriberCallback, this);
  _waypointSubscriber = _n.subscribe(WAYPOINT_TOPIC, 1000, &GenerateGlobalMap::WaypointSubscriberCallback, this);
  _gpsInfoSubscriber = _n.subscribe(GPS_INFO_TOPIC, 1000, &GenerateGlobalMap::GPSInfoSubscriberCallback, this);

  // Initialize global map
  _globalMap.info.width = 100;
  _globalMap.info.height = 100;
  _globalMapSize = 100*100;
  // _globalMap.data.data[_globalMap.data.info.width * _globalMap.data.info.height]; // Not sure about this...

}

GenerateGlobalMap::~GenerateGlobalMap(){

}


void GenerateGlobalMap::TransformLocalToGlobal(){

  uint8_t xGlobalVisionCoord;
  uint8_t yGlobalVisionCoord;

  // Loop through the vision map
  for(int index = 0; index < _localMapSize; index++) {

      xGlobalVisionCoord = cos(_compassAngle - _globalMapAngle) * ConvertIndexToLocalXCoord(index) - sin(_compassAngle - _globalMapAngle) * ConvertIndexToLocalYCoord(index) +            _globalMap.info.origin.position.x;
      yGlobalVisionCoord = sin(_compassAngle - _globalMapAngle) * ConvertIndexToLocalXCoord(index) + cos(_compassAngle - _globalMapAngle) * ConvertIndexToLocalYCoord(index) + _globalMap.info.origin.position.y;

    // Update global map with 0/1 to show that an obstacle dne/exists
    _globalMap.data[ConvertXYCoordToIndex(xGlobalVisionCoord, yGlobalVisionCoord, _globalMap.info.width)] = _localMap.data[index];

  }

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
  _globalMap.info.origin.position.x = waypointMsg->lon; // FIXME CONVERT STUFF TO X AND Y
  _globalMap.info.origin.position.y = waypointMsg->lat;

}


void GenerateGlobalMap::GPSInfoSubscriberCallback(const sb_msgs::Gps_info::ConstPtr& gpsInfoMsg){
  
  ROS_INFO("GPSInfoSubscriberCallback doing stuff");
  _compassAngle = gpsInfoMsg->angle;

}

void GenerateGlobalMap::testDoSomething() {

 ROS_INFO("Did something");

}
