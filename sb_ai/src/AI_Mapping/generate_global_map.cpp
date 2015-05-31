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
  _visionMapSize = 100*100;
  // _globalMap.data.data[_globalMap.data.info.width * _globalMap.data.info.height]; // Not sure about this...

}

GenerateGlobalMap::~GenerateGlobalMap(){

}


void GenerateGlobalMap::TransformLocalToGlobal(){

  uint8_t xGlobalVisionCoord;
  uint8_t yGlobalVisionCoord;

  // Loop through the vision map
  for(int index = 0; index < _localMapSize; index++) {

      xGlobalVisionCoord = cos(_compassAngle) * ConvertIndexToXCoord(index) - sin(_compassAngle) * ConvertIndexToYCoord(index) +            _globalMap.info.origin.position.x;
      yGlobalVisionCoord = sin(_compassAngle) * ConvertIndexToXCoord(index) + cos(_compassAngle) * ConvertIndexToYCoord(index) + _globalMap.info.origin.position.y;

    // Update global map with 0/1 to show that an obstacle dne/exists
    _globalMap.data[ConvertXYCoordToIndex(xGlobalVisionCoord, yGlobalVisionCoord, _globalMap.info.width)] = _imageMsg.data[index];

  }

}

void GenerateGlobalMap::LocalMapSubscriberCallback(const sensor_msgs::Image::ConstPtr& imageMsg){

  _imageMsg.height = imageMsg->height; // Number of rows
  _imageMsg.width = imageMsg->width; // Number of columns
  _imageMsg.step = imageMsg->step;
  _localMapSize = imageMsg->width * imageMsg->height;

  for(int index = 0; index < _localMapSize; index++){

    _imageMsg.data[index] = imageMsg->data[index];

  }

}

uint8_t GenerateGlobalMap::ConvertIndexToXCoord(uint8_t index){

   return index % _imageMsg.width;

}

uint8_t GenerateGlobalMap::ConvertIndexToYCoord(uint8_t index){

   return index / _imageMsg.width;

}

uint8_t GenerateGlobalMap::ConvertXYCoordToIndex(uint8_t x, uint8_t y, uint8_t width) {

  return y * width + x;

}

void GenerateGlobalMap::WaypointSubscriberCallback(const sb_msgs::Waypoint::ConstPtr& waypointMsg){

  _globalMap.info.origin.position.x = waypointMsg->lon; // FIXME CONVERT STUFF TO X AND Y
  _globalMap.info.origin.position.y = waypointMsg->lat;

}


void GenerateGlobalMap::GPSInfoSubscriberCallback(const sb_msgs::Gps_info::ConstPtr& gpsInfoMsg){
  
  _compassAngle = gpsInfoMsg->angle;

}

void GenerateGlobalMap::testDoSomething() {

  printf("Did something");

}
