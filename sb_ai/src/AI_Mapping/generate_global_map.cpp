#include "generate_global_map.hpp"
#include "math.h"

static const std::string VISION_TOPIC = "vision_topic";
static const std::string WAYPOINT_TOPIC = "warpoint_topic";
static const std::string GPS_INFO_TOPIC = "gps_info_topic";


GenerateGlobalMap::GenerateGlobalMap()
{

  // Initialize subscribers
  _imageSubscriber = _n.subscribe(VISION_TOPIC, 1000, LocalMapSubscriberCallback);
  _waypointSubscriber = _n.subscribe(WAYPOINT_TOPIC, 1000, WaypointSubscriberCallback);
  _gpsInfoSubscriber = _n.subscribe(GPS_INFO_TOPIC, 1000, GPSInfoSubscriberCallback);

  // Initialize global map
  _globalMap.data.info.width = 100;
  _globalMap.data.info.height = 100;
  // _globalMap.data.data[_globalMap.data.info.width * _globalMap.data.info.height]; // Not sure about this...

}

GenerateGlobalMap::~GenerateGlobalMap(){

}


void GenerateGlobalMap::TransformLocalToGlobal(){

  // Loop through the vision map
  for(int index = 0; index < _visionMapSize; index++) {

    // If there is an obstacle, then update the global map
    if(_imageMsg.data[index] == 1) {

      float  xGlobalVisionCoord = cos(_poseMsg.angle) * ConvertIndexToXCoord(index) - sin(_poseMsg.angle) * ConvertIndexToYCoord(index) +            _globalMap.info.origin.position.x;
      float yGlobalVisionCoord = sin(_poseMsg.angle) * ConvertIndexToXCoord(index) + cos(_poseMsg.angle) * ConvertIndexToYCoord(index) + _globalMap.info.origin.position.y;

    }

    // Update global map with 1 to show that an obstacle exists
    _globalMap.data[ConvertXYCoordToIndex(xGlobalVisionCoord, yGlobalVisionCoord, _globalMap.data.info.width)] = 1;

  }

}

void GenerateGlobalMap::LocalMapSubscriberCallback(const sensor_msgs::Image::ConstPtr& imageMsg){

  _imageMsg.height = imageMsg->height; // Number of rows
  _imageMsg.width = imageMsg->width; // Number of columns
  _imageMsg.step = imageMsg->step;
  _mapSize = imageMsg->step * imageMsg->height; // no clue if this is right, according to the documentation it is... wtf, row * col makes more sense to me

  for(int index = 0; index < _mapSize; index++){

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

void GenerateGlobalMap::WaypointSubscriberCallback(const sb_msgs::ConstPtr&  waypointMsg){

  _globalMap.info.origin.position.x = waypointMsg->lon; // FIXME CONVERT STUFF TO X AND Y
  _globalMap.info.origin.position.y = waypointMsg->lat;

}


void GenerateGlobalMap::GPSInfoSubscriberCallback(const std_msgs::Float32::ConstPtr& gpsInfoMsg){
  
  _poseMsg.theta = gpsInfoMsg.angle;

}

void GenerateGlobalMap::testDoSomething() {

  printf("Did something");

}
