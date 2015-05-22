#include "generate_vision_map.hpp"
#include "math.h"

static const string VISION_TOPIC = "vision_topic";
static const string GEO_POINT_TOPIC = "geo_point_topic";
static const string ANGLE_TOPIC = "angle_topic";

GenerateGlobalMap::GenerateGlobalMap(){

  _imageSubscriber = _n.subscribe(VISION_TOPIC, 1000, LocalMapSubscriberCallback);
  _geoPointSubscriber = _n.subscribe(GPS_TOPIC, 1000, GeoPointSubscriberCallback);
  _angleSubscriber = _n.subscribe(GPS_TOPIC, 1000, AngleSubscriberCallback);

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

      xGlobalVisionCoord = cos(_poseMsg.angle) * ConvertIndexToXCoord(index) - sin(_poseMsg.angle) * ConvertIndexToYCoord(index) +            _globalMap.info.origin.position.x;
      yGlobalVisionCoord = sin(_poseMsg.angle) * ConvertIndexToXCoord(index) + cos(_poseMsg.angle) * ConvertIndexToYCoord(index) + _globalMap.info.origin.position.y;

    }

    // Update global map with 1 to show that an obstacle exists
    _globalMap.data[ConvertXYCoordToIndex(xGlobalVisionCoord, yGlobalVisionCoord, _globalMap.data.info.width)] = 1;

  }

}

uint8_t GenerateGlobalMap::ConvertIndexToXCoord(uint8_t index){

   return index % _imageMsg->width;

}

uint8_t GenerateGlobalMap::ConvertIndexToXCoord(uint8_t index){

   return index / _imageMsg->width;

}

uint8_t GenerateGlobalMap::ConvertXYCoordToIndex(uint8_t x, uint8_t y, uint8_t width) {

  return y * width + x;

}

void GenerateGlobalMap::LocalMapSubscriberCallback(const sensor_msgs::Image::ConstPtr& imageMsg){

  _imageMsg.height = imageMsg->height; // Number of rows
  _imageMsg.width = imageMsg->width; // Number of columns
  _imageMsg.step = imageMsg->step;
  _mapSize = imageMsg->step * imageMsg->height; // no clue if this is right, according to the documentation it is... wtf, row * col makes more sense to me

  for(int index = 0; index < _mapSize; index++){

    _imageMsg.data[index] = imageMsg.data[index];

  }

}

void GenerateGlobalMap::GeoPointSubscriberCallback(const geographic_msgs::GeoPoint::ConstPtr& geoPointMsg){

  _globalMap.info.origin.position.x = pose2DMsg->x; // FIXME CONVERT STUFF TO X AND Y
  _globalMap.info.origin.position.y = pose2DMsg->y;

}

void GenerateGlobalMap::AngleSubscriberCallback(const std_msgs::Float32::ConstPtr& angleMsg){
  
  _poseMsg.theta = angleMsg.data;

}

void GenerateGlobalMap::testDoSomething() {

  printf("Did something");

}
