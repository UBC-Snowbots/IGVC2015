#include "generate_vision_map.hpp"
#include "math.h"

GenerateVisionMap::GenerateVisionMap(){

  _imageSubscriber = _n.subscribe("vision_node", 1000, VisionSubscriberCallback);

}

GenerateVisionMap::~GenerateVisionMap(){

}

void GenerateVisionMap::UpdateGlobalMapWithVisionData(nav_msgs::OccupancyGrid& globalMap){

  // Loop through the vision map
  for(int index = 0; index < _visionMapSize; index++) {

    // If there is an obstacle, then update the global map
    if(_imageMsg.data[index] == 1) {

      xGlobalVisionCoord = cos(_poseMsg.angle) * ConvertIndexToXCoord(index) - sin(_poseMsg.angle) * ConvertIndexToYCoord(index) +            globalMap.info.origin.position.x;
      yGlobalVisionCoord = sin(_poseMsg.angle) * ConvertIndexToXCoord(index) + cos(_poseMsg.angle) * ConvertIndexToYCoord(index) + globalMap.info.origin.position.y;

    }

    // Update global map with 1 to show that an obstacle exists
    globalMap.data[ConvertXYCoordToIndex(xGlobalVisionCoord, yGlobalVisionCoord, globalMap.data.info.width)] = 1;

  }

}

uint8_t GenerateVisionMap::ConvertIndexToXCoord(uint8_t index){

   return index % _imageMsg->width;

}

uint8_t GenerateVisionMap::ConvertIndexToXCoord(uint8_t index){

   return index / _imageMsg->width;

}

uint8_t GenerateVisionMap::ConvertXYCoordToIndex(uint8_t x, uint8_t y, uint8_t width) {

  return y * width + x;

}

void GenerateVisionMap::VisionSubscriberCallback(const sensor_msgs::Image::ConstPtr& imageMsg){

  _imageMsg.height = imageMsg->height; // Number of rows
  _imageMsg.width = imageMsg->width; // Number of columns
  _imageMsg.step = imageMsg->step;
  _mapSize = imageMsg->step * imageMsg->height;

  for(int index = 0; index < _mapSize; index++){

    _imageMsg.data[index] = imageMsg.data[index];

  }

}

void GenerateVisionMap::CompassSubscriberCallback(const int angle){
  
  _poseMsg.theta = angle; // probably isn't the angle of the robot w.r.t to the global map, need to fix this

}
