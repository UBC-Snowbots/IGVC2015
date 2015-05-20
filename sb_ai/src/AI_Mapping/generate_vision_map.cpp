#include "generate_vision_map.hpp"

GenerateVisionMap::LocalToGlobalMapping(){

  _imageSubscriber = _n.subscribe("vision_node", 1000, VisionSubscriberCallback);

}

GenerateVisionMap::~LocalToGlobalMapping(){

}

nav_msgs::OccupancyGrid GenerateVisionMap::LocalToGlobal(nav_msgs::OccupancyGrid& localMap){

  

}

uint8_t ConvertIndexToXCoord(uint8 index){

   return index / _imageMsg->width;

}

uint8_t ConvertIndexToXCoord(uint8 index){

   return index % _imageMsg->width;

}

void VisionSubscriberCallback(const sensor_msgs::Image::ConstPtr& imageMsg){

  _imageMsg->height = imageMsg->height;
  _imageMsg->width = imageMsg->width;
  _imageMsg->data = imageMsg->data;

}
