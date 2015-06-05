#include "BlitMap.h"

BlitMap::BlitMap(std::size_t w, std::size_t h)
{
  ourMap.data.assign(w*h, -1);
}

void BlitMap::blit(std::size_t xpos, std::size_t ypos, const nav_msgs::OccupancyGrid& input)
{
  std::size_t our_x = xpos, our_y = ypos;
  std::size_t input_x = 0;
  std::size_t input_y = 0;
  while(isOnMap(input_x,input_y,input)){
    input_x = our_x - xpos;
    input_y = our_y - ypos;

    addPoint(our_x, our_y, getPoint(input_x,input_y,input));
    
    if(input_y == input.info.width){
      our_x = xpos;
      our_y++;
    }
  }
}

void BlitMap::clear()
{
  ourMap.data.assign(ourMap.data.size(), -1);
}
  
const nav_msgs::OccupancyGrid& BlitMap::getMap()
{
  return ourMap;
}

void BlitMap::setPoint(std::size_t x, std::size_t y, Type data)
{
  ourMap.data.at(x + ourMap.info.width*y) = data;
}

void BlitMap::addPoint(std::size_t x, std::size_t y, Type data)
{
  Type ours = getPoint(x,y,ourMap);
  Type eventual = ours;
  if(ours==-1){
    eventual = data;
  }else if( (data>= 0) && (data <= 100) ){
    eventual = data > ours ? data: ours;
  }
  
  setPoint(x,y,eventual);
}

int BlitMap::getPoint(std::size_t x, std::size_t y, const nav_msgs::OccupancyGrid& map){
  return map.data.at(x + map.info.width*y);
}

bool BlitMap::isOnMap(std::size_t x, std::size_t y, const nav_msgs::OccupancyGrid& map)
{
  return (x + map.info.width*y) < map.data.size();
}

int main()
{
  return 0;
}

