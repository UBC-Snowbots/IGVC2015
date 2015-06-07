#include "nav_msgs/OccupancyGrid.h"

class BlitMap{
  public:
  typedef int Type;
  BlitMap(std::size_t w, std::size_t h);
  // copies data onto this map, with the input at position (x,y)
  void blit(std::size_t x, std::size_t y, const nav_msgs::OccupancyGrid& input);
  const nav_msgs::OccupancyGrid& getMap();
  void clear();
  private:
  void setPoint(std::size_t x, std::size_t y, Type data);
  void addPoint(std::size_t x, std::size_t y, Type data);
  int getPoint(std::size_t x, std::size_t y, const nav_msgs::OccupancyGrid& map);
  static bool isOnMap(std::size_t x, std::size_t y, const nav_msgs::OccupancyGrid& map);
  nav_msgs::OccupancyGrid ourMap;
};
