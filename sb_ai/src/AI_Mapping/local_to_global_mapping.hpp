#pragma once

#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"

class LocalToGlobalMapping {

  private:

  

  public:

    LocalToGlobalMapping();

    ~LocalToGlobalMapping();

    nav_msgs::OccupancyGridTransform LocalToGlobal(nav_msgs::OccupancyGrid& localMap);

};
