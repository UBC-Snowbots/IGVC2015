#pragma once

#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Pose2D.h"

class LocalToGlobalMapping {

  private:

  

  public:

    LocalToGlobalMapping();

    ~LocalToGlobalMapping();

    nav_msgs::OccupancyGridTransform LocalToGlobal(const geometry_msgs::Pose2D	 globalLocationPtr, nav_msgs::OccupancyGrid& localMap);

};
