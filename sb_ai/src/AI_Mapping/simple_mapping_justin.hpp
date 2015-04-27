#pragma once

#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "nav_msgs/OccupancyGrid.h"

class SimpleMapping{

  private:

    ros::NodeHandle _n;

    ros::Publisher _pose2DPublisher;

    ros::Publisher _occupancyGridPublisher;

    ros::Subscriber _gpsSubscriber;

    geometry_msgs::Pose2D _pose2DMsg;

    nav_msgs::OccupancyGrid _occupancyGridMsg;

  public:
  
    SimpleMapping();

    ~SimpleMapping();

    void GPSSubscriberCallback(const geometry_msgs::Pose2D::ConstPtr& pose2DMsg);

    void PublishPose2D();

    void PublishOccupanyGrid();
    

}

