#pragma once

#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "nav_msgs/OccupancyGrid.h"

/**
  * Inputs:
  *   
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

    // Tells us the robot location and orientation in the global coord system
    void GPSSubscriberCallback(const geometry_msgs::Pose2D::ConstPtr& pose2DMsg);

    // Publish the current global location and orientation
    void PublishPose2D();

    void PublishOccupanyGrid();
    

};

