#include "simple_mapping_justin.hpp"
#include "sb_msgs/AISimGPS.h"

SimpleMapping::SimpleMapping() {

  _pose2DPublisher = _n.advertise<geometry_msgs::Pose2D>("ai_pose2D", 1000);
  _occupancyGridPublisher = _n.advertise<nav_msgs::OccupancyGrid("ai_occupancy_grid", 1000);

  _gpsSubscriber = _n.subscribe("ai_gps", 1000, GPSSubscriberCallback);

  _pose2DMsg.x = 0;
  _pose2DMsg.y = 0;
  _pose2DMsg.theta = 0;

}

SimpleMapping::~SimpleMapping() { 

}

void SimpleMapping::GPSSubscriberCallback(const sb_msgs::AISimGPS::ConstPtr& AISimGPSMsg) {

  _pose2DMsg.x = AISimGPSMsg.latitude; // Need to convert from latitude to x and y
  _pose2DMsg.y = AISimGPSMsg.longitude;
  _pose2DMsg.theta = AISimGPSMsg.orientation;

}

void SimpleMapping::PublishPose2D() {

  _pose2DPublisher.publish(_pose2DMsg);

}

void SimpleMapping::PublishOccupanyGrid() {

  _occupancyGridPublisher.publish(_occupancyGridMsg);

}

