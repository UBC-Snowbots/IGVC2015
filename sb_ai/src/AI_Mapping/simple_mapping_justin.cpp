#include "simple_mapping_justin.hpp"

SimpleMapping::SimpleMapping() {

  _pose2DPublisher = _n.advertise<geometry_msgs::Pose2D>("ai_pose2D", 1000);
  _occupancyGridPublisher = _n.advertise<nav_msgs::OccupancyGrid("ai_occupancy_grid", 1000);

  _gpsSubscriber = _n.subscribe("ai_gps", 1000, gpsSubscriberCallback);

  _pose2DMsg.x = 0;
  _pose2DMsg.y = 0;
  _pose2DMsg.theta = 0;

}

SimpleMapping::~SimpleMapping() { 

}

void SimpleMapping::GPSSubscriberCallback(const geometry_msgs::Pose2D::ConstPtr& pose2DMsg) {

  _pose2DMsg.x = pose2DMsg.x;
  _pose2DMsg.y = pose2DMsg.y;
  _pose2DMsg.theta = pose2DMsg.theta;

}

void SimpleMapping::PublishPose2D() {

    _pose2DPublisher.publish(_pose2DMsg);

}

void SimpleMapping::PublishOccupanyGrid() {

  _occupancyGridPublisher.publish(_occupancyGridMsg);

}

