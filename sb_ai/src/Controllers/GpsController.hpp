#pragma once

#include "ros/ros.h"
#include <string>
#include <iostream>
#include <geometry_msgs/Twist.h>
#include "ControllerBase.hpp"
#include "sb_msgs/Waypoint.h"
#include "sb_msgs/Gps_info.h"
#include "Controllers/sb_ai.h"
#include <math.h>
#include "sb_msgs/compass.h"

#define PI 3.14159265

namespace ai
{
  class GpsController: public ai::ControllerBase
  {
  
  private:
  
    double angleCompass;
    bool moveStatus;
    bool goal;
    bool msg_flag = false;
    double d = 0; //distance in metres from currentWaypoint to targetWaypoint
    double theta = 0; //theta is the angle the robot needs to turn from currentWaypoint to targetWaypoint
    sb_msgs::Waypoint CurrentWaypoint,LastWaypoint,off, buffWaypoint, avgWaypoint;
    geometry_msgs::Twist twist_msg;
    sb_msgs::Gps_info pub_data; //Angle and distance are being published
    int next_move, prev_move = 0;
    sb_msgs::Waypoint TargetWaypoint;
    static double const EARTH_RADIUS = 6378.137;
    long double buffer [10];
    
    void StartGps();
    bool CheckGoal();
    int NextMoveLogic (double distance, double angle);
    geometry_msgs::Twist GetTwistMsg(int next_move);
    double CreateDistance();
    double CreateAngle();
    sb_msgs::Gps_info Createdata();

	  
  public:
  
	  GpsController(ros::NodeHandle& nh);
	  void GpsCallback(const std_msgs::String::ConstPtr& msg); // gpsSubHandle
	  void CompassCallback(const sb_msgs::compass::ConstPtr& msg); // compassSubHandle
	  geometry_msgs::Twist Update();
  };
}

