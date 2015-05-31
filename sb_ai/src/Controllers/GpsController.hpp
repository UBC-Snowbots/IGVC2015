#pragma once

#include "ros/ros.h"
#include <string>
#include <iostream>
#include <geometry_msgs/Twist.h>
#include "ControllerBase.hpp"
#include "sb_msgs/Waypoint.h"
#include "sb_msgs/Gps_info.h"
#include <math.h>
#include "sb_msgs/compass.h"
#include "std_msgs/String.h"



#define GPS_SUB_TOPIC "GPS_USB_DATA"
#define COMPASS_SUB_TOPIC "robot_state"
#define GPS_PUB_TOPIC "GPS_DATA"
#define WAYPOINT_PUB_TOPIC "GPS_COORD"
#define GPS_SERV_TOPIC "GPS_SERVICE"

namespace ai
{
  class GpsController: public ai::ControllerBase
  {
  
  private:
  
    static const double EARTH_RADIUS = 6378.137;
    static const double PI = 3.14159265;
  
    double angleCompass;
    bool moveStatus, goal, msg_flag, calibrate;;
    double d; //distance in metres from currentWaypoint to targetWaypoint
    double theta; //theta is the angle the robot needs to turn from currentWaypoint to targetWaypoint
    sb_msgs::Waypoint CurrentWaypoint, LastWaypoint, off, buffWaypoint, avgWaypoint, TargetWaypoint;
    geometry_msgs::Twist twist_msg;
    sb_msgs::Gps_info pub_data; //Angle and distance are being published
    int next_move, prev_move, avg_count;
    long double buffer [10];
    
    ros::Subscriber gps_Sub;
    ros::Subscriber compass_Sub;
    ros::Publisher gps_pub;
    ros::Publisher coord_pub;
    ros::ServiceClient client;
    
    void StartGps();
    bool CheckGoal();
    int NextMoveLogic(double distance, double angle);
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

