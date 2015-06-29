#pragma once

#include "ros/ros.h"
#include <string>
#include <iostream>
#include <geometry_msgs/Twist.h>
#include "ControllerBase.hpp"
#include "sb_msgs/Waypoint.h"
#include "sb_msgs/Gps_info.h"
#include <math.h>
#include "sb_msgs/RobotState.h"
#include "std_msgs/String.h"
#include "sb_gps/Gps_Service.h"
#include "sb_msgs/MoveCommand.h"



#define GPS_SUB_TOPIC "GPS_USB_DATA"
#define COMPASS_SUB_TOPIC "robot_state"
#define GPS_PUB_TOPIC "GPS_DATA"
#define WAYPOINT_PUB_TOPIC "GPS_COORD"
#define GPS_SERV_TOPIC "GPS_SERVICE"
#define MOVE_COMMAND_TOPIC "move_command"

namespace ai
{
  class GpsController: public ai::ControllerBase
  {
  
  private:
  
    static const double EARTH_RADIUS = 6371000.0; 
    static const double PI = 3.14159265; 
		static const double MAG_DECL = 7.28; //Magnetic declination
  
    int angleCompass;
    bool moveStatus, goal, msg_flag;
    double d; //distance in metres from currentWaypoint to targetWaypoint
    int theta; //theta is the angle the robot needs to turn from currentWaypoint to targetWaypoint
		double roboSpeed;//speed at which robot is commanded to move at
    sb_msgs::Waypoint CurrentWaypoint, LastWaypoint, offWaypoint, buffWaypoint, avgWaypoint, TargetWaypoint, calibration;
    geometry_msgs::Twist twist_msg;
    sb_msgs::Gps_info pub_data; //Angle and distance are being published
    int next_move, prev_move, avg_count;
    long double buffer [10];
    
    ros::Subscriber gps_Sub;
    ros::Subscriber state_Sub;
    ros::Publisher gps_pub;
    ros::Publisher coord_pub;
    ros::ServiceClient client;
		sb_gps::Gps_Service srv;
		ros::Subscriber jaus;    
		bool rest;
    void StartGps();
    bool CheckGoal();
    int NextMoveLogic(double distance, int angle);
    geometry_msgs::Twist GetTwistMsg(int next_move);
    double CreateDistance();
    int CreateAngle();
    sb_msgs::Gps_info Createdata();
		void setWaypoints (sb_msgs::Waypoint& wp, double lon, double lat);
		void setWaypoints (sb_msgs::Waypoint& wp1, sb_msgs::Waypoint& wp2);
		void print (int color, const std::string &message);
		void print (int color, double value);
		void print (int color, const std::string &message, double value);
		void print (const std::string &message);
		void print (const std::string &message, double value);
		void print (int color, const std::string &message, double value, double value2);
		void calibrate();
		void calcwaypoint();
		void calcwaypoint(int i);
		void CommandReceiver(const sb_msgs::MoveCommand& command);
		void makeSpeed (double speed);
	  
  public:
  	
	  GpsController(ros::NodeHandle& nh);
	  void GpsCallback(const std_msgs::String::ConstPtr& msg); // gpsSubHandle
	  void CompassCallback(const sb_msgs::RobotState::ConstPtr& msg); // compassSubHandle
	  geometry_msgs::Twist Update();
  };
}

