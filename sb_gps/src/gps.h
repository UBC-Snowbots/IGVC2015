#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include "sb_msgs/CarCommand.h"
#include <sstream>
#include <iostream>
#include <fstream>
#include <string>
#include <math.h>
#include <stdlib.h>

// REMEMBER TO EDIT WAYPOINTS FILE DIRECTORY TODO

using namespace std;

struct waypoint 
{
	int long_x;
	int lat_y;
};

// functions
void GetWaypoint();
void CalculateAngle(); 
int* ReturnWaypoints();
void CheckWaypointStatus();
void steeringTest();

// variables
static const string GPS_NODE_NAME = "gps_node"; 
static const string GPS_OUTPUT_TOPIC = "gps_nav"; 
static const string GPS_TEST_TOPIC = "vision_vel"; // test sub
static const string GPS_INPUT_TOPIC = "gps_state"; // gps_state
//static const int SECOND = 1000000;
bool isAtGoal = false, isFinished = false; 
double lat, lon, angle, last_angle = 0, steering;
struct waypoint current_waypoint;
struct waypoint goal_waypoint;
struct waypoint last_waypoint;
int* waypoints_list = ReturnWaypoints();
int size = sizeof(waypoints_list), c = 0, x_dist, y_dist;
float x;
float y;
float dx;
float dy;
int RunCount = 0;

