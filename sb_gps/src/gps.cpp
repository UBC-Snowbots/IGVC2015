/* Author: Vincent Yuan
*  Date: Jan 3, 2015
*  Purpose: Main GPS function node
*  Function: Collect, parse NMEA data, create twist message, r, theta
*/

//Standard Headers
#include <stdlib.h>
#include <math.h>
#include <iostream> //for cout, rather that using stdio
#include <fstream> //read from file
#include <sstream>
#include <ros/ros.h> //for ros system
#include <std_msgs/String.h>
#include <string.h>
#include <geometry_msgs/Twist.h>
#include "sb_msgs/CarCommand.h"
#include "sb_gps/GotoWaypoint.h"
#include "sb_msgs/Waypoint.h"

//Constants
#define EARTH_RADIUS 6378.137 //In KM
#define PI 3.14159265

using namespace std;

struct waypoint {
  double lon;
  double lat;
  waypoint(){
    lon=lat=0;
  }
};

// Variables

double *NMEA; //To hold received suscription message
double angleCompass;
bool moveStatus;
bool goal;
bool gpsFlag = true; //indicates connection from gps chip
double d = 0; //distance in metres from currentWaypoint to targetWaypoint
double theta = 0; //theta is the angle the robot needs to turn from currentWaypoint to targetWaypoint
waypoint CurrentWaypoint,TargetWaypoint,LastWaypoint;
geometry_msgs::Twist nextTwist;


static const string GPS_NODE_NAME = "gps_node";
static const string GPS_OUTPUT_TOPIC = "gps_nav";
static const string GPS_TEST_TOPIC = "vision_vel"; // test sub
static const string GPS_INPUT_TOPIC = "gps_state"; // gps_state
static const string NODE_NAME = "sb_gps";
static const string PUBLISH_TOPIC = "gps_twist";
static const string GPS_INPUT_DIRECTION = "waypoint";
static const string SERVICE_NAME = "goto_waypoint";

//static int LOOP_FREQ = 30;

bool goto_waypoint(sb_gps::GotoWaypoint::Request &req, sb_gps::GotoWaypoint::Response &res);
void gpsSubHandle(const std_msgs::String::ConstPtr& msg);
geometry_msgs::Twist createNextTwist(geometry_msgs::Twist nextTwist);
bool checkGoal (waypoint CurrentWayPoint, waypoint TargetWayPoint);
double createAngle(double compassAngle);
double createDistance (void);

int main (int argc, char **argv){



  ros::init(argc, argv, GPS_NODE_NAME); //initialize access point to communicate
  ros::NodeHandle nh; //create handle to this process node, NodeHandle is main access point to communication with ROS system. First one intializes node
  ros::Publisher gps_test_pub = nh.advertise<geometry_msgs::Twist>(GPS_TEST_TOPIC, 20);
  ros::Publisher gps_data_pub = nh.advertise<sb_msgs::CarCommand>(GPS_OUTPUT_TOPIC, 20);
  ros::Subscriber gps_Sub = nh.subscribe(GPS_INPUT_TOPIC, 20, gpsSubHandle);
  ros::ServiceServer service = nh.advertiseService(SERVICE_NAME, goto_waypoint);//finish from sb_WaypointManager

  ros::Rate loop_rate(5); //10hz loop rate


	while (ros::ok()){
    ROS_INFO("Everything is going to be ok");

		while (gpsFlag == true){

        createDistance ();
  			if(checkGoal (CurrentWaypoint, TargetWaypoint) || moveStatus){
            nextTwist.linear.x = 0;
            nextTwist.linear.y = 0;
            nextTwist.linear.z = 0;

            nextTwist.angular.x = 0;
            nextTwist.angular.y = 0;
            nextTwist.angular.z = 0;

            if (moveStatus)
            ROS_INFO("Command stop");
            else
            ROS_INFO("Current Position:");
            cout << "Current longitude: " << CurrentWaypoint.lon << " Current latitude: " << CurrentWaypoint.lat << endl;
            ROS_INFO("Target Position:");
            cout << "Target  longitude: " << TargetWaypoint.lon  << " Target  latitude: " << TargetWaypoint.lat  << endl;
            cout << "Arrived at destination" << endl;
  				}
  			else{
            theta = createAngle(angleCompass);
            nextTwist = createNextTwist (nextTwist); //Make new twist message
            ROS_INFO("Current Position:");
            cout << "Current longitude: " << CurrentWaypoint.lon << " Current latitude: " << CurrentWaypoint.lat << endl;
            ROS_INFO("Twist lin.x = %f, Twist lin.y = %f, Twist lin.z = %f", nextTwist.linear.x, nextTwist.linear.y, nextTwist.linear.z);
            ROS_INFO("Twist ang.x = %f, Twist ang.y = %f, Twist ang.z = %f", nextTwist.angular.x, nextTwist.angular.y, nextTwist.angular.z);
            cout << "Angle to destination" << theta << endl;
  		    }

        gps_test_pub.publish(nextTwist); /*tdo: change name of publisher_name*/
        ros::spinOnce(); //ros spin is to ensure that ros threading does not leave suscribes un processed
        loop_rate.sleep();
      }
    }
    return 0;
}

bool goto_waypoint(sb_gps::GotoWaypoint::Request &req, sb_gps::GotoWaypoint::Response &res){
  res.at_location = false;
  ROS_INFO("request: latitude= %i, longitude = %i", req.location.lat, req.location.lon);//Waypointmanager request
  if (req.move){
    ROS_INFO("request: move = true");
    moveStatus = true;
  }
  else{
    ROS_INFO("request: move = false");
    moveStatus = false;
    //ROS_INFO("sending back response: []", res);
  }
  return true;
}

void gpsSubHandle(const std_msgs::String::ConstPtr& msg){
  /*
 	Input Parameter: Suscribed Message
 	Output: void
 		1. Pointer
 	Purpose: Handles suscribed gps Message, check for NMEA, Parse
 	Notes: Copied from last years' code
  */
 	char a[64];
 	char temp[8];
 	int x, y, i = 0, j;
 	string str = msg->data;

 	while (str[i] != '\0') { a[i] = str[i]; i++; }
 	if (a[0] != 'B' && a[1] != ',') { cout << "no B" << endl; return; }
 	if( a[10] != 'e' && a[24] != 'e') { cout << "no e" << endl; return; }
 	if( a[14] != ',' && a[28] != ',') { cout << "no comma" << endl; return; }
 	if( a[33] != '.') { cout << "no compass" << endl; return; }

 	//processing x coord
 	for (i = 2, j=0; i<9 || j<7; i++, j++) {
 		if (a[i]=='.') { j--; }
 		else { temp[j] = a[i]; }
 	}
 	LastWaypoint.lon = CurrentWaypoint.lon;
 	CurrentWaypoint.lon = atoi(temp);

 	//processing y coord
 	for (i=15, j=0; i<10 || j<8; i++, j++) {
 		if (a[i] == '.') { j--; }
 		else { temp[j] = a[i]; }
 	}
 	LastWaypoint.lat = CurrentWaypoint.lat;
 	CurrentWaypoint.lat = atoi(temp);

 	//current_direction = atof(msg->data.substr(29).c_str());
 	// up to 6 decimal precision ~10cm, error ~2m
 	ROS_INFO("Current status: ");
 	cout << "lat: " << CurrentWaypoint.lat << endl;
 	cout << "lon: " << CurrentWaypoint.lon << endl;
 	//cout << current_direction << endl;
  return;
 }
geometry_msgs::Twist createNextTwist(geometry_msgs::Twist nextTwist){

  nextTwist.linear.x = 0;// nextTwist.x; // velocity in x [-1,1] 1 is right full throttle
 	nextTwist.linear.y = 0; //nextTwist.y; // velocity in y [-1,1] 1 is forwards full-throttle
 	nextTwist.linear.z = 0; //nextTwist.z; // always 0

 	nextTwist.angular.x = 0;//nextTwist.dx; // always 0
 	nextTwist.angular.y = 0;//nextTwist.dy; // always 0
 	nextTwist.angular.z = 0;//nextTwist.dz;//
}

bool checkGoal (waypoint CurrentWaypoint, waypoint TargetWaypoint){
  return (CurrentWaypoint.lon == TargetWaypoint.lon) && (CurrentWaypoint.lat == TargetWaypoint.lat);
}

double createAngle(double angleCompass){
	/*
	Input Parameter:
	1. direction of robot from North (0-359 degrees)
	??
	2. x cordinates of TargetWaypoint in metres
	3. y cordinates of TargetWaypoint in metres
	Output: the direction the robot needs to turn 
	Purpose: calculates angle of robot to target waypoint ((-180) to 180 degrees)
	*/
	double x = 1, y = 1; //x and y cordinates in metres, this needs to be calculated or passed in a paramaters

	double theta = 0, angleWaypoint = 180; //theta: angle robot needs to turn, angleWaypoint goal angle from North
	double r = sqrt(x*x + y*y); //r = distance from the robot to waypoint
	double angleGoal = 180/PI * (acos(abs(y) / r)); //reference angle with respect to y-axis

	if (x > 0 && y >= 0) //goal angle in quad 1
		angleWaypoint = angleGoal;
	else if (x >= 0 && y < 0) //goal angle in quad 4
		angleWaypoint = (180 - angleGoal);
	//checking special condition under quad 1 and 4
	if (angleCompass > angleWaypoint + 180)
		theta = 360 - angleCompass + angleWaypoint;

	if (x < 0 && y <= 0) //goal angle in quad 3
		angleWaypoint = angleGoal + 180;
	else if (x < 0 && y > 0) //goal angle in quad 2 
		angleWaypoint = 360 - angleGoal;
	//checking special condition under quad 3 and 2 
	if (angleCompass <= angleWaypoint - 180){
		if (x < 0 && y <= 0) //checks for quad 3 
			theta = angleGoal - angleCompass - 180;
		else if (x < 0 && y > 0) //checks for quad 2
			theta = -angleCompass - angleGoal;
	}

	//for any other condtions
	if (theta == 0){
		theta = angleWaypoint - angleCompass;
	}
	return theta;
}

double createDistance (void){
	/*
	Input Parameter: void
	Output: distance in meteres from currentWaypoint to targetWaypoint
	Purpose: calculates distance from target waypoints
	Link:http://stackoverflow.com/questions/19412462/getting-distance-between-two-points-based-on-latitude-longitude-python
	*/

	//requires CurrentWaypoint.lat, CurrentWaypoint.lon, TargetWaypoint.lat, TargetWaypoint.lon
	double toRad = PI / 180;

	double lat1 = CurrentWaypoint.lat * toRad;
	double lon1 = CurrentWaypoint.lon * toRad;
	double lat2 = TargetWaypoint.lat * toRad;
	double lon2 = TargetWaypoint.lon * toRad;
	//from http://www.movable-type.co.uk/scripts/latlong.html, Distance formula (using haversine)
	double a = sin((lat2 - lat1) / 2)*sin((lat2 - lat1) / 2) + cos(lat1) * cos(lat2) * sin((lon2 - lon1) / 2)*sin((lon2 - lon1) / 2);
	double c = 2 * atan2(sqrt(a), sqrt(1 - a));
	double d = 6371000 * c;
	return d;
}
