//Author: Vincent Yuan 
//Date: May 19, 2015
//Purpose: GPS Data processing 
//Input: Compass Data (sb_driver node), GPS Data (sb_gps node) 
//Output: long, lat, distance(relative to waypoint), theta (relative to waypoint)
//Things needed to be implemented: compass callback function, struct 

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <string>
#include <iostream>
#include "sb_ai.h"
#include <geometry_msgs/Twist.h>
#include <math.h>


using namespace std; 

#define EARTH_RADIUS 6378.137

struct waypoint {
  double lon;
  double lat;
  waypoint(){
    lon=lat=0;
  }
};
double *NMEA; //To hold received suscription message
double angleCompass;
bool moveStatus;
bool goal;
bool gpsFlag = true; //indicates connection from gps chip
double d = 0; //distance in metres from currentWaypoint to targetWaypoint
double theta = 0; //theta is the angle the robot needs to turn from currentWaypoint to targetWaypoint
waypoint CurrentWaypoint,TargetWaypoint,LastWaypoint;
geometry_msgs::Twist twist_msg;
int next_movement = 0;


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

geometry_msgs::Twist GetTwistMsg(int next_move) 
{
	geometry_msgs::Twist twist;

	twist.linear.x = 0;
	twist.linear.y = 0;
	twist.linear.z = 0;
		
	twist.angular.x = 0;
	twist.angular.y = 0;
	twist.angular.z = 0;

	if (next_move == -1 || next_move == 1) { twist.linear.x = next_move; }
	if (next_move == -2 || next_move == 2) { twist.linear.y = next_move; }
	cout << "Next move: " << next_move << endl;
	return twist;
}

int main (int argc, char **argv){

  ros::init(argc, argv, AI_NODE_NAME); //initialize access point to communicate
  ros::NodeHandle nh; //create handle to this process node, NodeHandle is main access point to communication with ROS system. First one intializes node
  ros::Subscriber gps_Sub = nh.subscribe("GPS_USB_DATA", 20, gpsSubHandle);
  ros::Publisher car_pub = nh.advertise<geometry_msgs::Twist>(PUB_TOPIC, 100);	

  ros::Rate loop_rate(10); //10hz loop rate


  while (ros::ok()){
    	  ROS_INFO ("Position:");
	   cout << "Current longitude: " << CurrentWaypoint.lon << " Current latitude: " << CurrentWaypoint.lat << endl;
  		   
	  twist_msg = GetTwistMsg(next_movement);
	  car_pub.publish(twist_msg);
	  ros::spinOnce(); //ros spin is to ensure that ros threading does not leave suscribes un processed
	  loop_rate.sleep();
}
    return 0;
}

