//-------------------------------------------------
// UBC Snowbots
// createDistance.cpp
//   	 Purpose: Test Code
//		  
// Author: Nicholas Wu


#include "ros/ros.h"

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

//Variables
struct waypoint {
	double lon;
	double lat;
	waypoint(){
		lon = lat = 0;
	}
};
//Functions
double createDistance(void);

using namespace ros;
using namespace std;

//Constants
const double PI = acos((double)-1);

static const string NODE_NAME = "createDistanceTrial";
const int MSG_QUEUE_SIZE = 20;

int main(int argc, char **argv)
{
//Stuff to run once 
    init(argc, argv, NODE_NAME); //initializes your ROS node
	 while (ros::ok())
	{
	double d = 0;
	
	d = createDistance();
	printf("%lf\n", d);
	}
return 0;
}
double createDistance(void){
	waypoint currentWaypoint, targetWaypoint;
	double toRad = PI / 180;
	//currently these lat and lon are random just for testing
	currentWaypoint.lat = 40.7486; 
	currentWaypoint.lon = -73.9864;
	targetWaypoint.lat = 41.3829;
	targetWaypoint.lon = -70.3921;
	double lat1 = currentWaypoint.lat * toRad;
	double lon1 = currentWaypoint.lon * toRad;
	double lat2 = targetWaypoint.lat * toRad;
	double lon2 = targetWaypoint.lon * toRad;
	//from http://www.movable-type.co.uk/scripts/latlong.html, Distance formula (using haversine)
	double a = sin((lat2 - lat1) / 2)*sin((lat2 - lat1) / 2) + cos(lat1) * cos(lat2) * sin((lon2 - lon1) / 2)*sin((lon2 - lon1) / 2);
	double c = 2 * atan2(sqrt(a), sqrt(1 - a));
	double d = 6371000 * c;
	return d;
}
