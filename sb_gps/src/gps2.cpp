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
#include "ros/ros.h" //for ros system
#include "std_msgs/String.h"
#include <string.h>

//Constants
#define EARTH_RADIUS 6378.137 //In KM
#define WAYPOINT_FILE "practice1.txt" //using practice1.txt for testing

using namespace std;

struct twist{
double x;
double y;
double z;
double dx;
double dy;
double dz;
};

struct waypoint {
double lon_x;
double lat_y;
};

// Variables

double *NMEA; //To hold received suscription message
double botDirection;
bool goal;

static const string GPS_NODE_NAME = "gps_node";
static const string GPS_OUTPUT_TOPIC = "gps_nav";
static const string GPS_TEST_TOPIC = "vision_vel"; // test sub
static const string GPS_INPUT_TOPIC = "gps_state"; // gps_state
static const string NODE_NAME = "sb_gps";
static const string PUBLISH_TOPIC = "gps_twist";
static const string GPS_INPUT_DIRECTION = "waypoint"
static int LOOP_FREQ = 30;

//functions
int gpsStatus (); //Checking if gps has started receiving data (((STATUS = 0)))
void nmeaParse (); //Parse intake data, from USB/buffer.txt/suscribe? Need to write driver (((STATUS = 0)))
void getWaypoint (); //collects waypoint from txt file (((STATUS = 0)))
void createAngle (double *theta, double angleCompass); //Calculates Angle  (((STATUS = 0)))
void createDistance (double *d); //Calculates Distance (((STATUS = 0)))
int checkGoal (); //Check to see if we are at destination (((STATUS = 1)))
void createTwist (); //Calculates Twist (((STATUS = 0)))
/* The 3 create functions should be suscribed to

*/

//geometry_msgs::Vector3 directions
int main (int argc, char **argv){

  waypoint currentWayPoint,targetWayPoint;
	twist nextTwist;

	if (currentWayPoint == NULL || targetWaypoint == NULL || nextTwist == NULL){
		//cout << "Error, not enough memory for program" << endl;
		ROS_INFO("Error, not enough memory for program");
		//cout << "Memory Error in main function, struct initiation" << endl;
		ROS_INFO("Memory Error in Main, Struct Fail");
		} //Memory allocation error message
	else {
		targetWaypoint = {0.0}; //intialize to 0
		currentWayPoint = {0.0}; //intialize to 0
		nextTwist = {0.0}
		}

	int *gpsFlag; //0 for no connection; 1 for satellite connection

	ros::init(argc, argv, GPS_NODE_NAME); //initialize access point to communicate
	ros::NodeHandle nh; //create handle to this process node, NodeHandle is main access point to communication with ROS system. First one intializes node
	ros::Publisher gps_test_pub = nh.advertise<gemoertry_msgs::Twist>(GPS_TEST_TOPIC, 20);
	ros::Publisher gps_data_pub = nh.advertise<sb_msgs::CarCommand>(GPS_OUTPUT_TOPIC, 20);
	ros::Suscriber gps_Sub = nh.suscribe(GPS_INPUT_TOPIC, 20, gpsSubHandle);
	ros::ServiceClient client = n.serviceClient<>//finish from sb_WaypointManager
	ros::Rate loop_Rate(5); //10hz loop rate

	geometry_msgs::Twist nextTwist;

	while (ros::ok()){
		while (gpsflag == true){
			nmeaParse();
			//cout << "Everything is going to be ok" << endl;
			ROS_INFO("Everything is going to be ok");

			if(checkGoal (currentWayPoint, targetWayPoint)){
				createTwist (nextTwist); //Make new twist message

				ROS_INFO("Twist lin.y and ang.z is %f , %f", twistMsg.linear.y, twistMsg.angular.z);
				publisher_name.publish(twist); //TODO: change name of publisher_name
				ros::spinOnce();
			  loop_rate.sleep();

				}
			else{
				twist.linear.x = 0;
				twist.linear.y = 0;
				twist.linear.z = 0;

				twist.angular.x = 0;
				twist.angular.y = 0;
				twist.angular.z = 0;

				ROS_INFO("Arrived at destination");
				publisher_name.publish(twist); //TODO: change name of publisher_name
				ros::spinOnce(); //ros spin is to ensure that ros threading does not leave suscribes un processed
			  loop_rate.sleep();
		}


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
	last_waypoint.long_x = current_waypoint.long_x;
	current_waypoint.long_x = atoi(temp);

	//processing y coord
	for (i=15, j=0; i<10 || j<8; i++, j++) {
		if (a[i] == '.') { j--; }
		else { temp[j] = a[i]; }
	}
	last_waypoint.lat_y = current_waypoint.lat_y;
	current_waypoint.lat_y = atoi(temp);

	//current_direction = atof(msg->data.substr(29).c_str());
	// up to 6 decimal precision ~10cm, error ~2m
	ROS_INFO("Current status: ")
	cout << "lat: " + current_waypoint.lat_y << endl;
	cout << "lon: " + current_waypoint.long_x << endl;
	//cout << current_direction << endl;
}

void createAngle(double *theta, double angleCompass, double x, double y){
	/*
	Input Parameter:
	1. pointer to hold angle
	2. direction of robot from North (0-359 degrees)
	3. x cordinate of target waypoint
	4. y codinate of target waypoint
	Output: void (use pointer)
	Purpose: calculates angle of robot to target waypoint ((-180)-180 degrees)
	*/

	// x is the x cordinate, y is the y cordinate
	double angleWaypoint; //Angle from North to waypoint
	double r = sqrt(x*x + y*y); //distance from the robot to waypoint
	double angleGoal = 180 * (acos(abs(y) / r) / PI); //angle to target waypoint from the y-axis
	double angleCompass180; //angleCompass +/- 180

	if (angleCompass >= 180){
		angleCompass180 = angleCompass - 180;
	}
	else if (angleCompass < 180){
		angleCompass180 = angleCompass + 180;
	}
	//while direction of goal angle is in quadrant 1
	if (x > 0 && y >= 0) {
		angleWaypoint = angleGoal;
		if (angleCompass <= angleCompass180) {
			if (angleWaypoint > angleCompass180){
				*theta = (angleWaypoint - angleCompass - 360);
			}
			else{
				*theta = angleWaypoint - angleCompass;
			}
		}
		else if (angleCompass > angleCompass180) {
			if (angleWaypoint < angleCompass180){
				*theta = (angleCompass - angleWaypoint - 360);
			}
			else{
				*theta = angleWaypoint - angleCompass;
			}
			if (angleWaypoint < angleCompass - 180){
				*theta = -*theta;
			}
		}
	}
	//while direction of goal angle is in quadrant 4
	if (x >= 0 && y < 0) {
		angleWaypoint = (180 - angleGoal);
		if (angleCompass <= angleCompass180) {
			if (angleWaypoint > angleCompass180){
				*theta = (angleWaypoint - angleCompass - 360);
			}
			else{
				*theta = angleWaypoint - angleCompass;
			}
		}
		else if (angleCompass > angleCompass180) {
			if (angleWaypoint < angleCompass180){
				*theta = (angleCompass - angleWaypoint - 360);
			}
			else{
				*theta = angleWaypoint - angleCompass;
			}
			if (angleWaypoint < angleCompass - 180){
				*theta = -*theta;
			}
		}
	}
	//while direction of goal angle is in quadrant 3
	if (x < 0 && y <= 0) {
		angleWaypoint = (180 + angleGoal);
		if (angleCompass < angleCompass180) {
			if (angleWaypoint > angleCompass180){
				*theta = (angleWaypoint - angleCompass - 360);
			}
			else{
				*theta = angleWaypoint - angleCompass;
			}
		}
		else if (angleCompass > angleCompass180) {
			if (angleWaypoint < angleCompass180){
				*theta = (angleCompass - angleWaypoint - 360);
			}
			else{
				*theta = angleWaypoint - angleCompass;
			}
		}
	}
	//while direction of goal angle is in quadrant 2
	if (x < 0 && y > 0) {
		angleWaypoint = (360 - angleGoal);
		if (angleCompass < angleCompass180) {
			if (angleWaypoint > angleCompass180){
				*theta = (angleWaypoint - angleCompass - 360);
			}
			else{
				*theta = angleWaypoint - angleCompass;
			}
		}
		else if (angleCompass > angleCompass180) {
			if (angleWaypoint < angleCompass180){
				*theta = (angleCompass - angleWaypoint - 360);
			}
			else{
				*theta = angleWaypoint - angleCompass;
			}
		}
	}
}



double getCompass (const std_msgs::String:ConstPtr& angle){
	double compass = 0;
	angle = convertAngle(compass)
	return angle;
}

double convertAngle (double realAngle){
	/*
	Purpose: Converts angle 0>angle>359 to -180>angle>180
	*/
	if (realAngle > 180.0)
		return (-1)*(realAngle - 180.0);
	else if (realAngle < 180)
		return realAngle;
}


void createDistance (double *d){
	/*
	Input Parameter: pointer to store distance
	Output: void (use pointer)
	Purpose: calculates distance from target waypoints
	Link:http://stackoverflow.com/questions/19412462/getting-distance-between-two-points-based-on-latitude-longitude-python
	*/
	lon = 	(*currentWayPoint).lon_x;
	lat = (*currentWayPoint).lat_y;

	double dlong,dlat;

	dlong = (*targetWayPoint).lon_x - (*currentWayPoint).lon_x;
	dlat = (*targetWayPoint).lat_y - (*currentWayPoint).lat_y;

	double a = (sin(dlat/2))**2 + cos((*currentWayPoint).lat_y) * 		cos((*targetWayPoint).lat_y) * (sin(dlong/2))**2;
	double c = 2 * atan2(sqrt(a), sqrt(1-a));

	*d = R * c;
}
	

int checkGoal (double *currentWayPoint, double *targetWayPoint){
	/*
	Input Parameter:
		1. pointer to waypoint struct of current position
		2. pointer to waypoint struct of target
	Output: 1 for at destination, 0 for cont.
	Purpose: check if we are at goal
	Status: Finished
	*/
	if (currentWayPoint.long_x == targetWayPoint.long_x){
		if (currentWayPoint.long_y == targeWayPoint.long_y){
			return 1;
		}
		else
			return 0;
	}
	else return 0;
}




void createTwist (double *nextTwist, double *theta, double *d){
	/*
	Input Parameter: pointer to twist struct that holds released twist message
	Output: void (use pointer)
	Purpose:
		1. Alters twist message for the next output twist message
		2. Computes x,y,z? (Direction) dx,dy (velocity)
	*/

	nextTwist.x =
	nextTwist.y =
	nextTwist.z =
	nextTwist.dx =
	nextTwist.dy =
	nextTwist.dz =

	// Set-Up Your Velocities (these will be between 0 and 1, 0 min and 1 max)
	twist.linear.x = nextTwist.x; // velocity in x [-1,1] 1 is right full throttle
	twist.linear.y = nextTwist.y; // velocity in y [-1,1] 1 is forwards full-throttle
	twist.linear.z = nextTwist.z; // always 0

	twist.angular.x = nextTwist.dx; // always 0
	twist.angular.y = nextTwist.dy; // always 0
	twist.angular.z = nextTwist.dz;// [-1,1] -1 is turning right, -1 is turning left full throttle


}


/*void getWaypoint (){

	Input Parameter:
		1. Pointer to txt file
		2. Pointer to targete
	Output: void (use pointers)
	Purpose: Get target waypoint from txt file and store it

	string output;
	int count = 0;
	int array_size;
	int* waypoints_array = NULL;
	ifstream waypoints_file ("../WaypointsTxt/practice1.txt");
	if (waypoints_file.is_open())
	{
		while (getline(waypoints_file, output))
		{
			if (count == 0) {
				array_size = atoi(output.c_str())*2;
				waypoints_array = new int [array_size];
				count++;
			}

			else {
				waypoints_array[count-1] = atoi(output.c_str());
				count++;
			}
		}
		waypoints_file.close();
	}

	else
	{
		ROS_INFO("Unable to open file");
	}
}*/
