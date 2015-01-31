/* Author: Vincent Yuan
*  Date: Jan 3, 2015
*  Purpose: Main GPS function node
*  Function: Collect, parse NMEA data, create twist message, r, theta
*/ 

//Standard Headers
#include <stlib.h> 
#include <math.h> 
#include <iostream> //for cout, rather that using stdio
#include <fstream> //read from file
#include <sstream>  
#include "ros/ros.h" //for ros system
#include "std_msgs/string.h" 

//Constants
#define TRUE 1 
#define FALSE 0 
#define EARTH_RADIUS 6378.137 //In KM 

using namepsace std; 

struct twist{
double x; 
double y; 
double z;
double dx; 
double dy;
double dz;
} 

struct waypoint {
double lon_x; 
double lat_y;
} 

// Variables 

float double *NMEA; //To hold received suscription message 
double botDirection; 

static const string GPS_NODE_NAME = "gps_node"; 
static const string GPS_OUTPUT_TOPIC = "gps_nav"; 
static const string GPS_TEST_TOPIC = "vision_vel"; // test sub
static const string GPS_INPUT_TOPIC = "gps_state"; // gps_state
static const string NODE_NAME = "sb_gps";
static const string PUBLISH_TOPIC = "gps_twist"; 
static int LOOP_FREQ = 30; 

//functions 
int gpsStatus (); //Checking if gps has started receiving data (((STATUS = 0)))
void nmeaParse (); //Parse intake data, from USB/buffer.txt/suscribe? Need to write driver (((STATUS = 0)))
void getWaypoint (); //collects waypoint from txt file (((STATUS = 0)))
void createAngle (double *theta); //Calculates Angle  (((STATUS = 0)))
void createDistance (double *d); //Calculates Distance (((STATUS = 0)))
int checkGoal (); //Check to see if we are at destination (((STATUS = 1))) 
void longToMetre (); //Calculates long/lat for distance in meters (((STATUS = 0)))
void createTwist (); //Calculates Twist (((STATUS = 0)))
/* The 3 create functions should be suscribed to

*/ 

geometry_msgs::Vector3 directions
int main (int argc, char **argv){

	struct waypoint currentWayPoint = (struct waypoint*) malloc ( sizeof(struct waypoint)); //Create struct for current way point
	struct waypoint targetWayPoint = (struct waypoint*) malloc ( sizeof (struct waypoint)); //Create struct for target way point 
	struct twist nextTwist = (struct twist*) malloc (sizeof(struct twist)); //create struct for the next output twist message
	if (currentWayPoint == NULL || targetWaypoint == NULL || nextTwist == NULL){
		cout << "Error, not enough memory for program" << endl; 
		cout << "Memory Error in main function, struct initiation" << endl; 
		} //Memory allocation error message 
	else {
		targetWaypoint = {0.0,0.0}; //intialize to 0
		currentWayPoint = {0.0,0.0}; //intialize to 0 
		nextTwist = {0.0,
		}

	int *gpsFlag; //0 for no connection; 1 for satellite connection 
	
	ros::init(argc, argv, GPS_NODE_NAME); //initialize access point to communicate 
	ros::NodeHandle nh; //create handle to this process node, NodeHandle is main access point to communication with ROS system. First one intializes node
	ros::Publisher gps_test_pub = nh.advertise<gemoertry_msgs::Twist>(GPS_TEST_TOPIC, 20); 
	ros::Publisher gps_data_pub = nh.advertise<sb_msgs::CarCommand>(GPS_OUTPUT_TOPIC, 20); 
	ros::Suscriber gps_Sub = nh.suscribe(GPS_INPUT_TOPIC, 20, gpsCallback); 
	
	ros::Rate loop_Rate(5); //10hz loop rate
	
	geometry_msgs::Twist nextTwist; 

	while (ros::ok()){
		while (gpsflag == TRUE){
		nmeaParse();
		cout << "Everything is going to be ok" << endl;
				
		createTwist (nextTwist); //Make new twist message
		checkGoal (

		// Set-Up Your Velocities (these will be between 0 and 1, 0 min and 1 max)
		twist.linear.x = twist.x; // velocity in x [-1,1] 1 is right full throttle
		twist.linear.y = twist.y; // velocity in y [-1,1] 1 is forwards full-throttle
		twist.linear.z = twist.z; // always 0

		twist.angular.x = twist.dx; // always 0
		twist.angular.y = twist.dy; // always 0
		twist.angular.z = twist.dz;// [-1,1] -1 is turning right, -1 is turning left full throttle

	
		ROS_INFO("Twist lin.y and ang.z is %f , %f", twistMsg.linear.y, twistMsg.angular.z);
		publisher_name.publish(twist); //TODO: change name of publisher_name
		ros::spinOnce();
	   	loop_rate.sleep();

		
		} 



	
	
}

void getWaypoint (*FILE, *targetWayPoint){
	/*
	Input Parameter: 
		1. Pointer to txt file 
		2. Pointer to targete 
	Output: void (use pointers) 
	Purpose: Get target waypoint from txt file and store it 
	*/	
} 


void createAngle (double *theta, double compass){
	/*
	Input Parameter: 
		1. pointer to hold angle
		2. direction from compass 
	Output: void (use pointer) 
	Purpose: calculates angle from target waypoint 
	*/
}


void createDistance (double *d){
	/*
	Input Parameter: pointer to store distance
	Output: void (use pointer) 
	Purpose: calculates distance from target waypoints
	Link:http://stackoverflow.com/questions/19412462/getting-distance-between-two-points-based-on-latitude-longitude-python
	*/

	double dlong,dlat
       *d = 
	return 0;
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
	if (currentWayPoint.x == targetWayPoint.x){
		if (currentWayPoint.y == targeWayPoint.y){
			if (currentWayPoint.z == targeWayPoint.z){
				return 1; 
			else 
				return 0; 
			}
		}
		else 
			return 0; 
	}
	else return 0; 
}


void longToMetre ( double *currentWayPoint){
	/*
	Input Parameter: pointer to waypoint struct of current position
	Output: void (use pointer) 
	Purpose: 
		1. Updates struct holding current waypoint 
		2. Calculates waypoint long and lat from parsed NMEA
	*/
 
	lon = 	(*currentWayPoint).lon_x; 
	lat = (*currentWayPoint).lat_y;

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
	


}
















