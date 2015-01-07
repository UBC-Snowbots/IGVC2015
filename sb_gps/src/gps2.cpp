/* Author: Vincent Yuan
*  Date: Jan 3, 2015
*  Purpose: Main GPS function node
*  Function: Collect, parse NMEA data, create twist message, r, theta
*/ 

//Standard Headers
#include <stlib.h> 
#include <math.h> 
#include <iostream>
#include <sstream>  

//Constants
#define TRUE 1 
#define FALSE 0 

using namepsace std; 

struct twist{
int x; 
int y; 
int dx; 
int dy;
} 

struct waypoint {
int lon_x; 
int lat_y;
} 

// Variables 

float double *NMEA; //To hold received suscription message 
double botDirection; 



//functions 
int gpsStatus (); //Checking if gps has started receiving data
void nmeaParse (); //Parse intake data, from USB/buffer.txt/suscribe? Need to write driver
void getWaypoint (); //collects waypoint from txt file 
void createAngle (double *theta); //Calculates Angle  
void createDistance (double *d); //Calculates Distance
int checkGoal (); //Check to see if we are at destination 
void longToMetre (); //Calculates long/lat for distance in meters
void createTwist (); //Calculates Twist
/* The 3 create functions should be suscribed to

*/ 


int main (void){

	struct waypoint *currentWayPoint = (*waypoint) malloc ( sizeof(*waypoint)); 
	struct waypoint *currentWayPoint = (*waypoint) malloc ( sizeof (*waypoint)); 
	int *gpsFlag; //0 for no connection; 1 for satellite connection 


	while (gpsflag == TRUE){
		nmeaParse();
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
	*/
}


int checkGoal (double *currentWayPoint, double *targetWayPoint){
	/*
	Input Parameter: 
		1. pointer to waypoint struct of current position
		2. pointer to waypoint struct of target
	Output: 1 for at destination, 0 for cont. 
	Purpose: check if we are at goal 
	*/
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


void createTwist (double *nextTwist){
	/*
	Input Parameter: pointer to twist struct that holds released twist message
	Output: void (use pointer) 
	Purpose: 
		1. Alters twist message for the next output twist message
		2. Computes x,y,z? (Direction) dx,dy (velocity)
	*/

	


}
















