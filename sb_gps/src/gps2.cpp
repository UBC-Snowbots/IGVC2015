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
	struct waypoint current = 0; 

	while (gpsStatus()){
		nmeaParse();
	
	} 















