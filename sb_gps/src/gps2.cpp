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
	*current = (*waypoint) malloc ( *sizeof(waypoint)); 

	while (gpsStatus() == TRUE){
		nmeaParse();
		
	} 


int gpsStatus(void){ 
	/*
	Input Parameter: 
	Output: 
	Purpose: 
	*/
} 


void getWaypoint (*FILE, *waypoint){
	/*
	Input Parameter: 
	Output: 
	Purpose: 
	*/
} 


void createAngle (double *theta){
	/*
	Input Parameter: 
	Output: 
	Purpose: 
	*/
}


void createDistance (double *d){
	/*
	Input Parameter: 
	Output: 
	Purpose: 
	*/
}


int checkGoal (*currentWaypoint, *target){
	/*
	Input Parameter: 
	Output: 
	Purpose: 
	*/
}


void longToMetre ( *waypoint){
	/*
	Input Parameter: 
	Output: 
	Purpose: 
	*/
 
	lon = 	*waypoint->lon_x; 
	lat = *waypoint->lat_y;

}


void createTwist ( *nextTwist){
	/*
	Input Parameter: 
	Output: 
	Purpose: 
	*/


}
















