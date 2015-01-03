#include <iostream> 
#include <stlib.h> 
#include <math.h> 


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

//functions 
int gpsStatus (); //Checking if gps has started receiving data 
int nmeaParse ();  


