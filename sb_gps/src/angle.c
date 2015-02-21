#include <stdlib.h>
#include <stdio.h>
#include <math.h>

//Functions
void getTheta1 (double compass, double phi);
void getTheta2 (double compass, double phi);

int main(void){

	double phi = 0;
	double theta = 0;
	double x = 1;
	double y = 1;
	double r = 5; 
	double s = acos( y/r );

	while ( x > 0, y >= 0 ) {
		phi = s;
		if ( compass <= (phi + 180) ) {
		getTheta1 (compass, phi);
		}
		else if ( compass > (phi + 180) ) {
		getTheta2 (compass, phi);
		}	
	}
	while ( x >= 0, y > 0 ) {
		phi = (180 - s);
		if ( compass <= (phi + 180) ) {
		getTheta1 (compass, phi);
		}
		else if ( compass > (phi + 180) ) {
		getTheta2 (compass, phi);
		}
	}
	while ( x < 0, y <= 0 ) {
		phi = (180 + s);
		if ( compass < (phi - 180) ) {
		getTheta1 (compass, phi);
		}
		else if ( compass > (phi - 180) ) {
		getTheta2 (compass, phi);
		}
	}
	while ( x < 0, y > 0 ) {
		phi = (360 - s);
		if ( compass < (phi - 180) ) {
		getTheta1 (compass, phi);
		}
		else if ( compass > (phi - 180) ) {
		getTheta2 (compass, phi);
		}
	}
	return 0;
}
void getTheta1 (double compass, double phi) {
	theta = (phi - compass);
}
void getTheta2 (double compass, double phi) {
		theta = (360 - compass + phi);
}
