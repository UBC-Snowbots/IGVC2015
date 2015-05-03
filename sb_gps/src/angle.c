#include <stdlib.h>
#include <stdio.h>
#include <math.h>

//Constants
#define PI acos(-1.0)

int main(void){
	//Variables
	double theta; //angle needed to turn robot into goal direction
	double phi; //variable needed to calculate theta
	double x; //x cordinate of the goal
	double y; //y cordinate of the goal
	double r; //distance from the robot to the goal
	double angleRobot; //angle of robot to the y-axis
	double angleCompass; //current angle of robot from North (0-359)

	r = sqrt(x*x + y*y);
	angleRobot = 180 * (acos(y / r) / PI);
	
	//while direction of goal angle is in quadrant 1
	while (x > 0 && y >= 0) {
		phi = angleRobot;
		if (angleCompass <= (phi + 180)) {
			theta = (phi - angleCompass);
		}
		else if (angleCompass > (phi + 180)) {
			theta = (360 - angleCompass + phi);
		}
	}
	//while direction of goal angle is in quadrant 4
	while (x >= 0 && y < 0) {
		phi = (180 - angleRobot);
		if (angleCompass <= (phi + 180)) {
			theta = (phi - angleCompass);
		}
		else if (angleCompass >(phi + 180)) {
			theta = (360 - angleCompass + phi);
		}
	}
	//while direction of goal angle is in quadrant 3
	while (x < 0 && y <= 0) {
		phi = (180 + angleRobot);
		if (angleCompass < (phi - 180)) {
			theta = (phi - angleCompass);
		}
		else if (angleCompass >(phi - 180)) {
			theta = (360 - angleCompass + phi);
		}
	}
	//while direction of goal angle is in quadrant 2
	while (x < 0 && y > 0) {
		phi = (360 - angleRobot);
		if (angleCompass < (phi - 180)) {
			theta = (phi - angleCompass);
		}
		else if (angleCompass >(phi - 180)) {
			theta = (360 - angleCompass + phi);
		}
	}
	return 0;
}


