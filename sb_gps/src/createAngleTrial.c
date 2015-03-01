#define _CRT_SECURE_NO_WARNINGS
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

//Constants
#define PI acos(-1)

void createAngle(double*, double);

int main(void){

	double theta = 0;
	double angleCompass = 0;
	int count = 0;
	int trials = 0;
	printf("enter the number of trials.\n");
	scanf("%d", &trials);
	
	for (theta = 0; count < trials; count++){
		printf("enter the angleCompass.\n");
		scanf("%lf", &angleCompass);
		createAngle(&theta, angleCompass);
		printf("need to turn %lf degrees\n", theta);
		system("PAUSE");
	}
	return 0;
}
void createAngle(double *theta, double angleCompass){
	/*
	Input Parameter:
	1. pointer to hold angle
	2. direction from compass
	Output: void (use pointer)
	Purpose: calculates angle from target waypoint
	*/

	// x is the x cordinate, y is the y cordinate
	double phi; //variable needed to calculate theta
	double x;
	double y;
	printf("enter x and y.\n");
	scanf("%lf %lf", &x, &y);
	double r = sqrt(x*x + y*y); //distance from the robot to waypoint
	double angleRobot = 180 * (acos(y / r) / PI); //angle of robot to the y-axis

	//while direction of goal angle is in quadrant 1
	if (x > 0 && y >= 0) {
		phi = angleRobot;
		if (angleCompass <= (phi + 180)) {
			*theta = (phi - angleCompass);
		}
		else if (angleCompass > (phi + 180)) {
			*theta = (360 - angleCompass + phi);
		}
	}
	//while direction of goal angle is in quadrant 4
	if (x >= 0 && y < 0) {
		phi = (180 - angleRobot);
		if (angleCompass <= (phi + 180)) {
			*theta = (phi - angleCompass);
		}
		else if (angleCompass >(phi + 180)) {
			*theta = (360 - angleCompass + phi);
		}
	}
	//while direction of goal angle is in quadrant 3
	if (x < 0 && y <= 0) {
		phi = (180 + angleRobot);
		if (angleCompass < (phi - 180)) {
			*theta = (phi - angleCompass);
		}
		else if (angleCompass >(phi - 180)) {
			*theta = (360 - angleCompass + phi);
		}
	}
	//while direction of goal angle is in quadrant 2
	if (x < 0 && y > 0) {
		phi = (360 - angleRobot);
		if (angleCompass < (phi - 180)) {
			*theta = (phi - angleCompass);
		}
		else if (angleCompass >(phi - 180)) {
			*theta = (360 - angleCompass + phi);
		}
	}
}

