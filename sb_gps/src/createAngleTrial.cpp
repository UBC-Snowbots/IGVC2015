//-------------------------------------------------
// UBC Snowbots
// createAngleTrial.cpp
//   	 Purpose: Testing code to determine the angle of
//		  the Robot to turn.
// Author: Nicholas Wu


#include "ros/ros.h"

#define _CRT_SECURE_NO_WARNINGS
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

//Constants
#define PI acos(-1)
//Functions
void createAngle(double *theta, double angleCompass);

using namespace ros;
using namespace std;

static const string NODE_NAME = "createAngleTrial";
const int MSG_QUEUE_SIZE = 20;

int main(int argc, char **argv)
{
//Stuff to run once 
    init(argc, argv, NODE_NAME); //initializes your ROS node
	 while (ros::ok())
	{
	double theta = 0; //angle the robot needs to turn
	double angleCompass = 0; //angle of robot from North
	int count = 0;
	int trials = 0;
	printf("enter the number of trials.\n");
	scanf("%d", &trials);

	for (theta = 0; count < trials; count++){
		printf("enter the angleCompass.\n");
		scanf("%lf", &angleCompass);
		createAngle(&theta, angleCompass);
		printf("need to turn %lf degrees\n", theta);
		}
	}
return 0;
}
void createAngle(double *theta, double angleCompass){
	/*
	Input Parameter:
	1. pointer to hold angle
	2. direction of robot from North (0-359 degrees)
	Output: void (use pointer)
	Purpose: calculates angle of robot to target waypoint ((-180)-180 degrees)
	*/

	// x is the x cordinate, y is the y cordinate
	double angleWaypoint; //Angle from North to waypoint
	double x;
	double y;
	printf("enter x and y.\n");
	scanf("%lf %lf", &x, &y);
	double r = sqrt(x*x + y*y); //distance from the robot to waypoint
	double angleGoal = 180 * (acos(abs(y) / r) / PI); //angle of goal to the y-axis
	double angleCompass180; //Angle +/- 180

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
			if (angleWaypoint < angleCompass180){
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
			if (angleWaypoint < angleCompass180){
				*theta = -*theta;
			}
		}
	}
	//while direction of goal angle is in quadrant 3
	if (x < 0 && y <= 0) {
		angleWaypoint = (180 + angleGoal);
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
			if (angleWaypoint > angleCompass180){
				*theta = (angleWaypoint - angleCompass - 360);
			}
		}
	}
	//while direction of goal angle is in quadrant 2
	if (x < 0 && y > 0) {
		angleWaypoint = (360 - angleGoal);
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
			if (angleWaypoint > angleCompass180){
				*theta = (angleWaypoint - angleCompass - 360);
			}			
		}
	}
}

