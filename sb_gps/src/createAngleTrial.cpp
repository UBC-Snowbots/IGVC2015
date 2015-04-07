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
double createAngle(double angleCompass);

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
	double angleCompass = 0;
	int count = 0;
	int trials = 0;
	printf("enter the number of trials.\n");
	scanf("%d", &trials);

	for (count = 0; count < trials; count++){
		printf("enter the angleCompass.\n");
		scanf("%lf", &angleCompass);
		theta = createAngle(angleCompass);
		printf("need to turn %lf degrees\n", theta);
		}
	}
return 0;
}

double createAngle(double angleCompass)
{
	double theta = 0, angleWaypoint = 180; //theta: angle robot needs to turn, angleWaypoint goal angle from North
	double x = 1, y = 1; //x and y cordinates in metres
	double r = sqrt(x*x + y*y); //r = distance from the robot to waypoint
	double angleGoal = 180/PI * (acos(abs(y) / r)); //reference angle with respect to y-axis

	if (x > 0 && y >= 0) //goal angle in quad 1
		angleWaypoint = angleGoal;
	else if (x >= 0 && y < 0) //goal angle in quad 4
		angleWaypoint = (180 - angleGoal);
	//checking special condition under quad 1 and 4
	if (angleCompass > angleWaypoint + 180)
		theta = 360 - angleCompass + angleWaypoint;

	if (x < 0 && y <= 0) //goal angle in quad 3
		angleWaypoint = angleGoal + 180;
	else if (x < 0 && y > 0) //goal angle in quad 2 
		angleWaypoint = 360 - angleGoal;
	//checking special condition under quad 3 and 2 
	if (angleCompass <= angleWaypoint - 180){
		if (x < 0 && y <= 0) //checks for quad 3 
			theta = angleGoal - angleCompass - 180;
		else if (x < 0 && y > 0) //checks for quad 2
			theta = -angleCompass - angleGoal;
	}

	//for any other condtions
	if (theta == 0){
		theta = angleWaypoint - angleCompass;
	}
	return theta;
}
