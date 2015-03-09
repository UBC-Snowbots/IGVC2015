/*
 * lidar_force_nav
 * Intelligent Ground Vehicle Challenge 2014
 * Oakland University - Rochester, Michigan
 * June 2014
 * 
 * UBC Snowbots -- Team Avalanche
 * University of British Columbia
 *
 */

#include <math.h>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <sb_msgs/CarCommand.h>
#include "gazebo/common/common.hh"
#include "gazebo_msgs/ModelState.h"
#include "gazebo_msgs/ApplyJointEffort.h"

using namespace ros;
using namespace std;

//Global constatns
static const double PI		 = 3.1415265;

//ros related constants
static const string NODE_NAME       = "lidar_node";
static const string PUBLISH_TOPIC   = "/gazebo/set_model_state";
static int LOOP_FREQ = 1;

geometry_msgs::Vector3 directions;
sb_msgs::CarCommand car_command;

int danger = 0;

// Clamps the value of a double, this ensures that the robot does not travel too fast
double clamp (double in, double cap)
{
	if      ( in >  cap) return cap;
	else if ( in < -1 * cap) return (-1 * cap);
	else                 return in;	
}

// Create a ModelState message for gazebo
gazebo_msgs::ModelState modelState_maker(sb_msgs::CarCommand cc)
{
	gazebo_msgs::ModelState modelState;
	
	geometry_msgs::Vector3 position;
	geometry_msgs::Vector3 orientation;
	geometry_msgs::Vector3 Linear;
	geometry_msgs::Vector3 Angular;
	
        modelState.model_name = "Snowbots";
	
	//Commented to prevent from resetting each value to 0
	 
	modelState.pose.position.x = 2;
	modelState.pose.position.y = 10;
	modelState.pose.position.z = 0;
	
	modelState.pose.orientation.x = 0;
	modelState.pose.orientation.y = 0;
	modelState.pose.orientation.z = 1.57;
	modelState.pose.orientation.w = 0;
	
	modelState.twist.linear.x = 0;
	modelState.twist.linear.y = 0;
	modelState.twist.linear.z = 0;
	
	modelState.twist.angular.x = 0;
	modelState.twist.angular.y = 0;
	modelState.twist.angular.z = 0;

	modelState.reference_frame = "world";

	return modelState;	
}


int main (int argc, char** argv)
{
	// Set up ROS node, publisher and subscriber 
	init(argc, argv, NODE_NAME);
	NodeHandle gazebo; //gazebo node handler
	Publisher gazebo_car_pub = gazebo.advertise<gazebo_msgs::ModelState>(PUBLISH_TOPIC, 1);

	gazebo_msgs::ModelState modelStateMsg = modelState_maker(car_command);
	ROS_INFO("the twist vector is %f", modelStateMsg.twist.angular.z);
	gazebo_car_pub.publish(modelStateMsg);

//	gazebo_msgs::ApplyJointEffort l; 
//	l.request.duration = ros::Duration(2.);
//	l.request.joint_name = "left_wheel_hinge";
//	l.request.effort = -1;
//	ros::ServiceClient client = gazebo.serviceClient<gazebo_msgs::ApplyJointEffort>("/gazebo/apply_joint_effort"); client.call(l);

//	gazebo_msgs::ApplyJointEffort r; 
//	r.request.duration = ros::Duration(2.);
//	r.request.joint_name = "right_wheel_hinge";
//	r.request.effort = -1;
//	ros::ServiceClient client2 = gazebo.serviceClient<gazebo_msgs::ApplyJointEffort>("/gazebo/apply_joint_effort"); client2.call(r);
	
	Rate loop_rate(LOOP_FREQ);

	ROS_INFO("ready to go");
	ROS_INFO("going");	

	while(ros::ok())
	{
		gazebo_msgs::ModelState modelStateMsg = modelState_maker(car_command);
		ROS_INFO("the twist vector is %f", modelStateMsg.twist.angular.z);
		gazebo_car_pub.publish(modelStateMsg);

		ros::spinOnce();

    		loop_rate.sleep();	
  	}
  	ROS_INFO("shutting down node");
  	return 0;
}
