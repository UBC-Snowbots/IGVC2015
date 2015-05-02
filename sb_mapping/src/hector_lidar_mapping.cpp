/*
 * hector_lidar_mapping
 * Intelligent Ground Vehicle Challenge 2015
 * Oakland University - Rochester, Michigan
 * June 2015
 * 
 * UBC Snowbots -- Team Avalanche
 * University of British Columbia
 *
 */

#include <math.h>
#include <iostream>
#include <ros/ros.h>
#include "nav_msgs/OccupancyGrid.h"
#include "std_msgs/Header.h"
#include "nav_msgs/MapMetaData.h"
#include "geometry_msgs/PoseStamped.h"

using namespace ros;
using namespace std;

//Global constatns
static const double PI		 = 3.1415265;

//ros related constants
static const string NODE_NAME       = "hector_lidar_mapping";
static const string SUBSCRIBE_TOPIC = "slam_out_pose";
static int LOOP_FREQ = 30;

void callback(const geometry_msgs::PoseStamped::ConstPtr msg_ptr)//const nav_msgs::OccupancyGrid::ConstPtr& msg_ptr)
{
	//std_msgs::Header header = msg_ptr->header;
	//nav_msgs::MapMetaData info = msg_ptr->info;

	geometry_msgs::Pose pose = msg_ptr->pose;
	ROS_INFO("%lf %lf -- %lf %lf", pose.position.x, pose.position.y, pose.orientation.z, pose.orientation.w);
}

int main (int argc, char** argv)
{
	// Set up ROS node, publisher and subscriber 
	init(argc, argv, NODE_NAME);
	NodeHandle n;
	Subscriber lidar_state = n.subscribe(SUBSCRIBE_TOPIC, 20, callback);
	Rate loop_rate(LOOP_FREQ);

	ROS_INFO("ready to go");
	ROS_INFO("going");
	
	ros::spin();

  	ROS_INFO("shutting down node");
  	return 0;
}
