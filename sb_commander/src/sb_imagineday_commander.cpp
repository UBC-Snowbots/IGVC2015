/*
 * Commander Node
 * Intelligent Ground Vehicle Challenge 2014
 * Oakland University - Rochester, Michigan
 * June 2014
 * 
 * UBC Snowbots -- Team Avalanche
 * University of British Columbia
 *
 */

#include <iostream>
#include <stdlib.h>
#include <string>
#include <math.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/LaserScan.h>

#include "sb_msgs/CarCommand.h"
#include "sb_msgs/LidarNav.h"

// Global Constants
static const int SECOND = 1000000; //1 million us
static const int DEFAULT_SPEED = 0.3;

// steering = 1 is left in stage so steering = -1 is right
geometry_msgs::Twist twistConvertor(sb_msgs::CarCommand car_msg);

using namespace ros;
using namespace std;

struct NavCommand
{
    NavCommand() : throttle(0), steering(0), priority(-2) {}
    double throttle;
    double steering;
    double priority;
};

// ROS-related Constants
static const string NODE_NAME               = "imagine_commander";
static const string CAR_PUBLISH_TOPIC       = "vision_vel";
static const string LIDAR_SUBSCRIBE_TOPIC   = "lidar_nav";  // lidar_nav node suggestion
static const int LOOP_FREQ = 30; // Hz

NavCommand lidar_command;

bool stopSignFlag = false;
bool redLightFlag = false;


//callback for lidar directions
void lidar_state_callback(const sb_msgs::CarCommand::ConstPtr& msg)
{
  ROS_INFO("RUNNING LIDAR CALLBACK!");
  cout << "test" << endl;
  lidar_command.throttle = msg->throttle;
  lidar_command.steering = msg->steering;
  lidar_command.priority = msg->priority;
}


geometry_msgs::Twist driver()
{
    geometry_msgs::Twist car_msg;
    car_msg.linear.x = 0;
    car_msg.linear.y = 0;
    car_msg.linear.z = 0;
    car_msg.angular.x = 0;
    car_msg.angular.y = 0;
    car_msg.angular.z = 0;

    car_msg.linear.y  = lidar_command.throttle;	//Throtle;
    car_msg.angular.z = lidar_command.steering;	//Steering;
    ROS_INFO("Running on lidar_nav - throttle: %0.2f, steering: %0.2f, priority: %0.2f", car_msg.linear.y, car_msg.angular.z, lidar_command.priority);		
		
    return car_msg;
}


int main( int argc, char** argv )
{
 
    //ros initialization
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle n;

    ros::Subscriber LIDAR_State = n.subscribe(LIDAR_SUBSCRIBE_TOPIC, 20, lidar_state_callback);

    ros::Publisher car_pub = n.advertise<geometry_msgs::Twist>(CAR_PUBLISH_TOPIC, 20);

    //controls how fast it goes
    ros::Rate loop_rate(LOOP_FREQ);

    ROS_INFO("ready to go");
    
    //give it 3 seconds before it starts moving!
    usleep(3*SECOND);
       
    ROS_INFO("going");   
    
    while(ros::ok())
    {
	geometry_msgs::Twist car_msg = driver();
        car_pub.publish(car_msg);
        
        ros::spinOnce();
        loop_rate.sleep();	
    }
    ROS_INFO("shutting down node");
  
    return 0;
}


