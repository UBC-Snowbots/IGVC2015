#include "ros/ros.h"
#include "std_msgs/String.h"
#include <string>
#include <iostream>
#include "sb_ai.h"
#include <geometry_msgs/Twist.h>

#include "ControllerBase.hpp"
#include "DijkstraController.hpp"
#include "VisionController.hpp"
#include "LidarController.hpp"

ai::ControllerBase* get_controller(ros::NodeHandle& nh){
	ros::NodeHandle private_nh("~");
	std::string param;
	private_nh.getParam(MODE_PARAM,param);
	if(param=="dijkstra"){
		return new ai::DijkstraController(nh);
	}else if(param=="vision"){
		return new ai::VisionController;
	}else if(param=="lidar"){
		return new ai::LidarController(nh);
	}else if(param=="gps"){
		return new ai::sb_gps;
	}else{
		std::cout << "Invalid " << MODE_PARAM << " '" << param << "'." << std::endl;
	}
	std::cout << "Current options: " << std::endl;
  std::cout<<"????"<<std::endl;
	std::cout << "\tdijkstra" << std::endl;
	std::cout << "\tlidar" << std::endl;
	std::cout << "\tvision" << std::endl;
	std::cout << "\tgps" << std::endl;
	std::cout << "Defaulting to dijkstra" << std::endl;
	return new ai::DijkstraController(nh);
}

// main function that runs this node
int main(int argc, char **argv)
{

	// Initializes this node. 
	// First 2 arguments are for ROS arguments and name remapping.
	// Last argument is for the name of this node.
	ros::init(argc, argv, AI_NODE_NAME);
	
	// Main access point to communication with ROS system.
	ros::NodeHandle nh;
	
	// Initialize publisher to PUB_TOPIC.
	// Second argument for advertise is the buffer size.
	ros::Publisher car_pub = nh.advertise<geometry_msgs::Twist>(PUB_TOPIC, 100);	
	
	ai::ControllerBase* controller = get_controller(nh);
	if(!controller) return EXIT_FAILURE;
	
	// Frequency of the loop. 
	// eg. 10 = 10hz
	ros::Rate loop_rate(controller->get_clock_speed());

	while (ros::ok()) {

		car_pub.publish(controller->update());	// this is where we publish twist output
		
		ros::spinOnce();	// required for subscriptions
		
		loop_rate.sleep();	// sleep for the time remaining to hit frequency loop rate
	
	}

	return 0;
}
