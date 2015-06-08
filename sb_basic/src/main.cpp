#include "ros/ros.h"
#include "std_msgs/String.h"
#include <string>
#include <iostream>
#include "Controllers/sb_ai.h"
#include <geometry_msgs/Twist.h>


#include "Controllers/VisionController.hpp"
#include "Controllers/LidarController.hpp"
#include "Controllers/GpsController.hpp"


ai::ControllerBase* GetController(ros::NodeHandle& nh)
{
	ros::NodeHandle private_nh("~");
	std::string param;
	private_nh.getParam(MODE_PARAM, param);
	
	if (param=="vision")
	{
	  std::cout << "Running Vision!" << std::endl;
		return new ai::VisionController(nh);
	}
	else if (param=="lidar")
	{
	  std::cout << "Running Lidar!" << std::endl;
		return new ai::LidarController(nh);
	}
	else if (param=="gps")
	{
	  std::cout << "Running GPS!" << std::endl;
	  return new ai::GpsController(nh);
	}
	else
	{
		std::cout << "Invalid " << MODE_PARAM << " '" << param << "'." << std::endl;
	}
	
	std::cout << "Current options: " << std::endl;
	std::cout << "\tlidar" << std::endl;
	std::cout << "\tvision" << std::endl;
	std::cout << "\tgps" << std::endl;
	return new ai::VisionController(nh);
}

// main function that runs this node
int main(int argc, char **argv)
{
	ros::init(argc, argv, AI_NODE_NAME);
	ros::NodeHandle nh;
	ros::Publisher car_pub = nh.advertise<geometry_msgs::Twist>(PUB_TOPIC, 10);	
	
	ai::ControllerBase* controller = GetController(nh);
	if (!controller) return EXIT_FAILURE;

	ros::Rate loop_rate(controller->GetClockSpeed());

	while (ros::ok()) 
	{
		car_pub.publish(controller->Update());	
		//if (controller->getVisionStatus == 0)
		//param == "gps"
		// ai::ControllerBase* controller = GetController(nh);
		ros::spinOnce();	
		loop_rate.sleep();	
	}

	return 0;
}
