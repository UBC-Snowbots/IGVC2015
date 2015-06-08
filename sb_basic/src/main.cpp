#include "ros/ros.h"
#include "std_msgs/String.h"
#include <string>
#include <iostream>
#include "Controllers/sb_ai.h"
#include <geometry_msgs/Twist.h>


#include "Controllers/VisionController.hpp"
#include "Controllers/LidarController.hpp"
#include "Controllers/GpsController.hpp"

std::string param;

basic::ControllerBase* GetController(ros::NodeHandle& nh)
{
	ros::NodeHandle private_nh("~");
	
	
	if (param=="vision")
	{
	  std::cout << "Running Vision!" << std::endl;
		return new basic::VisionController(nh);
	}
	else if (param=="lidar")
	{
	  std::cout << "Running Lidar!" << std::endl;
		return new basic::LidarController(nh);
	}
	else if (param=="gps")
	{
	  std::cout << "Running GPS!" << std::endl;
	  return new basic::GpsController(nh);
	}
	else
	{
		std::cout << "Invalid " << MODE_PARAM << " '" << param << "'." << std::endl;
	}
	
	std::cout << "Current options: " << std::endl;
	std::cout << "\tlidar" << std::endl;
	std::cout << "\tvision" << std::endl;
	std::cout << "\tgps" << std::endl;
	return new basic::VisionController(nh);
}

// main function that runs this node
int main(int argc, char **argv)
{
	ros::init(argc, argv, AI_NODE_NAME);
	ros::NodeHandle nh;
	ros::Publisher car_pub = nh.advertise<geometry_msgs::Twist>(PUB_TOPIC, 10);	
	
	int count = 0;
	int wp1 = 0; // turns to 1 if hit
	int wp2 = 0; // turns to 1 if hit
	double lgp; // lidar get priority
	double vgp; // vision get priority


	param = "vision";
	basic::ControllerBase* controller_vision = GetController(nh);
	param = "gps";
	basic::ControllerBase* controller_gps = GetController(nh);
	param = "lidar";
	basic::ControllerBase* controller_lidar = GetController(nh);

	if (!controller_vision) return EXIT_FAILURE;
	if (!controller_gps) return EXIT_FAILURE;
	if(!controller_lidar) return EXIT_FAILURE;

	ros::Rate loop_rate(controller_vision->GetClockSpeed());

	while (ros::ok()) 
	{
		//Qualification Round
		if (count == 0)
		{
		car_pub.publish(controller_vision->Update());
		std::cout<<"status:" <<(controller_vision->getVisionStatus());	
		if ((controller_vision->getVisionStatus()) == 0) 
		{
			count = 1;
            std::cout<<"switching to gps and lidar"<<std::endl;
        }
        }
        if(count == 1)
       {
        	std::cout<<"gps_mode"<<std::endl;
			car_pub.publish(controller_gps->Update());
	    }
  /*     
	    //Basic Course
        //Check vision & lidar priorities as well as whether gps waypoint reached
       vgp = (controller_vision->getPriority());
       lgp = (controller_lidar->getPriority());
       wp1 =(controller_gps->Waypoint1());
       wp2 = (controller_gps->Waypoint2());
       
       // Go along with vision and lidar stop vision only if lidar in red
       if(count == 1)
       {
       	if(lgp > vgp) car_pub.publish(controller_lidar->Update());
       	else car_pub.publish(controller_vision->Update());
       	if (wp1 == 1) count = 2;
       }
       if(count == 2)
       {
       	if(lgp > 0.5) car_pub.publish(controller_lidar->Update());
        if(vgp > 0.5) car_pub.publish(controller_vision->Update());
       	else car_pub.publish(controller_gps->Update());
       	car_pub.publish(controller_gps->Update());
        if (wp2 == 1) count = 3;
       }
       if(count == 3)
       {
        if(lgp > vgp) car_pub.publish(controller_lidar->Update());
       	else car_pub.publish(controller_vision->Update());
       }
*/
       //TODO: Would be nice to have a linear combination of priorities

       



		ros::spinOnce();	
		loop_rate.sleep();	
	}

	return 0;
}
