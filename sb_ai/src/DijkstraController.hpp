#pragma once

#include "ros/ros.h"
#include <string>
#include <iostream>
#include "sb_ai.h"
#include "sb_msgs/AISimMap.h"
#include "AI/dijkstra/dijkstra.h"
#include <geometry_msgs/Twist.h>

#include "ControllerBase.hpp"

namespace ai{

class DijkstraController: public ai::ControllerBase{
private:
	int * map_ptr;
	int width; 
	int height;
	int start;
	int goal;
	geometry_msgs::Twist twist_msg;
	Dijkstra dijkstras;
	int next_movement;
	ros::Subscriber map_sub;
protected:
	geometry_msgs::Twist GetTwistMsg(int next_move);
public:
	DijkstraController(ros::NodeHandle& nh);
	
	/** callback to receive data from subscription
	 *  this is where you do what you need to do with the map data
	 **/
	void map_callback(const sb_msgs::AISimMap::ConstPtr& msg);
	geometry_msgs::Twist update();
};

}

