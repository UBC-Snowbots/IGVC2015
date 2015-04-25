#include "DijkstraController.hpp"

namespace ai{

geometry_msgs::Twist DijkstraController::GetTwistMsg(int next_move){
	geometry_msgs::Twist twist;

	twist.linear.x = 0;
	twist.linear.y = 0;
	twist.linear.z = 0;
		
	twist.angular.x = 0;
	twist.angular.y = 0;
	twist.angular.z = 0;

	if (next_move == -1 || next_move == 1) { twist.linear.x = next_move; }
	if (next_move == -2 || next_move == 2) { twist.linear.y = next_move; }
	std::cout << "Next move: " << next_move << std::endl;
	return twist;
}

DijkstraController::DijkstraController(ros::NodeHandle& nh):
	map_ptr(NULL),
	width(0),
	height(0),
	start(90),
	goal(11),
	next_movement(0)
{
	// subscribe using the map_callback method as callback
	map_sub = nh.subscribe(MAP_SUB_TOPIC, 100, &DijkstraController::map_callback,this);
}

void DijkstraController::map_callback(const sb_msgs::AISimMap::ConstPtr& msg){
	if(map_ptr!=NULL)free(map_ptr);	// free memory allocated by map previously, if map memory is allocated
	width = msg->width;
	height = msg->height;
	map_ptr = new int[width*height];

	// copy map contents
	for (int i = 0; i < width*height; i++) {
		map_ptr[i] = msg->map_grid[i];
	}
}

/**
 *	Returns the next twist message to publish (this is called each main loop iteration)
**/
geometry_msgs::Twist DijkstraController::update(){
	if (dijkstras.Init(map_ptr, width, height, start, goal)) {
		dijkstras.Search(dijkstras.GetStart());
		dijkstras.ReconstructPath(dijkstras.GetGoal());
		next_movement = dijkstras.GetNextStep();
	}
	return GetTwistMsg(next_movement);
}

}
