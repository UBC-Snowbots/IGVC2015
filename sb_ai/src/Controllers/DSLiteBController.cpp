#include "DSLiteBController.hpp"

namespace dsliteB{
GridWorld* world;

//The dimensions of the square map
const int SIZE = 20;

//The minimum distance between the center of the robot and any obsticles
const int INTERSECTION_RADIUS = 2;

//How far the robot can look ahead when determining its next movement
//To avoid any funny business, this value should be greater than INTERSECTION_RADIUS
const int SCAN_RADIUS = 4;

/*
For testing purposes only, this array represents the "real map" to pathfind on.
0 is a free space and 1 is an impassible obsticle. The robot will have another similar
map, call it "robot map", but its data is limited by how far it can see (SCAN_RADIUS).

When pathfinding, the "robot map" should produce extra high cost obsticles
around an obsticle to account for the (INTERSECTION_RADIUS)
*/
int realWorld[SIZE*SIZE] = {
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0 };


/*
This function handles the comparision between the robot map and the real map.
When an inconstancy is detected between the two maps, the robot map will be
updated for that area.
*/
void scanMap(){
	//IN VERSION B: GOAL & START ARE SWITCHED
	int currentX = world->goal->x;
	int currentY = world->goal->y;

	for (int y = -SCAN_RADIUS; y <= SCAN_RADIUS; y++){
		for (int x = -SCAN_RADIUS; x <= SCAN_RADIUS; x++){
			if (world->withinWorld(currentX + x, currentY + y)){
				if (realWorld[(currentY + y)*SIZE + currentX + x] == 1
					&& world->getTileAt(currentX + x, currentY + y)->cost < PF_INFINITY){

					printf("\tInconistancy between maps detected at %d %d\n", currentX + x, currentY + y);
					world->inflate(currentX + x, currentY + y, PF_INFINITY);
				}
			}
		}
	}


}

/*
The function handles the execution of the pathfinder, it is essentially the
main loop of our program.
*/
void execute(){
	int counter = 0;

	//Having a wall during normal search doesn't seem to cause any problems
	//getTileAt(3,3)->cost = PF_INFINITY;
	world->computeShortestPath();

	while (world->start != world->goal){
		std::cout << "Iteration " << counter;

		if (world->goal->rhs == PF_INFINITY){
			std::cout << "\tNO PATH EXIST" << std::endl;
			break;
		}

		world->goal = world->goal->successor;
		if (world->goal != 0){
			printf("\tMoved to (%u, %u)\n", world->goal->x, world->goal->y);
			realWorld[world->goal->y * SIZE + world->goal->x] = 2;
		}
		else{
			printf("\tError null successor\n");
			return;
		}

		scanMap();
		counter++;

	}

	//Uncomment the line below to print every info about gridworld
	//world->printWorld();
}




geometry_msgs::Twist DSLiteBController::GetTwistMsg(int next_move){
	geometry_msgs::Twist twist;
	twist.linear.x = 0;
	twist.linear.y = 0;
	twist.linear.z = 0;
		
	twist.angular.x = 0;
	twist.angular.y = 0;
	twist.angular.z = 0;

	if (next_move == -1 || next_move == 1) {
		twist.linear.y = 0.1; twist.angular.z = next_move * 0.3; 
	}
	if (next_move == -2 || next_move == 2) { 
		twist.linear.y = 0.3;
	}
	    
	return twist;
}

DSLiteBController::DSLiteBController(ros::NodeHandle& nh):
	map_ptr(NULL),
	width(100),
	height(100),
	start(90),
	goal(11),
	next_movement(0)
{
	std::cout << "Generating Map" << std::endl;
	world = new GridWorld(SIZE, INTERSECTION_RADIUS);
	std::cout << "Finished generation" << std::endl;
	execute();
	std::cout << "Path calculated!" << std::endl;
	
	for (unsigned int y = 0; y < SIZE; y++){
		for (unsigned int x = 0; x < SIZE; x++){
			if (world->getTileAt(x, y)->cost == INFLATION){
				std::cout << "^ ";
			}else{
				std::cout << realWorld[y*SIZE + x] << " ";

			}
		}
		std::cout << std::endl;
	}
}

//Returns the next twist message to publish (this is called each main loop iteration)
geometry_msgs::Twist DSLiteBController::Update(){

	  
	return GetTwistMsg(0);
}
}
