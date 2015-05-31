#include "GridWorld.h"

GridWorld* world;

//The dimensions of the square map
const int SIZE = 20;

//The minimum distance between the center of the robot and any obsticles
const int INTERSECTION_RADIUS = 2;

//How far the robot can look ahead when determining its next movement
//To avoid any funny business, this value should be greater than INTERSECTION_RADIUS
const int SCAN_RADIUS = 3;

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
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0};


/*
This function handles the comparision between the robot map and the real map.
When an inconstancy is detected between the two maps, the robot map will be
updated for that area.
*/
void scanMap(){
	int currentX = world->start->x;
	int currentY = world->start->y;

	for (int y = -SCAN_RADIUS; y <= SCAN_RADIUS; y++){
		for (int x = -SCAN_RADIUS; x <= SCAN_RADIUS; x++){
			if (world->withinWorld(currentX + x, currentY + y)){
				if (realWorld[(currentY + y)*SIZE + currentX + x] == 1 
					&& world->getTileAt(currentX + x, currentY + y)->cost < INFINITY){

					printf("\tInconistancy between maps detected at %d %d\n", currentX+x, currentY+y);
					world->inflate(currentX + x, currentY + y, INFINITY);
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
	//getTileAt(3,3)->cost = INFINITY;

	world->computeShortestPath();

	while (world->start != world->goal){
		std::cout << "Iteration " << counter << std::endl;

		if (world->start->rhs == INFINITY){
			std::cout << "\tNO PATH EXIST" << std::endl;
			break;
		}

		world->start = world->getMinSuccessor(world->start).first;
		if (world->start != 0){
			std::cout << "\tMoved to: (" << world->start->x << ", " << world->start->y << ")" << std::endl;
			realWorld[world->start->y * SIZE + world->start->x] = 2;
		} else{
			std::cout << "NULL SUCCESSOR" << std::endl;
		}

		scanMap();
		counter++;
	}
	
	//Uncomment the line below to print every info about gridworld
	world->printWorld();
}


//Setting up the pathfinder etc...
void main(){
		std::cout << "Generating Map" << std::endl;
		world = new GridWorld(SIZE, INTERSECTION_RADIUS);
		std::cout << "Finished generation" << std::endl;
		execute();
		std::cout << "Path calculated!" << std::endl;

		for (unsigned int y = 0; y < SIZE; y++){
			for (unsigned int x = 0; x < SIZE; x++){
				std::cout << realWorld[y*SIZE + x] << " ";
			}
			std::cout << std::endl;
		}
		system("PAUSE");
}
