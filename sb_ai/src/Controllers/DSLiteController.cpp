#include "DSLiteController.hpp"

namespace ai
{
  DSLiteController::DSLiteController(ros::NodeHandle& nh):
  SIZE(20),
  INTERSECTION_RADIUS(2),
  SCAN_RADIUS(3)
  {
	  //map_sub = nh.subscribe(MAP_SUB_TOPIC, 10, &DijkstraController::MapCallback, this);
	  
	  // Initialize test map
	  realWorld = {
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
	  
	  std::cout << "Generating Map" << std::endl;
		world = new GridWorld(SIZE, INTERSECTION_RADIUS);
		std::cout << "Finished generation" << std::endl;
	  
  }

  //Returns the next twist message to publish (this is called each main loop iteration)
  geometry_msgs::Twist DSLiteController::Update()
  {
		execute();
		std::cout << "Path calculated!" << std::endl;

		for (unsigned int y = 0; y < SIZE; y++)
		{
			for (unsigned int x = 0; x < SIZE; x++)
			{
				std::cout << realWorld[y*SIZE + x] << " ";
			}
			std::cout << std::endl;
		}
  }
  
  void DSLiteController::scanMap()
  {
    int currentX = world->start->x;
    int currentY = world->start->y;

    for (int y = -SCAN_RADIUS; y <= SCAN_RADIUS; y++)
    {
	    for (int x = -SCAN_RADIUS; x <= SCAN_RADIUS; x++)
	    {
		    if (world->withinWorld(currentX + x, currentY + y))
		    {
			    if (realWorld[(currentY + y)*SIZE + currentX + x] == 1 && world->getTileAt(currentX + x, currentY + y)->cost < INFINITY)
			    {
				    printf("\tInconistancy between maps detected at %d %d\n", currentX+x, currentY+y);
				    world->inflate(currentX + x, currentY + y, INFINITY);
			    }
		    }
	    }
    }
  }

  void DSLiteController::execute()
  {
	  int counter = 0;

	  //Having a wall during normal search doesn't seem to cause any problems
	  //getTileAt(3,3)->cost = INFINITY;

	  world->computeShortestPath();

	  while (world->start != world->goal)
	  {
		  std::cout << "Iteration " << counter << std::endl;

		  if (world->start->rhs == INFINITY)
		  {
			  std::cout << "\tNO PATH EXIST" << std::endl;
			  break;
		  }

		  world->start = world->getMinSuccessor(world->start).first;
		  if (world->start != 0)
		  {
			  std::cout << "\tMoved to: (" << world->start->x << ", " << world->start->y << ")" << std::endl;
			  realWorld[world->start->y * SIZE + world->start->x] = 2;
		  } 
		  else
		  {
			  std::cout << "NULL SUCCESSOR" << std::endl;
		  }

		  scanMap();
		  counter++;
	  }
	
	  //Uncomment the line below to print every info about gridworld
	  world->printWorld();
  }
}
