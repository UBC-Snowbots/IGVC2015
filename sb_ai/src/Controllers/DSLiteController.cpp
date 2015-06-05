#include "DSLiteController.hpp"

namespace ai
{

  DSLiteController::DSLiteController(ros::NodeHandle& nh)
  {
    // Initialize world
		world = new GridWorld(SIZE, INTERSECTION_RADIUS);
		
		realWorld = new int[SIZE*SIZE];
		for (int i = 0; i < SIZE*SIZE; i++)
		{
		  realWorld[i] = 0; 
		}
		
		// Initialize subscribers
		map_sub = nh.subscribe(MAP_SUB_TOPIC, 10, &DSLiteController::MapCallback, this);
		gps_sub = nh.subscribe(GPS_SUB_TOPIC, 10, &DSLiteController::GpsCallback, this);
		compass_sub = nh.subscribe(COMPASS_SUB_TOPIC, 10, &DSLiteController::CompassCallback, this);
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
	  //getTileAt(3,3)->cost = PF_INFINITY;

	  world->computeShortestPath();

	  while (world->start != world->goal){
		  std::cout << "Iteration " << counter;

		  if (world->start->rhs == PF_INFINITY){
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
  
  
	void DSLiteController::MapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map)
	{
	  // Add local to global functions
	}
	
	
	void DSLiteController::GpsCallback(const std_msgs::String::ConstPtr& gps)
	{
	  // process gps data, dummy data for now
	  long_pos = 51.28374830;
	  lat_pos = 16.939458393;
	}
	
	
	void DSLiteController::CompassCallback(const std_msgs::String::ConstPtr& compass)
	{
	  // process compass data
	  orientation = 30.0;
	}
}
