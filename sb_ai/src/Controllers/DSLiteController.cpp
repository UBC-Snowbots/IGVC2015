#include "DSLiteController.hpp"

namespace ai
{

  DSLiteController::DSLiteController(ros::NodeHandle& nh)
  {
    // Initialize variable hacks
    gps_count = 0;
    compass_count = 0;
    global_width = (MAP_WIDTH + MAP_PADDING * 2) / RESOLUTION;
    global_height = (MAP_HEIGHT + MAP_PADDING * 2) / RESOLUTION;
    global_size = global_width * global_height;
	  origin_x = (int) global_width * START_POS_X;
	  origin_y = (int) global_height * START_POS_Y;
	  origin_lat = -1;
	  origin_long = -1;
	  x_pos = -1;
	  y_pos = -1;
  
    // Initialize world
		world = new GridWorld(global_width, INTERSECTION_RADIUS);
		
		realWorld = new int[global_size];
		for (int i = 0; i < global_size; i++)
		{
		  realWorld[i] = 0; 
		}
		
		// Initialize subscribers
		map_sub = nh.subscribe(MAP_SUB_TOPIC, 10, &DSLiteController::MapCallback, this);
		gps_sub = nh.subscribe(GPS_SUB_TOPIC, 10, &DSLiteController::GpsCallback, this);
		compass_sub = nh.subscribe(COMPASS_SUB_TOPIC, 10, &DSLiteController::CompassCallback, this);
		
		// Initialize test global map pub
		global_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("test_global_map", 10);
		global_map.data.assign(global_size, 0);
		global_map.info.origin.position.x = 0;
		global_map.info.origin.position.y = 0;
		global_map.info.origin.position.z = 0;
  }


  //Returns the next twist message to publish (this is called each main loop iteration)
  geometry_msgs::Twist DSLiteController::Update()
  {
		execute();
		std::cout << "Path calculated!" << std::endl;

		for (unsigned int y = 0; y < global_height; y++)
		{
			for (unsigned int x = 0; x < global_width; x++)
			{
				std::cout << realWorld[y*global_width + x] << " ";
			}
			std::cout << std::endl;
		}
		
		UpdateGlobalMapTest();
		global_map_pub.publish(global_map);
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
			    if (realWorld[(currentY + y)*global_width + currentX + x] == 1 && world->getTileAt(currentX + x, currentY + y)->cost < INFINITY)
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
			  realWorld[world->start->y * global_width + world->start->x] = 2;
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
	  int global_x, global_y, local_x, local_y;
    int localMapSize = map->info.width * map->info.height; 

	  // Loop through the vision map
	  for (int index = 0; index < localMapSize; index++) 
	  {
	    local_x = AI_Utilities::ConvertIndexToLocalXCoord(index, map->info.width) - ((map->info.width / 2) - 1);
	    local_y = AI_Utilities::ConvertIndexToLocalYCoord(index, map->info.width) - (map->info.height - 1);
	    
		  global_x = cos(orientation)
				  * local_x
				  - sin(orientation) * local_y
				  + AI_Utilities::GetGlobalIndexX(origin_long, long_pos, origin_x, RESOLUTION);
				  
		  global_y = sin(orientation)
				  * local_x
				  + cos(orientation) * local_y
				  + AI_Utilities::GetGlobalIndexY(origin_lat, lat_pos, origin_y, RESOLUTION);

		  // Update global map with 0/1 to show that an obstacle dne/exists
		  realWorld[AI_Utilities::ConvertXYCoordToIndex(global_x,
				  global_y, global_width)] =
				  map->data[index];
	  }
	}
	
	
	void DSLiteController::GpsCallback(const sb_msgs::Waypoint::ConstPtr& gps)
	{
	  if (!gps) return; // null check
	  
	  long_pos = gps->lon;
	  lat_pos = gps->lat;
	  
	  if (gps_count == 0)
	  {
	    origin_long = gps->lon;
	    origin_lat = gps->lat;
	    gps_count++; 
	  }
	  
	  else if (gps_count < 10) 
	  {
      origin_long += gps->lon;
      origin_long /= 2;
      origin_lat += gps->lat;
      origin_lat /= 2;
      gps_count++; 
    }
    
    else 
    {
      long_pos = gps->lon;
      lat_pos = gps->lat;
      // estimate global map position here
    }
	}
	
	
	void DSLiteController::CompassCallback(const sb_msgs::RobotState::ConstPtr& compass)
	{
	  if (!compass) return; // null check
	  
	  if (compass_count == 0)
	  {
	    global_orientation = compass->compass;
	  }
	  
	  else if (compass_count < 10)
	  {
	    global_orientation += compass->compass;
	    global_orientation /= 2;
	  }
	  
	  else 
	  {
	    orientation = compass->compass;
	    orientation -= global_orientation;
	    
	    if (orientation > 180)
	    {
	      orientation -= 360;
	    }
	    
	    if (orientation < -180)
	    {
	      orientation += 360;
	    }
	    
	    orientation *= AI_Utilities::PI / 180;
	  }
	}
	
	
	void DSLiteController::UpdateGlobalMapTest()
	{
	  for (int i = 0; i < global_size; i++)
	  {
	    global_map.data[i] = realWorld[i];
	  }
	}
	
}
