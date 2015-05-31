#include "DijkstraController.hpp"

namespace ai
{
  DijkstraController::DijkstraController(ros::NodeHandle& nh):
	  width(10),
	  height(10),
	  start(10),
	  goal(95)
  {
	  //map_sub = nh.subscribe(MAP_SUB_TOPIC, 10, &DijkstraController::MapCallback, this);
	  map_ptr = new int[width*height];
	  for (int i = 0; i < width*height; i++)
	  {
	    map_ptr[i] = 0;
	  }
  }

  //Returns the next twist message to publish (this is called each main loop iteration)
  geometry_msgs::Twist DijkstraController::Update()
  {
	  if (dijkstras.Init(map_ptr, width, height, start, goal)) 
	  {
		  dijkstras.Search(dijkstras.GetStart());
		  dijkstras.ReconstructPath(dijkstras.GetGoal());
		  dijkstras.SetFirstAndThird(first, fourth);
		  AI_Utilities::Location targets[2];
		  AI_Utilities::Location target_one, target_two;
		  target_one = dijkstras.ConvertToLocation(first);
		  target_two = dijkstras.ConvertToLocation(fourth);
		  std::cout << "Fourth: " << fourth << std::endl;
		  std::cout << "Goal: " << dijkstras.GetGoal() << std::endl;
		  AI_Utilities::GetVelocity(targets, 0.0f, twist_msg);
		  std::cout << "T_one: " << target_one.x << ", " << target_one.y << std::endl;
		  std::cout << "T_two: " << target_two.x << ", " << target_two.y << std::endl;
	  }
	  
	  std::cout << "Linear y = " << twist_msg.linear.y << std::endl;
	  std::cout << "Angular z = " << twist_msg.angular.z << std::endl;
	  return twist_msg;
  }
}
