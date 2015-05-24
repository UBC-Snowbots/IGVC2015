#include "DijkstraController.hpp"

namespace ai
{
  geometry_msgs::Twist DijkstraController::GetTwistMsg(int next_move)
  {
	  geometry_msgs::Twist twist;

	  twist.linear.x = 0;
	  twist.linear.y = 0;
	  twist.linear.z = 0;
		
	  twist.angular.x = 0;
	  twist.angular.y = 0;
	  twist.angular.z = 0;

	  if (next_move == -1 || next_move == 1) 
	    { twist.linear.y = 0.1; twist.angular.z = next_move * 0.3; }
	  if (next_move == -2 || next_move == 2) 
	    { twist.linear.y = 0.3; }
	    
	  return twist;
  }

  DijkstraController::DijkstraController(ros::NodeHandle& nh):
	  map_ptr(NULL),
	  width(100),
	  height(100),
	  start(90),
	  goal(11),
	  next_movement(0)
  {
	  //map_sub = nh.subscribe(MAP_SUB_TOPIC, 10, &DijkstraController::MapCallback, this);
  }

  //Returns the next twist message to publish (this is called each main loop iteration)
  geometry_msgs::Twist DijkstraController::Update()
  {
	  if (dijkstras.Init(map_ptr, width, height, start, goal)) 
	  {
		  dijkstras.Search(dijkstras.GetStart());
		  dijkstras.ReconstructPath(dijkstras.GetGoal());
		  next_movement = dijkstras.GetNextStep();
	  }
	  
	  return GetTwistMsg(next_movement);
  }
}
