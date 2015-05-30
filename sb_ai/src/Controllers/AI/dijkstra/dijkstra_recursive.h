#pragma once
#include "../structures.h"
#include <geometry_msgs/Twist.h>

#define MAX_ANG_VEL 0.3f
#define MAX_LIN_VEL 0.3f

class Dijkstra
{
private:
	
	int * map;
	int map_width, map_height, map_size;
	int start, goal;
	int * parent;	// predecessors
	int * distance;	// distance from start, -1 if not travelled
	int destination;
	int first, second, third, fourth;
	int ConvertToIndex(Location * xy);
	void CheckBoundaries(Location * neighbor, int current);
	
public:

  Location ConvertToLocation(int n);

	// set these before starting
	void SetStart(int s);
	void SetGoal(int g);
	void SetMap(int * curr_map);
	
	int GetStart();
	int GetGoal();

	int GetNextStep();	// for testing horizontal and vertical only
	void GetPathIndex(int &firstIndex, int &fourthIndex, int location);
	
	bool Init(int * main_map, int width, int height, int start_loc, int goal_loc);	
	void Search(int location);	// Executes the algorithm
	void ReconstructPath(int location);	
	void SetFirstAndThird(int &first_index, int &third_index);
	Location GetDisplacement();
	
	geometry_msgs::Twist GetVelocity(Location* next_targets, float elsa_yaw, geometry_msgs::Twist &elsa_command);
	
};
