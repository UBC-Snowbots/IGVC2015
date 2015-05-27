#pragma once

struct Location
{
	int x;
	int y;
};

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
	Location ConvertToLocation(int n);
	void CheckBoundaries(Location * neighbor, int current);
	

public:

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
	void ReconstructPath(int location);	// change to bool
	Location GetDisplacement();
	
};
