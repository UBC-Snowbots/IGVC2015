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

	int ConvertToIndex(Location xy);
	Location ConvertToLocation(int n);
	void CheckBoundaries(Location xy, int n);
	void CheckNeighbors(int n);
	void ReconstructPath();

public:

	void SetStart(int s);
	void SetGoal(int g);
	
	void Init(int * main_map, int width, int height);	
	void Search();	// Executes the algorithm
};