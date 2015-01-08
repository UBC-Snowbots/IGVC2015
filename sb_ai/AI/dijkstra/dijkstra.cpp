#include <iostream>
#include "dijkstra.h"

using namespace std;

// PRIVATE


int Dijkstra::ConvertToIndex(Location xy)
{
	return 0;
}

Location Dijkstra::ConvertToLocation(int n)
{
	Location x;
	return x; 
}

void Dijkstra::CheckBoundaries(Location xy, int n)
{
	return;
}

void Dijkstra::CheckNeighbors(int n)
{
	return;
}

void Dijkstra::ReconstructPath()
{
	return;
}

void Dijkstra::SetStart(int s)
{
	start = s;
	return;
}

void Dijkstra::SetGoal(int g)
{
	goal = g;
	return;
}
	
// PUBLIC	

void Dijkstra::Init(int * main_map, int width, int height)		// Executes the algorithm
{
	map_size = width * height;
	
	// do a check to see if map, width, height, start, and goal are valid
	if (!map || width <= 0 || height <= 0 || start < 0 || goal < 0 
			|| start >= map_size || goal >= map_size)
		{ return; }
	
	map = main_map;
	map_width = width;
	map_height = height;
	
	parent = new int[map_size];
	distance = new int[map_size];
	destination = -1;
	
	for (int i=0; i < map_size; i++) {
		parent[i] = -1;
		distance[i] = -1;
	}
	parent[start] = start;
	distance[start] = 0;
	
	return;
}	

void Dijkstra::Search() 
{
	return;
}

int main()
{
	return 0;
}