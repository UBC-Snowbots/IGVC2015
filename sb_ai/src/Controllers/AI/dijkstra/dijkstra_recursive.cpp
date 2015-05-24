#include <iostream>
#include "dijkstra_recursive.h"
#include <assert.h>

using namespace std;

// PRIVATE


int Dijkstra::ConvertToIndex(Location * xy)
{
	return (xy->y * map_width) + xy->x;
}

Location Dijkstra::ConvertToLocation(int n)
{
	Location xy;
	xy.x = n % map_width;
	xy.y = n / map_width;
	return xy; 
}

void Dijkstra::CheckBoundaries(Location * neighbor, int current)
{
	if (neighbor->x < 0 || neighbor->x >= map_width || neighbor->y < 0 || neighbor->y >= map_height) {
		*neighbor = ConvertToLocation(parent[current]);
	}
	return;
}



// PUBLIC	


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

int Dijkstra::GetStart() 
{
	return start;
}

int Dijkstra::GetGoal()
{
	return goal;
}

void Dijkstra::SetMap(int * curr_map) 
{ 
	map = curr_map; 
}

// ret -1,1 for horizontal, -2,2 for vertical, 0 for don't move
int Dijkstra::GetNextStep()
{
	if (first != -1) {
		Location first_loc = ConvertToLocation(first);
		Location curr_loc = ConvertToLocation(start);
		int x, y;
		x = first_loc.x - curr_loc.x;
		y = first_loc.y - curr_loc.y;
		if (x != 0) { return x; }
		else if (y != 0) { return 2*y; }
		else { return 0; }
	}
		
	return 0;	
}

bool Dijkstra::Init(int * main_map, int width, int height, int start_loc, int goal_loc)		// Executes the algorithm
{	
	map_size = width * height;
	
	// do a check to see if map, width, height, start, and goal are valid
	if (!main_map || width <= 0 || height <= 0 || start_loc < 0 || goal_loc < 0 
			|| start_loc >= map_size || goal_loc >= map_size)
		{ cout << "No map" << endl; return false; }
	
	map = main_map;
	map_width = width;
	map_height = height;
	
	start = start_loc;
	goal = goal_loc;
	
	parent = new int[map_size];
	distance = new int[map_size];
	destination = -1;
	
	for (int i=0; i < map_size; i++) {
		parent[i] = -1;
		distance[i] = -1;
	}
	parent[start] = start;
	distance[start] = 0;

	first = -1;
	second = -1;
	third = -1;
	
	return true;
}	


void Dijkstra::Search(int location) 
{
	if (location == goal) { return; }
	
	Location up, down, left, right, current_loc;
	current_loc = ConvertToLocation(location);
	up = current_loc;
	down = current_loc;
	left = current_loc;
	right = current_loc;
	up.y -= 1;
	down.y += 1;
	left.x -= 1;
	right.x += 1;
	
	CheckBoundaries(&up, location);
	CheckBoundaries(&down, location);
	CheckBoundaries(&left, location);
	CheckBoundaries(&right, location);

	int n, s, e, w;
	n = ConvertToIndex(&up);
	s = ConvertToIndex(&down);
	w = ConvertToIndex(&left);
	e = ConvertToIndex(&right);
		
	int possible_moves[4] = {n, s, e, w};
	int neighbor, temp_dist;
	
	for (int i = 0; i < 4; i++) {
	
		neighbor = possible_moves[i];
		
		
		if (neighbor != parent[location] && map[neighbor] != 1 ) {
			
			temp_dist = distance[location] + 1;
			
			// if neighbor is the goal
			if (neighbor == goal) {
				destination = goal;
				if(temp_dist < distance[goal] || distance[goal] == -1) {
					parent[goal] = location;
					distance[goal] = temp_dist;
				}
				Search(neighbor);
			}
			
			// neighbor distance == -1
			else {
				if (distance[neighbor] == -1 || temp_dist < distance[neighbor]) {
					parent[neighbor] = location;
					distance[neighbor] = temp_dist;
					Search(neighbor);
				}
			}
		}
	}
	
	return;
}


void Dijkstra::ReconstructPath(int location)
{
	if (destination == -1) {
		cout << "No path to goal location" << endl;
		return;
	}
	
	else {
		// make sure the parent of location is valid
		assert(parent[location] >= 0 && parent[location] < map_size);

		if (location == start) {
			cout << location << " ";
		}

		else {
			// get next 3 locations after start
			third = second;
			second = first;
			first = location;

			ReconstructPath(parent[location]);
			cout << location << " ";
		}
	}

	return;
}


// later on for turning
Location Dijkstra::GetDisplacement()
{
	Location displacement;

	if (destination != -1) {
		Location third_loc = ConvertToLocation(third);
		Location curr_loc = ConvertToLocation(start);
		displacement.x = third_loc.x - curr_loc.x;
		displacement.y = third_loc.y - curr_loc.y;
	}
	else {
		displacement.x = 0;
		displacement.y = 0;
	}

	return displacement;
}
