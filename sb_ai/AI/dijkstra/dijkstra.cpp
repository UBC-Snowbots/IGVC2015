#include <iostream>
#include "dijkstra.h"

using namespace std;

// PRIVATE


int Dijkstra::ConvertToIndex(Location xy)
{
	return xy.y * map_width + xy.x;
}

Location Dijkstra::ConvertToLocation(int n)
{
	Location xy;
	xy.x = n % map_width;
	xy.y = n / map_width;
	return xy; 
}

void Dijkstra::CheckBoundaries(Location neighbor, int current)
{
	if (neighbor.x < 0 || neighbor.x >= map_width || neighbor.y < 0 || neighbor.y >= map_height) {
		neighbor = ConvertToLocation(parent[current]);
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

bool Dijkstra::Init(int * main_map, int width, int height)		// Executes the algorithm
{	
	// do a check to see if map, width, height, start, and goal are valid
	if (!main_map || width <= 0 || height <= 0 || start < 0 || goal < 0 
			|| start >= map_size || goal >= map_size)
		{ return false; }
	
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
	
	return true;
}	


void Dijkstra::Search(int location) 
{
	if (start == goal) { return; }
	
	Location up, down, left, right, current_loc;
	current_loc = ConvertToLocation(location);
	up = current_loc;
	down = current_loc;
	left = current_loc;
	right = current_loc;
	up.y --;
	down.y ++;
	left.x --;
	right.x ++;
	
	CheckBoundaries(up, location);
	CheckBoundaries(down, location);
	CheckBoundaries(left, location);
	CheckBoundaries(right, location);
	
	int n, s, e, w;
	n = ConvertToIndex(up);
	s = ConvertToIndex(down);
	w = ConvertToIndex(left);
	e = ConvertToIndex(right);

	int possible_moves[4] = {n, s, e, w};
	int neighbor, temp_dist;
	
	for (int i = 0; i < 4; i++) {
	
		neighbor = possible_moves[i];
		
		if (neighbor != -1 && neighbor != parent[current_loc] && map[neighbor] != 1) {
			
			temp_dist = distance[current_loc]++;
			
			// if neighbor is the goal
			if (neighbor == goal) {
				destination = goal;
				if(temp_dist < distance[goal]) {
					parent[goal] = current_loc;
					distance[goal] = temp_dist;
				}
			}
			
			// neighbor distance == -1
			else {
				if (distance[neighbor] == -1 || distance[neighbor] < temp_dist) {
					parent[neighbor] = current_loc;
					distance[neighbor] = temp_dist;
					Search(neighbor);
				}
			}
			
		}
		
	}
	
	return;
}

void Dijkstra::ReconstructPath()
{
	if (destination == -1) {
		cout << "No path to goal location" << endl;
	}
	
	
	return;
}


int main()
{
	int * map;	// TODO
	Dijkstra dijkstras;
	dijkstras.SetStart(45);
	dijkstras.SetGoal(21);
	dijkstras.Init(map, 10, 10);
	dijkstras.Search(dijkstras.GetStart());
	dijkstras.ReconstructPath();
	return 0;
}