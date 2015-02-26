#include "AStar.hpp"

#include <iostream>
#include <math.h>

using namespace std;

// Initialization function for preparing the variables for A*
AStar::AStar(int map_width, int map_height) :
			_map_width(map_width), _map_height(map_height),
			_frontier_size(0)
{
	_map.resize(_map_width * _map_height);
	_frontier.resize(_map_width * _map_height);
	_came_from.resize(_map_width * _map_height);
	_cost_from_start.resize(_map_width * _map_height);
	_estimated_cost.resize(_map_width * _map_height);

	int index;

	for (int y = 0; y < _map_height; y++) {
		for (int x = 0; x < _map_width; x++) {
			index = y*_map_width + x;
			_map[index] = 0;
		}
	}

	for (int i = 0; i < 100; i++) {
		_came_from[i] = -1;
		_frontier[i] = -1;
		_cost_from_start[i] = _INF;
		_estimated_cost[i] = _INF;
	}

}

AStar::~AStar()
{

}

// Input: 2 location indexes
// This swaps the cost between the two locations in the _frontier array
void AStar::Swap(int one, int two)
{
	int temp;
	temp = _frontier[one];
	_frontier[one] = _frontier[two];
	_frontier[two] = temp;
	return;
}

// This finds the proper position of a location in the _frontier
// This assumes that the location was inserted into the end of the _frontier tree
void AStar::FrontierPriority(int _frontier_position)
{
	// Check that there is a need to find priority
	if (_frontier_size == 1) {
		return;
	}

	else {

		int parent = (_frontier_position - 1) / 2;

		while (parent >= 0) {	// check that it's not out of range

			// The _frontier is prioritizes the locations using their estimated costs
			if (_estimated_cost[_frontier[parent]] > _estimated_cost[_frontier[_frontier_position]]) {
				Swap(parent, _frontier_position);
			}
			else { break; }

			parent = (parent - 1) / 2;
		}

		return;
	}
}

// This inserts a location into the _frontier
void AStar::Insert(int location)
{
	_frontier[_frontier_size] = location;
	FrontierPriority(_frontier_size);
	_frontier_size++;
	return;
}

// TODO: Heuristic function estimates the cost to travel from start_location to goal_location
// Admissible: Heuristic(start, goal) <= ActualCost(start, goal)
// Pythagoras. Assume admissible?
// May need to multiply by some scaling factor to represent physical distances
int AStar::Heuristic(int start_location, int goal_location)
{
	int start_x = GetX(start_location);
	int start_y = GetY(start_location);
	int goal_x = GetX(goal_location);
	int goal_y = GetY(goal_location);

	return (int)sqrt(pow((goal_x - start_x), 2) - pow((goal_x - start_y), 2));
}

// This prints out the path if a path is found
void AStar::ReconstructPath(int start_location, int goal_location)
{
	int curr = goal_location;
	while (curr != start_location) {
		cout << curr << " : (" << GetX(curr) << "," << GetY(curr) << ")" << endl;

		curr = _came_from[curr];
	}
	cout << curr << " : (" << GetX(curr) << "," << GetY(curr) << ")" << endl;
	return;
}

// This checks whether the cost from start value of the specified location requires overriding
bool AStar::NotAlreadyTravelled(int location, int curr_loc)
{
	if (_came_from[location] == -1) { return true; }
	else if (_cost_from_start[curr_loc] + 1 < _cost_from_start[location]) { return true; }
	else { return false; }
}


// TODO: a search to check that the location is not found in the _frontier array
bool AStar::NotInFrontier(int location)
{
	for (int i = 0; i < _frontier_size; i++) {
		if (_frontier[i] == location) {
			return false;
		}
	}
	return true;
}


// This adds the next locations the robot needs to explore to the _frontier array
void AStar::AddNeighborsToFrontier(int current_location, int goal)
{
	int neighbors[4];
	int x = GetX(current_location);
	int y = GetY(current_location);

	if (y - 1 >= 0) { neighbors[0] = GetIndex(x, y - 1); }	// Up
	else { neighbors[0] = -1; }

	if (x - 1 >= 0) { neighbors[1] = GetIndex(x - 1, y); }	// Left
	else { neighbors[1] = -1; }

	if (y + 1 < 10) { neighbors[2] = GetIndex(x, y + 1); }	// Down
	else { neighbors[2] = -1; }

	if (x + 1 < 10) { neighbors[3] = GetIndex(x + 1, y); }	// Right
	else { neighbors[3] = -1; }

	for (int i = 0; i < 4; i++) {
		if (neighbors[i] != -1) {
			if (NotAlreadyTravelled(neighbors[i], current_location) && NotInFrontier(neighbors[i])) {
				_came_from[neighbors[i]] = current_location;
				_cost_from_start[neighbors[i]] = _cost_from_start[current_location] + 1;
				_estimated_cost[neighbors[i]] = _cost_from_start[neighbors[i]] + Heuristic(current_location, goal);
				Insert(neighbors[i]);
			}
		}
	}

	return;
}

// This removes the location with the smallest cost out of the _frontier array
void AStar::RemoveRoot()
{
	int current_pos = 0;
	int smallest, other, smallest_val, other_val;
	_frontier_size--;
	_frontier[0] = _frontier[_frontier_size];
	_frontier[_frontier_size] = -1;		// invalidate position in heap

	smallest = (2 * current_pos) + 1;
	other = smallest + 1;

	// only loop if left child is valid
	while (smallest < _frontier_size) {

		smallest_val = _frontier[smallest];

		// check if right child is invalid
		if (other >= _frontier_size) {
			if (_estimated_cost[smallest_val] < _estimated_cost[_frontier[current_pos]]) {
				Swap(current_pos, smallest);
				current_pos = smallest;
			}
			else { break; }
		}

		// otherwise compare both children and switch with smallest one
		else {

			other_val = _frontier[other];

			if (_estimated_cost[smallest_val] > _estimated_cost[other_val]) {
				smallest = other;
				other = other - 1;
			}

			if (_estimated_cost[smallest_val] < _estimated_cost[_frontier[current_pos]]) {
				Swap(current_pos, smallest);
				current_pos = smallest;
			}

			else {
				break;
			}
		}

		smallest = (2 * current_pos) + 1;
		other = smallest + 1;
	}

	return;
}

// This is the A* implementation. It is very similar to the one shown in Wiki.
// http://en.wikipedia.org/wiki/A*_search_algorithm
void AStar::AStarSearch(int start, int goal)
{
	// TODO: check that the start and goal are valid integers 
	if (start < 0 || goal < 0 || start >= _map_width*_map_height || goal >= _map_width*_map_height) {
		return;
	}

	// TODO: _cost_from_start[start] = ?
	_cost_from_start[start] = 0;

	// TODO: _estimated_cost[start] = ?
	_estimated_cost[start] = Heuristic(start, goal);

	Insert(start);

	// keep searching for a path while there are still traversable locations
	while (_frontier_size != 0) {

		// TODO: int current_location = ?
		int current_location = _frontier[0];

		// TODO: if the current_location == goal then ?
		if (current_location == goal) {
			return ReconstructPath(start, goal);
		}

		// TODO: What function do you use to remove the current location from the _frontier ?
		RemoveRoot();

		// TODO: There is a function that takes care of choosing the neighbors and adding them to the _frontier
		AddNeighborsToFrontier(current_location, goal);

		//_frontier_size = 0;	// this is here just to make the initial template runs. Remove this when you no longer need it.
	}

	cout << "No path found" << endl;
	return;
}
