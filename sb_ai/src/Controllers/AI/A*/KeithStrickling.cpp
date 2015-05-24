#include <iostream>
#include <math.h>

using namespace std;

// Edited by Keith Strickling: keithstrickling@gmail.com

/** VARIABLES **/

int map_width = 10;				// width of the map
int map_height = 10;			// height of the map
int map[10*10];					// array that stores the main map

int frontier[10*10];			// priority queue that stores the set of locations we are currently searching through
int frontier_size = 0;			// number of locations stored in the frontier array
int came_from[10*10];			// stores the parent locations of the locations on the map for easy backtracking when printing out the path
int cost_from_start[10*10];		// the accumulated cost from the start location to a location on the map
int estimated_cost[10*10];		// the estimated cost is the cost_from_start[location] + Heuristic(location)
static int INF = 200;

// Initialization function for preparing the variables for A*
void Init()
{
	int x, y, index;
	
	for (y = 0; y < map_height; y++) {
		for (x = 0; x < map_width; x++) {
			index = y*map_width + x;
			map[index] = 0;
		}
	} 
	
	for (int i = 0; i < 100; i++) {
		came_from[i] = -1;
		frontier[i] = -1;
		cost_from_start[i] = INF;
		estimate_cost[i] = INF;
	}
	
}


// Input: 2 location indexes
// This swaps the cost between the two locations in the frontier array
void Swap(int one, int two) 
{
	int temp;
	temp = frontier[one];
	frontier[one] = frontier[two];
	frontier[two] = temp;
	return;
} 


// This finds the proper position of a location in the frontier
// This assumes that the location was inserted into the end of the frontier tree
void FrontierPriority(int frontier_position)
{
	// Check that there is a need to find priority
	if (frontier_size == 1) {
		return;
	}
	
	else {
		
		int parent = (frontier_position - 1) / 2;
		
		while (parent >= 0) {	// check that it's not out of range

			// The frontier is prioritizes the locations using their estimated costs
			if (estimated_cost[frontier[parent]] > estimated_cost[frontier[frontier_position]]) {
				Swap(parent, frontier_position); 
			}
			else { break; }

			parent = (parent - 1) / 2;
		}
		
		return;
	}
}


// This inserts a location into the frontier
void Insert(int location) 
{
	frontier[frontier_size] = location;
	FrontierPriority(frontier_size);
	frontier_size++;
	return;
}


// TODO: Heuristic function estimates the cost to travel from start_location to goal_location
// Admissible: Heuristic(start, goal) <= ActualCost(start, goal)
int Heuristic(int start_location, int goal_location)
{
	return 0;
}


// This prints out the path if a path is found
void ReconstructPath(int start_location, int goal_location)
{
	int curr = goal_location;
	while (curr != start_location) {
		cout << curr << endl;
		curr = came_from[curr];
	} 
	cout << curr << endl;
	return;
}



// TODO: Convert coordinates (x,y) to an index that would be used to access the map array
int GetIndex(int x, int y) 
{
	return 0; 
}


// TODO: Convert the location index to an X location value
int GetX(int index)
{
	return 0;
}


// TODO: Convert the location index to a Y location value
int GetY(int index) 
{
	return 0;
}


// This checks whether the cost from start value of the specified location requires overriding
bool NotAlreadyTravelled(int location, int curr_loc)
{
	if (came_from[location] == -1) { return true; }
	else if (cost_from_start[curr_loc] + 1 < cost_from_start[location]) { return true; }
	else { return false; }
}


// TODO: a search to check that the location is not found in the frontier array
bool NotInFrontier(int location)
{
	return true;
}


// This adds the next locations the robot needs to explore to the frontier array
void AddNeighborsToFrontier(int current_location, int goal)
{
	int neighbors[4];
	int x = GetX(current_location);
	int y = GetY(current_location);
	
	if (y-1 >= 0) { neighbors[0] = GetIndex(x, y-1); }	// Up
	else { neighbors[0] = -1; }
	
	if (x-1 >= 0) { neighbors[1] = GetIndex(x-1, y); }	// Left
	else { neighbors[1] = -1; }
	
	if (y+1 < 10) { neighbors[2] = GetIndex(x, y+1); }	// Down
	else { neighbors[2] = -1; }
	
	if (x+1 < 10) { neighbors[3] = GetIndex(x+1, y); }	// Right
	else { neighbors[3] = -1; }
	
	for (int i = 0; i < 4; i++) {
		if (neighbors[i] != -1) {
			if (NotAlreadyTravelled(neighbors[i], current_location) && NotInFrontier(neighbors[i])) {
				came_from[neighbors[i]] = current_location;
				cost_from_start[neighbors[i]] = cost_from_start[current_location] + 1;
				estimated_cost[neighbors[i]] = cost_from_start[neighbors[i]] + Heuristic(current_location, goal); 
				Insert(neighbors[i]);
			}
		}
	}
	
	return;
}


// This removes the location with the smallest cost out of the frontier array
void RemoveRoot()
{		
	int current_pos = 0;
	int smallest, other, smallest_val, other_val;
	frontier_size--;
	frontier[0] = frontier[frontier_size];
	frontier[frontier_size] = -1;		// invalidate position in heap
	
	smallest = (2*current_pos)+1;
	other = smallest+1;
	
	// only loop if left child is valid
	while (smallest < frontier_size) {
	
		smallest_val = frontier[smallest];
		
		// check if right child is invalid
		if (other >= frontier_size) {
			if (estimated_cost[smallest_val] < estimated_cost[frontier[current_pos]]) {
				Swap(current_pos, smallest);
				current_pos = smallest;
			}
			else { break; }
		}
		
		// otherwise compare both children and switch with smallest one
		else {

			other_val = frontier[other];
		
			if (estimated_cost[smallest_val] > estimated_cost[other_val]) {
				smallest = other;
				other = other - 1;
			}
			
			if (estimated_cost[smallest_val] < estimated_cost[frontier[current_pos]]) {
				Swap(current_pos, smallest);
				current_pos = smallest;
			}
		
			else {
				break;
			}	
		}	

		smallest = (2*current_pos)+1;
		other = smallest+1;
	} 
	
	return;
}


// This is the A* implementation. It is very similar to the one shown in Wiki.
// http://en.wikipedia.org/wiki/A*_search_algorithm
void AStarSearch(int start, int goal) 
{
	// TODO: check that the start and goal are valid integers 

	// TODO: cost_from_start[start] = ?
	// TODO: estimated_cost[start] = ?
	
	Insert(start);
	
	// keep searching for a path while there are still traversable locations
	while (frontier_size != 0) {
					
		// TODO: int current_location = ?
		
		// TODO: if the current_location == goal then ?

		// TODO: What function do you use to remove the current location from the frontier ?
		// TODO: There is a function that takes care of choosing the neighbors and adding them to the frontier
		
		frontier_size = 0;	// this is here just to make the initial template runs. Remove this when you no longer need it.
	}
	
	cout << "No path found" << endl;
	return;
}


/** This is the main function that runs when the program starts **/
int main()
{
	Init();
	AStarSearch(1, 99);
	return 0;
}
