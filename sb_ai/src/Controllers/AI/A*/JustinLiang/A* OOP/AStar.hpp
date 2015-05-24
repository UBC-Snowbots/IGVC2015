// Author: Justin Liang

#include <vector>

using namespace std;

class AStar {

public:

	AStar(int map_width, int map_height);
	~AStar();
	void AStarSearch(int start, int goal);
	void ReconstructPath(int start_location, int goal_location);

private:

	/** VARIABLES **/
	int _map_width;				// width of the map
	int _map_height;			// height of the map
	vector<int> _map;			// array that stores the main map

	vector<int> _frontier;			// priority queue that stores the set of locations we are currently searching through
	int _frontier_size;			// number of locations stored in the frontier array
	vector<int> _came_from;			// stores the parent locations of the locations on the map for easy backtracking when printing out the path
	vector<int> _cost_from_start;		// the accumulated cost from the start location to a location on the map
	vector<int> _estimated_cost;		// the estimated cost is the cost_from_start[location] + Heuristic(location)
	const int _INF = 200;

	/** FUNCTIONS **/
	void Init();
	void Swap(int one, int two);
	void FrontierPriority(int frontier_position);
	void Insert(int location);

	// TODO: Convert the location index to an X location value
	inline int GetX(int index) 
	{
		return index % _map_width;
	}

	// TODO: Convert the location index to a Y location value
	inline int GetY(int index) 
	{
		return index / _map_height;
	}

	// TODO: Convert coordinates (x,y) to an index that would be used to access the map array
	int GetIndex(int x, int y)
	{
		return y*_map_width + x;
	}

	int Heuristic(int start_location, int goal_location);
	bool NotAlreadyTravelled(int location, int curr_loc);
	bool NotInFrontier(int location);
	void AddNeighborsToFrontier(int current_location, int goal);
	void RemoveRoot();

};

