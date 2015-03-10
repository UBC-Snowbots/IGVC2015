#pragma once

class AStar
{
private:
	int map_width;
	int map_height;
	int * map;
	int * frontier;
	int frontier_size;
	int * came_from;
	int * cost_from_start;
	int * estimated_cost;
	static const int INF = 2000;
	void Swap(int one, int two);
	int GetIndex(int x, int y);
	int GetX(int index); //intrinsic calculation
	int GetY(int index);
	void FrontierPriority(int frontier_position);
	void Insert(int location);
	bool NotAlreadyTravelled(int location, int curr_loc);
	int Heuristic(int start_location, int goal_location);
	bool NotInFrontier(int location);
	void AddNeighborsToFrontier(int current_location, int goal);
	void RemoveRoot();

public:
	bool Init(int* map, int width, int height);
	void ReconstructPath(int start_location, int goal_location);
	void AStarSearch(int start, int goal);
};
