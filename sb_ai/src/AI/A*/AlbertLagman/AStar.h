#pragma once

class AStar
{
private:
	int map_width = 10;
	int map_height = 10;
	int map[100];
	int frontier[100];
	int frontier_size = 0;
	int came_from[100];
	int cost_from_start[100];
	int estimated_cost[100];
	static const int INF = 200;
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
	bool Init();
	void ReconstructPath(int start_location, int goal_location);
	void AStarSearch(int start, int goal);
};
