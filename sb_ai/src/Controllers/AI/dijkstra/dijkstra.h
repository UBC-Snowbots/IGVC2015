#pragma once

#include <queue>          // std::priority_queue
#include <vector>         // std::vector
#include <functional>     // std::greater

#define INVALID -1  // for now

struct Location
{
	int x;
	int y;
};

class Dijkstra
{
private:
	
  int start, goal;
  int map_width, map_height, map_size;
  int * dist; // distances to vertex i from start
  int * prev; // index of previously travelled vertex
  int * travelled;  // checks for finished/unfinished vertices
	bool AreAllTravelled();
	
public:

  bool Init(int start_v, int goal_v, int width, int height);
	
};
