#include "dijkstra.h"
#include <iostream>

bool Dijkstra::AreAllTravelled()
{
  return true;
}

bool Dijkstra::Init(int start_v, int goal_v, int width, int height)
{
  start = start_v;
  goal = goal_v;
  
  map_width = width;
  map_height = height;
  map_size = width * height;
  
  if (start < 0 || start > map_size) { return false; }
  if (goal < 0 || goal > map_size) { return false; }
  
  dist = new int[map_size];
  prev = new int[map_size];
  travelled = new int[map_size];
  
  if (!(dist && prev && travelled))
  {
    std::cout << "Arrays failed to initialize." << std::endl;
    return false;
  }
  
  for (int i = 0; i < map_size; i++)
  {
    dist[i] = INVALID;
    prev[i] = INVALID;
    travelled[i] = 0;
  }
  
  dist[start] = 0;
  
  std::cout << "...Initialization successful." << std::endl;
  return true;
}

int main()
{
  Dijkstra algorithm;
  if (!algorithm.Init(3, 67, 100, 100)) 
  { 
    std::cout << "...Initialization failed." << std::endl; 
    return 0; 
  }
  return 0;
}
