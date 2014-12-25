# pragma once

int last, start, goal;		// assigned when gps starts reading
int frontier[10];
int km = 0;
int rhs[100];
int g[100];
int c[100];
int prev_map[100];
int map[100];
int map_changes[10];		// only concerned about local surroundings
static int INF = 100000; // some arbitrarily large number


// For D* Lite
void Initialize();
void ComputerShortestPath();
void Heuristic(int end);	// simple, direct distance
int * Predecessors(int location);	
int * Successors(int location);
struct key CalculateKey(int location);
void UpdateLocation(int location);

// helper
int GetMinSuccessor();
bool CheckChangesInMap();

// need to create heap structure / class
// need to create key struct to use in heap
