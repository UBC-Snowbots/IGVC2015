#include "dijkstra.h"
#include <iostream>

using namespace std;

int * RandomMap()
{
	int * map = new int[100];	
	for (int i = 0; i < 100; i++) {
		map[i] = 0;
	}
	return map;
}

int main()
{
	Dijkstra dijkstras;
	dijkstras.Init(RandomMap(), 10, 10, 14, 98);
	dijkstras.Search(dijkstras.GetStart());
	dijkstras.ReconstructPath(dijkstras.GetGoal());
	return 0;
}