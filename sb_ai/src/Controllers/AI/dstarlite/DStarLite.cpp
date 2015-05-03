#include <iostream>
#include "DStarLite.h"
#include "PriorityQueue.h"

using namespace std;




int main() 
{
	last = start;
	
	Initialize();
	
	ComputerShortestPath();
	
	while (start != goal) {
		
		if (rhs[start] == INF) { 
			cout << "No path found." << endl;
			return 0; 
		}
		
		start = GetMinSuccessor();
		// send message to robot to move to start
		
		if (CheckChangesInMap()) {
			// update everything and recalculate shortest path again
		}
		
	}
	return 0;
}