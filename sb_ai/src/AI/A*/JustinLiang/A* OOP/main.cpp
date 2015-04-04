#include "Astar.hpp"

/** This is the main function that runs when the program starts **/
int main()
{

	AStar AStarMap(10, 10);
	AStarMap.AStarSearch(1, 99);

	system("PAUSE");
	return 0;
}
