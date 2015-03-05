/** This is the main function that runs when the program starts **/
#include <iostream>
#include "AStar.h"

int main()
{
	AStar star;
	bool result=star.Init();
	star.AStarSearch(1, 99);
	return 0;
}
