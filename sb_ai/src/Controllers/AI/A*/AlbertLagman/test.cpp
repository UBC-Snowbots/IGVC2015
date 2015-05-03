/** This is the main function that runs when the program starts **/
#include <iostream>
#include "AStar.h"

int* generateMap(int width, int height)
{
	int x, y, index;
	int * map = new int[width * height];


	for (y = 0; y < height; y++) {
		for (x = 0; x < width; x++) {
			index = y*width + x;
			map[index] = 0;
		}
	}
	return map;
} 

int main()
{
	int * main_map;
	AStar star;
	main_map = generateMap(10,10);
	bool result=star.Init(main_map, 10, 10);
	star.AStarSearch(1, 99);
	return 0;
}
