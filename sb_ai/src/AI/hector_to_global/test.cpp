#include <iostream>
#include <vector>
#include "hector_to_global.h"
#include <stdlib.h>

#define SIZE 8
#define RADIUS 2
int main()
{
	std::vector<int8_t> hector;
	std::vector<int8_t> global;

	for (int i = 0; i < 64; i++)
	{
		hector.push_back(1);
		global.push_back(0);
	}
	
	coordinate previousposition;
	previousposition.x = 7;
	previousposition.y = 0;

	hector_to_global h2g(previousposition,RADIUS,SIZE,hector, global);
	h2g.test_print();
	h2g.update_global();
	std::cout << "After update:\n" << std::endl;
	h2g.test_print();
	return 0;
}
