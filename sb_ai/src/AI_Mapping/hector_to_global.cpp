#include <iostream>
#include <vector>
#include <stdlib.h>
#include "hector_to_global.h"

//CONSTRUCTOR
hector_to_global::hector_to_global (
	coordinate prev, 
	uint8_t rad,
	uint8_t mapsize,
	std::vector <int8_t>& hector,
	std::vector <int8_t>& global
	)
{
	previous_location = prev;
	searchradius = rad;
	hector_map = &hector;
	global_map = &global;
	size = mapsize;
}

void hector_to_global::test_print()
{
	std::cout << "Hector is: \n" << std::endl;
	for (int i = 0; i < size*size; i++)
	{
		std::cout << int(hector_map->at(i)) << "\t";
		if ((i % size) == size - 1)
			std::cout << "\n";
	}

	std::cout << "\n" << std::endl;
	std::cout << "Global is: \n" << std::endl;
	for (int i = 0; i < size*size; i++)
	{
		std::cout << int(global_map->at(i)) << "\t";
		if ((i % size) == size - 1)
			std::cout << "\n";
	}
	std::cout << "\n" << std::endl;
}

unsigned int hector_to_global::coord_to_single_dimension (const coordinate& location)	
{
	return (location.y * size + location.x);	
}

coordinate hector_to_global::dimension_to_coord(const int& arrayposition)
{
	coordinate coord;
	coord.x = arrayposition%size;
	coord.y = arrayposition/size;
	return coord;
}

void hector_to_global::update_hector(std::vector <int8_t>& hector)
{
	hector_map = &hector;
}

std::vector<int8_t>* hector_to_global::update_global()
{
	for (unsigned int i = 0; i < searchradius; i++)
	{
		for (unsigned int j = 0; j < searchradius; j++)
		{
			coordinate searchcoord;
			searchcoord.x = previous_location.x + i;
			searchcoord.y = previous_location.y + j;
			copy_coord_from_hector(searchcoord);

			searchcoord.x = previous_location.x - i;
			searchcoord.y = previous_location.y - j;
			copy_coord_from_hector(searchcoord);

			searchcoord.x = previous_location.x + i;
			searchcoord.y = previous_location.y - j;
			copy_coord_from_hector(searchcoord);

			searchcoord.x = previous_location.x - i;
			searchcoord.y = previous_location.y + j;
			copy_coord_from_hector(searchcoord);

		}	
	}
	return global_map;
}

void hector_to_global::copy_coord_from_hector(coordinate coord)
{
	if (coord.x > size - 1)
		coord.x = size - 1;
	else if (coord.x < 0)
		coord.x = 0;
	if (coord.y > size - 1)
		coord.y = size - 1;
	else if (coord.y < 0)
		coord.y = 0;
	unsigned int dimension = coord_to_single_dimension(coord);
	(*global_map).at(dimension) = (*hector_map).at(dimension);
}
