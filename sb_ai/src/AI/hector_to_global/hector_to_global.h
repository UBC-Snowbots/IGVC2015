#pragma once
#include <iostream>
#include <stdlib.h>
#include <vector>

struct coordinate
{
	int x;
	int y;	
};

class hector_to_global
{
private:
	std::vector <int8_t>* hector_map;
	std::vector <int8_t>* global_map;
	coordinate previous_location;
	uint8_t searchradius;
	uint8_t size; //size n is an n x n square
	unsigned int coord_to_single_dimension ();
	void copy_coord_from_hector(coordinate coord);

	unsigned int coord_to_single_dimension (const coordinate& location);
	coordinate dimension_to_coord(const int& arrayposition);
	

public:
	void test_print();
	hector_to_global (
		coordinate prev, 		//becomes previous_location
		uint8_t rad,			//becomes search radius
		uint8_t mapsize,		//becomes size
		std::vector <int8_t>& hector,	//becomes hector_map
		std::vector <int8_t>& global	//becomes global_map
		);

	void update_hector(std::vector <int8_t>& hector);
	std::vector<int8_t>* update_global();
};


