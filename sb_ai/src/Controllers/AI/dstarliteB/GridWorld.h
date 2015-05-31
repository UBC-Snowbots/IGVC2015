#define __STDC_LIMIT_MACROS
#include <stdint.h>
#include <climits>
#include <stdio.h>
#include <algorithm>
#include <vector>
#include <iostream>

#define PF_INFINITY INT_MAX
#define INFLATION INT_MAX/2
#define SQRT2 1.4142135623

class GridWorld{

	typedef std::pair<double, double> KeyPair;

public:
	struct Tile{
		const unsigned int x, y;
		double rhs, g, h, cost;
		KeyPair key;
		bool isOpen;
		Tile* successor;

		Tile(unsigned int x, unsigned int y, double cost);
		Tile(Tile& other);

		void info() const;
	};

	typedef std::pair<GridWorld::Tile*, double> TilePair;

	bool withinWorld(unsigned int x, unsigned int y) const;
	bool static compareKeys(const KeyPair& left, const KeyPair& right);
	bool static compareTiles(Tile*& left, Tile*& right);

	bool computeShortestPath();
	void updateVertex(Tile*& tile);

	double calculateH(Tile*& tile);
	double calculateC(Tile*& tileA, Tile*& tileB);
	KeyPair calculateKey(Tile*& tile);

	Tile* getTileAt(unsigned int x, unsigned int y) const;

	TilePair getMinSuccessor(Tile*& tile);
	std::vector<Tile*> getNeighbours(Tile*& tile);

	//------------------------------------------------
	unsigned int size;
	int radius;
	double km;

	Tile* start;
	Tile* goal;
	Tile* previous;

	std::vector<Tile*> open;
	std::vector<Tile*> world;


	GridWorld(unsigned int size, int radius);
	void printWorld() const;
	void updateCost(unsigned int x, unsigned int y, double cost);
	void inflate(unsigned int x, unsigned int y, double cost);
};
