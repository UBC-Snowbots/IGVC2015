#include "GridWorld.h"

/*
BEFORE ATTEMPTING TO READ THIS FILE, PLEASE HAVE A BASIC UNDERSTANDING OF 
MT-D*LITE FROM READING ITS ORIGINAL RESEARCH PAPER'S PSEUDO-CODE.
*/
GridWorld::GridWorld(unsigned int size, int radius){
	this->size = size;
	this->radius = radius;

	for (unsigned int y = 0; y < size; y++){
		for (unsigned int x = 0; x < size; x++){
			world.push_back(new Tile(x, y, 10));
		}
	}

	goal = getTileAt(0, 0);
	start = getTileAt(size - 1, size - 1);

	//Initializing the pathfinder's default values
	km = 0;
	previous = goal;
	start->rhs = 0;

	start->isOpen = true;
	start->h = calculateH(start);
	start->key = GridWorld::KeyPair(start->h, 0);
	open.push_back(start);
}

/*
This method handles the inflation of surrounding tiles. When an obsticle
is detected, the surrounding tile's cost can be inflated to make the robot
avoid getting anywhere near the obsticle. By the nature of pathfinding on
grid maps, the resultant path tends to "hug the wall", however this will make
the path have some distance between itself and any obsticles
*/
void GridWorld::inflate(unsigned int x, unsigned int y, double newCost){
	updateCost(x, y, newCost);
	for (int dy = -radius; dy <= radius; dy++){
		for (int dx = -radius; dx <= radius; dx++){
			if (abs(dy) + abs(dx) != 0){
				Tile* t = getTileAt(x + dx, y + dy);
				if (t != 0 && newCost > t->cost){
					updateCost(x + dx, y + dy, INFLATION);
				}

			}
		}
	}
}

/*
This method does the same thing as the pseudo-code's updateVertex(),
except for grids instead of graphs.

Pathfinding algorimths tend to be demonstraited with a graph rather than a grid,
in order to update the cost between two tiles we must update both the tile and its neighbour.
*/
void GridWorld::updateCost(unsigned int x, unsigned int y, double newCost){
	static int count = 1;
	count++;
	Tile* tile = getTileAt(x, y);
	
	printf("Updating <%d, %d> from %2.0lf to %2.0lf - Update: %d\n", x, y, tile->cost, newCost, count);
	km += calculateH(previous);
	previous = goal;

	//I am aware that the following code below could be refactored by 50%
	//since it's repeating itself with only a few changes

	double oldCost = tile->cost;
	double oldCostToTile, newCostToTile;

	//Update CURRENT by finding its new minimum RHS-value from NEIGHBOURS
	std::vector<Tile*> neighbours(getNeighbours(tile));
	for (int i = 0; i < neighbours.size(); i++){
		tile->cost = oldCost;
		oldCostToTile = calculateC(tile, neighbours[i]);

		tile->cost = newCost;
		newCostToTile = calculateC(tile, neighbours[i]);

		if (oldCostToTile > newCostToTile){
			if (tile != start && tile->rhs > neighbours[i]->g + newCostToTile){
				tile->successor = neighbours[i];
				tile->rhs = neighbours[i]->g + newCostToTile;
			}
		}
		else if (tile != start && tile->successor == neighbours[i]){
			TilePair minSucc(getMinSuccessor(tile));
			tile->rhs = minSucc.second;
			tile->successor = (tile->rhs == PF_INFINITY ? 0 : minSucc.first);
		}
	}

	updateVertex(tile);

	//Update all NEIGHBOURING cells by finding their new min RHS-values from CURRENT
	for (int i = 0; i < neighbours.size(); i++){
		tile->cost = oldCost;
		oldCostToTile = calculateC(tile, neighbours[i]);

		tile->cost = newCost;
		newCostToTile = calculateC(tile, neighbours[i]);

		if (oldCostToTile > newCostToTile){
			if (neighbours[i] != start && neighbours[i]->rhs > tile->g + newCostToTile){
				neighbours[i]->successor = tile;
				neighbours[i]->rhs = tile->g + newCostToTile;
				updateVertex(neighbours[i]);
			}

		}
		else if (neighbours[i] != start && neighbours[i]->successor == tile){
			TilePair minSucc(getMinSuccessor(neighbours[i]));
			neighbours[i]->rhs = minSucc.second;
			neighbours[i]->successor = (neighbours[i]->rhs == PF_INFINITY ? 0 : minSucc.first);

			updateVertex(neighbours[i]);
		}
	}

	computeShortestPath();
}

bool GridWorld::computeShortestPath(){
	if (open.empty()){
		std::cout << "ERROR: No tiles to expand on... can't do anything" << std::endl;
		return false;
	}


	while (!open.empty() && (compareKeys(open.front()->key, goal->key = calculateKey(goal)) || goal->rhs != goal->g)){
		Tile* current = open.front();
		//Notice that CURRENT wasn't pop/removed yet

		//std::cout << "Expanding:";
		//current->info();
		//std::cout << std::endl;
		KeyPair k_new = calculateKey(current);

		if (compareKeys(current->key, k_new)){
			//Tiles under this branch will have to be updated as incremental search has happened
			current->key = k_new;
			make_heap(open.begin(), open.end(), GridWorld::compareTiles);

		}
		else if (current->g > current->rhs){
			//Majority of the tiles will fall under this conditional branch as
			//it undergoes normal A* pathfinding

			current->g = current->rhs;

			open.erase(open.begin());
			make_heap(open.begin(), open.end(), GridWorld::compareTiles);
			current->isOpen = false;

			std::vector<Tile*> neighbours(getNeighbours(current));
			for (int i = 0; i < neighbours.size(); i++){
				if (neighbours[i] != start && neighbours[i]->rhs > current->g + calculateC(current, neighbours[i])){
					neighbours[i]->successor = current;
					neighbours[i]->rhs = current->g + calculateC(current, neighbours[i]);
					updateVertex(neighbours[i]);
				}
			}

		}
		else {
			//Tiles under this branch will need to be updated during incremental search
			current->g = PF_INFINITY;

			//Update CURRENT
			if (current != start && current->successor == current){
				TilePair minSucc(getMinSuccessor(current));
				current->rhs = minSucc.second;
				current->successor = current->rhs == PF_INFINITY ? 0 : minSucc.first;
			}
			updateVertex(current);

			//Update NEIGHBOURS
			std::vector<Tile*> neighbours(getNeighbours(current));
			for (int i = 0; i < neighbours.size(); i++){
				if (neighbours[i] != start && neighbours[i]->successor == current){
					TilePair minSucc(getMinSuccessor(neighbours[i]));
					neighbours[i]->rhs = minSucc.second;
					neighbours[i]->successor = neighbours[i]->rhs == PF_INFINITY ? 0 : minSucc.first;
				}
				updateVertex(neighbours[i]);
			}

		}
		//Uncomment this to see CURRENT'S new values
		//std::cout << "Expanded:";
		//current->info();
		//std::cout << std::endl;
	}
	return true;
}

void GridWorld::updateVertex(GridWorld::Tile*& tile){
	bool isIncosistent = tile->rhs != tile->g; //potential problem with floating point comparison?

	if (isIncosistent && tile->isOpen){
		//tile->h = calculateH(tile);
		tile->key = calculateKey(tile);
		make_heap(open.begin(), open.end(), GridWorld::compareTiles);

	}
	else if (isIncosistent && !tile->isOpen){
		//tile->h = calculateH(tile);
		tile->key = calculateKey(tile);
		tile->isOpen = true;

		open.push_back(tile);
		push_heap(open.begin(), open.end(), GridWorld::compareTiles);

	}
	else if (!isIncosistent && tile->isOpen){
		open.erase(std::find(open.begin(), open.end(), tile));
		make_heap(open.begin(), open.end(), GridWorld::compareTiles);
		tile->isOpen = false;
	}

}


bool GridWorld::withinWorld(unsigned int x, unsigned int y) const{
	return y >= 0 && y < size && x >= 0 && x < size;
}

GridWorld::Tile* GridWorld::getTileAt(unsigned int x, unsigned int y) const{
	return withinWorld(x, y) ? world[y*size+x] : NULL;
}

std::vector<GridWorld::Tile*> GridWorld::getNeighbours(Tile*& tile){
	//It should be impossible to return a null neighbour, therefore I didn't bother
	//with null pointer checks whenever I use this method
	std::vector<GridWorld::Tile*> neighbours;
	for (int dy = -1; dy <= 1; dy++){
		for (int dx = -1; dx <= 1; dx++){
			if (abs(dy) + abs(dx) != 0){
				Tile* neighbour = getTileAt(tile->x + dx, tile->y + dy);
				if (neighbour != 0){
					neighbours.push_back(neighbour);
				}
			}
		}
	}

	return neighbours;
}

bool GridWorld::compareTiles(Tile*& left, Tile*& right){
	/*
	The heap functions from <algorithm> operates as a MAX heap,
	by making sure that left < right. In order to operate as a MIN heap
	we'll reverse the order by comparing right < left
	*/
	return compareKeys(right->key, left->key);
}

bool GridWorld::compareKeys(const KeyPair& left, const KeyPair& right){
	if (left.first < right.first){
		return true;
	}
	else if (left.first == right.first && left.second < right.second){
		return true;
	}
	else {
		return false;
	}
}

GridWorld::TilePair GridWorld::getMinSuccessor(GridWorld::Tile*& tile){
	Tile* minTile = 0;
	double minCost = PF_INFINITY;

	std::vector<Tile*> neighbours(getNeighbours(tile));
	for (int i = 0; i < neighbours.size(); i++){
		double cost = calculateC(tile, neighbours[i]);
		double g = neighbours[i]->g;

		if (cost == PF_INFINITY || g == PF_INFINITY){
			continue;
		}

		if (cost + g < minCost){ //potential overflow?
			minTile = neighbours[i];
			minCost = cost + g;
		}
	}

	return TilePair(minTile, minCost);
}

double GridWorld::calculateH(GridWorld::Tile*& tile){
	unsigned int dx = labs(tile->x - goal->x);
	unsigned int dy = labs(tile->y - goal->y);

	if (dx > dy){
		std::swap(dx, dy);
	}

	return ((SQRT2 - 1) * dx + dy) * 10;
}

double GridWorld::calculateC(GridWorld::Tile*& tileA, GridWorld::Tile*& tileB){
	if (tileA->cost == PF_INFINITY || tileB->cost == PF_INFINITY){
		return PF_INFINITY;
	}

	if (labs(tileA->x - tileB->x) + labs(tileA->y - tileB->y) == 2){
		//These two tiles are diagonally adjacent to each other
		return SQRT2 * (tileA->cost + tileB->cost) / 2;
	}
	return (tileA->cost + tileB->cost) / 2;
}

GridWorld::KeyPair GridWorld::calculateKey(GridWorld::Tile*& tile){
	double key2 = std::min(tile->g, tile->rhs);
	double key1 = key2 + calculateH(tile) + km;
	//H-value should be recalculated as it can change during incremental search

	return KeyPair(key1, key2);
}


GridWorld::Tile::Tile(unsigned int x, unsigned int y, double cost) : x(x), y(y){
	this->cost = cost;

	this->rhs = PF_INFINITY;
	this->g = PF_INFINITY;
	this->h = 0;

	this->isOpen = false;
	this->successor = 0;
}

GridWorld::Tile::Tile(Tile& other) : x(other.x), y(other.y){
	this->cost = other.cost;

	this->rhs = other.rhs;
	this->g = other.g;
	this->h = other.h;

	this->isOpen = other.isOpen;
	this->successor = other.successor;
}

void GridWorld::Tile::info() const{

	printf("[(%d, %d)  H: %.2lf, G: %.2lf, RHS: %.2lf, K:<%.2lf, %.2lf>]", this->x, this->y,
		this->h == PF_INFINITY ? -1 : this->h,
		this->g == PF_INFINITY ? -1 : this->g,
		this->rhs == PF_INFINITY ? -1 : this->rhs,
		this->key.first == PF_INFINITY ? -1 : this->key.first,
		this->key.second == PF_INFINITY ? -1 : this->key.second);

	if (this->successor != 0){
		printf("[P: (%u, %u)]", this->successor->x, this->successor->y);
	}
}

void GridWorld::printWorld() const{
	std::cout << "H:" << std::endl;
	for (unsigned int y = 0; y < size; y++){
		for (unsigned int x = 0; x < size; x++){
			printf("%2.0lf ", getTileAt(x, y)->h);
		}
		std::cout << std::endl;
	}

	std::cout << "C:" << std::endl;
	for (unsigned int y = 0; y < size; y++){
		for (unsigned int x = 0; x < size; x++){
			double cost = getTileAt(x, y)->cost;
			if (cost == PF_INFINITY){
				printf("-1 ");
			}
			else if (cost == INFLATION){
				printf("^^ ");
			}
			else{
				printf("%2.0lf ", cost);
			}

		}
		std::cout << std::endl;
	}

	std::cout << "G:" << std::endl;
	for (unsigned int y = 0; y < size; y++){
		for (unsigned int x = 0; x < size; x++){
			printf("%2.0lf ", getTileAt(x, y)->g == PF_INFINITY ? -1 : getTileAt(x, y)->g);
		}
		std::cout << std::endl;
	}

	std::cout << "RHS:" << std::endl;
	for (unsigned int y = 0; y < size; y++){
		for (unsigned int x = 0; x < size; x++){
			printf("%2.0lf ", getTileAt(x, y)->rhs == PF_INFINITY ? -1 : getTileAt(x, y)->rhs);
		}
		std::cout << std::endl;
	}
	std::cout << std::endl;
}
