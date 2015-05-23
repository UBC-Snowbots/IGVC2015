#include "GridWorld.h"

GridWorld::GridWorld(unsigned int size){
	this->size = size;
	world.reserve(size * size);
	for (unsigned int y = 0; y < size; y++){
		for (unsigned int x = 0; x < size; x++){
			world.push_back(new Tile(x, y, 10));
		}
	}

	goal = getTileAt(0,0);
	start = getTileAt(size - 1, size - 1);

	//INIT
	km = 0;
	oldGoal = goal;
	start->rhs = 0;
	
	start->isOpen = true;
	start->h = calculateH(start);
	start->key = GridWorld::KeyPair(start->h, 0);
	open.push_back(start);
}



void GridWorld::updateCost(unsigned int x, unsigned int y, double newCost){
	Tile* tile = getTileAt(x,y);
	printf("\tUpdating: X:%d Y:%d\n", tile->x, tile->y);
	km += calculateH(oldGoal);
	oldGoal = goal;

	//I am aware that the following code below could be refactored by 50%
	//since it's repeating itself with only a few changes

	double oldCost = tile->cost;
	double oldCostToTile, newCostToTile;

	//Update CURRENT by finding its new minimum RHS-value from NEIGHBOURS
	std::vector<Tile*> neighbours(getNeighbours(tile));
	for (std::vector<Tile*>::iterator neighbour = neighbours.begin(); neighbour != neighbours.end(); ++neighbour){
		if (newCost == INFINITY && (*neighbour)->cost < newCost){
			(*neighbour)->cost = INFINITY / 2;
		}

		tile->cost = oldCost;
		oldCostToTile = calculateC(tile, *neighbour);

		tile->cost = newCost;
		newCostToTile = calculateC(tile, *neighbour);
		
		if (oldCostToTile > newCostToTile){
			if(tile != start && tile->rhs > (*neighbour)->g + newCostToTile){
				tile->parent = (*neighbour);
				tile->rhs = (*neighbour)->g + newCostToTile;
			}
		}
		else if (tile != start && tile->parent == (*neighbour)){
			TilePair minSucc(getMinSuccessor(tile));
			tile->rhs = minSucc.second;
			tile->parent = (tile->rhs == INFINITY ? 0 : minSucc.first);
		}
	}

	updateVertex(tile);

	//Update all NEIGHBOURING cells by finding their new min RHS-values from CURRENT
	//std::vector<Tile*> neighbours(getNeighbours(tile));
	for (std::vector<Tile*>::iterator neighbour = neighbours.begin(); neighbour != neighbours.end(); ++neighbour){
		tile->cost = oldCost;
		oldCostToTile = calculateC(tile, *neighbour);

		tile->cost = newCost;
		newCostToTile = calculateC(tile, *neighbour);

		if (oldCostToTile > newCostToTile){
			if(*neighbour != start && (*neighbour)->rhs > tile->g + newCostToTile){
				(*neighbour)->parent = tile;
				(*neighbour)->rhs = tile->g + newCostToTile;
				updateVertex(*neighbour);
			}

		} else if (*neighbour != start && (*neighbour)->parent == tile){
			TilePair minSucc(getMinSuccessor(*neighbour));
			(*neighbour)->rhs = minSucc.second;
			(*neighbour)->parent = ((*neighbour)->rhs == INFINITY ? 0 : minSucc.first);
			updateVertex(*neighbour);
		}
	}
	
	computeShortestPath();
}

bool GridWorld::computeShortestPath(){
	if(open.empty()){
		std::cout << "ERROR: No tiles to expand on... can't do anything" << std::endl;
		return false;
	}


	while(!open.empty() && (compareKeys(open.front()->key, goal->key = calculateKey(goal)) || goal->rhs != goal->g)){
		Tile* current = open.front();
		//Notice that CURRENT wasn't pop/removed yet

		//std::cout << "Expanding:";
		//current->info();
		//std::cout << std::endl;
		KeyPair k_new = calculateKey(current);
		
		if(compareKeys(current->key, k_new)){
			//Tiles under this branch will have to be updated as incremental search has happened
			current->key = k_new;
			make_heap(open.begin(), open.end(), GridWorld::compareTiles);

		} else if (current->g > current->rhs){
			//Majority of the tiles will fall under this conditional branch as
			//it undergoes normal A* pathfinding

			current->g = current->rhs;

			open.erase(open.begin());
			make_heap(open.begin(), open.end(), GridWorld::compareTiles);
			current->isOpen = false;

			std::vector<Tile*> neighbours(getNeighbours(current));
			for (std::vector<Tile*>::iterator neighbour = neighbours.begin(); neighbour != neighbours.end(); ++neighbour){
				if(*neighbour != start && (*neighbour)->rhs > current->g + calculateC(current, *neighbour)){
					(*neighbour)->parent = current;
					(*neighbour)->rhs = current->g + calculateC(current, *neighbour);
					updateVertex(*neighbour);
				}
			}

		} else {
			//Tiles under this branch will need to be updated during incremental search
			current->g = INFINITY;

			//Update CURRENT
			if(current != start && current->parent == current){
				TilePair minSucc(getMinSuccessor(current));
				current->rhs = minSucc.second;
				current->parent = current->rhs == INFINITY ? 0 : minSucc.first;
			}
			updateVertex(current);

			//Update NEIGHBOURS
			std::vector<Tile*> neighbours(getNeighbours(current));
			for (std::vector<Tile*>::iterator neighbour = neighbours.begin(); neighbour != neighbours.end(); ++neighbour){
				if (*neighbour != start && (*neighbour)->parent == current){
					TilePair minSucc(getMinSuccessor(*neighbour));
					(*neighbour)->rhs = minSucc.second;
					(*neighbour)->parent = (*neighbour)->rhs == INFINITY ? 0 : minSucc.first;
				}
				updateVertex(*neighbour);
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

	if(isIncosistent && tile->isOpen){
		//tile->h = calculateH(tile);
		tile->key = calculateKey(tile);
		make_heap(open.begin(), open.end(), GridWorld::compareTiles);
	
	}else if(isIncosistent && !tile->isOpen){
		//tile->h = calculateH(tile);
		tile->key = calculateKey(tile);
		tile->isOpen = true;
		
		open.push_back(tile);
		push_heap(open.begin(), open.end(), GridWorld::compareTiles);
		
	}else if (!isIncosistent && tile->isOpen){
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

bool GridWorld::isDiagonal(GridWorld::Tile*& tileA, GridWorld::Tile*& tileB){
	return labs(tileA->x - tileB->x) + labs(tileA->y - tileB->y) == 2;
}

std::vector<GridWorld::Tile*> GridWorld::getNeighbours(Tile*& tile){
	//It should be impossible to return a null neighbour, therefore I didn't bother
	//with null pointer checks whenever I use this method
	std::vector<GridWorld::Tile*> neighbours;
	for (int dy = -1; dy <= 1; dy++){
		for (int dx = -1; dx <= 1; dx++){
			if(abs(dy) + abs(dx) == 1){
				//For 4 neighbours: abs(dy) + abs(dx) == 1
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
	if(left.first < right.first){
		return true;
	} else if(left.first == right.first && left.second < right.second){
		return true;
	} else {
		return false;
	}
}

GridWorld::TilePair GridWorld::getMinSuccessor(GridWorld::Tile*& tile){
	Tile* minTile = 0;
	double minCost = INFINITY;

	std::vector<Tile*> neighbours(getNeighbours(tile));
	for (std::vector<Tile*>::iterator neighbour = neighbours.begin(); neighbour != neighbours.end(); ++neighbour){
		double cost = calculateC(tile, *neighbour);
		double g = (*neighbour)->g;
			
		if(cost == INFINITY || g == INFINITY){
			continue;
		}
			
		if(cost + g < minCost){ //potential overflow?
			minTile = (*neighbour);
			minCost = cost + g;
		}
	}
	
	return TilePair(minTile, minCost);
}

double GridWorld::calculateH(GridWorld::Tile*& tile){
	unsigned int dx = labs(tile->x - goal->x);
	unsigned int dy = labs(tile->y - goal->y);

	if(dx > dy){
		std::swap(dx,dy);
	}

	tile->h = ((SQRT2-1) * dx + dy) * 10;
	return tile->h;
}

double GridWorld::calculateC(GridWorld::Tile*& tileA, GridWorld::Tile*& tileB){
	if(tileA->cost == INFINITY || tileB->cost == INFINITY){
		return INFINITY;
	}
	/*
	if(labs(tileA->x - tileB->x) + labs(tileA->y - tileB->y) == 2){
		//These two tiles are diagonally adjacent to each other
		Tile* vertical = getTileAt(tileB->x, tileB->y - (tileA->y - tileB->y));
		Tile* horizontal = getTileAt(tileB->x - (tileA->x - tileB->x), tileB->y);
		
		if(vertical != 0 && vertical->cost == INFINITY || horizontal != 0 && horizontal->cost == INFINITY){
			return INFINITY;
		}

		return SQRT2 * (tileA->cost + tileB->cost) / 2;
	}*/
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

	this->rhs = INFINITY;
	this->g = INFINITY;
	this->h = 0;

	this->isOpen = false;
	this->parent = 0;
}

GridWorld::Tile::Tile(Tile& other) : x(other.x), y(other.y){
	this->cost = other.cost;

	this->rhs = other.rhs;
	this->g = other.g;
	this->h = other.h;

	this->isOpen = other.isOpen;
	this->parent = other.parent;
}

void GridWorld::Tile::info() const{

	printf("[(%d, %d)  H: %.2lf, G: %.2lf, RHS: %.2lf, K:<%.2lf, %.2lf>]", this->x, this->y,
		this->h == INFINITY ? -1 : this->h, 
		this->g == INFINITY ? -1 : this->g,
		this->rhs == INFINITY ? -1 : this->rhs, 
		this->key.first == INFINITY ? -1 : this->key.first,
		this->key.second == INFINITY ? -1 : this->key.second);

	if(this->parent != 0){
		printf("[P: (%u, %u)]", this->parent->x, this->parent->y);
	}
}

void GridWorld::printWorld() const{
std::cout << "H:" << std::endl;
	for (unsigned int y = 0; y < size; y++){
		for (unsigned int x = 0; x < size; x++){
			printf("%2.0lf ", getTileAt(x,y)->h);
		}
		std::cout << std::endl;
	}

	std::cout << "C:" << std::endl;
	for (unsigned int y = 0; y < size; y++){
		for (unsigned int x = 0; x < size; x++){
			Tile* t(getTileAt(x, y));
			printf("%2.0lf ", getTileAt(x,y)->cost == INFINITY ? -1 : getTileAt(x,y)->cost);
		}
		std::cout << std::endl;
	}

	std::cout << "G:" << std::endl;
	for (unsigned int y = 0; y < size; y++){
		for (unsigned int x = 0; x < size; x++){
			printf("%2.0lf ", getTileAt(x,y)->g == INFINITY ? -1 : getTileAt(x,y)->g);
		}
		std::cout << std::endl;
	}

	std::cout << "RHS:" << std::endl;
	for (unsigned int y = 0; y < size; y++){
		for (unsigned int x = 0; x < size; x++){
			printf("%2.0lf ", getTileAt(x,y)->rhs == INFINITY ? -1 : getTileAt(x,y)->rhs);
		}
		std::cout << std::endl;
	}
	std::cout << std::endl;
}