#include "GridWorld.h"

/*
BEFORE ATTEMPTING TO READ THIS FILE, PLEASE HAVE A BASIC UNDERSTANDING OF D*LITE
FROM READING ITS ORIGINAL RESEARCH PAPER'S PSEUDO-CODE. 
*/

GridWorld::GridWorld(unsigned int size, int radius){
	this->size = size;
	this->radius = radius;
	for (unsigned int y = 0; y < size; y++){
		for (unsigned int x = 0; x < size; x++){
			world.push_back(new Tile(x, y, 10));
		}
	}

	start = getTileAt(0,0);
	goal = getTileAt(size - 1, size - 1);

	//Initializing the pathfinder's default values
	km = 0;
	previous = start;
	goal->rhs = 0;
	
	goal->isOpen = true;
	goal->h = calculateH(goal);
	goal->key = GridWorld::KeyPair(goal->h, 0);
	open.push_back(goal);
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
					updateCost(x+dx, y+dy, INFLATION);
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
	Tile* tile = getTileAt(x,y);

	//printf("<%d, %d>\n", x, y);
	km += calculateH(previous);
	previous = start;

	//I am aware that the following code below could be refactored by 50%
	//since it's repeating itself with only a few changes

	double oldCost = tile->cost;	
	double oldCostToTile, newCostToTile;
	double currentRHS, otherG;
	
	//Update CURRENT by finding its new minimum RHS-value from NEIGHBOURS
	for(Tile* neighbour : getNeighbours(tile)){
		tile->cost = oldCost;
		oldCostToTile = calculateC(tile, neighbour);
		
		tile->cost = newCost;
		newCostToTile = calculateC(tile, neighbour);

		currentRHS = tile->rhs;
		otherG = neighbour->g;

		if(oldCostToTile > newCostToTile){
			if(tile != goal){
				tile->rhs = std::min(currentRHS, (newCostToTile + otherG));
			}
		}else if (currentRHS == (oldCostToTile + otherG)){
			if(tile != goal){
				tile->rhs = getMinSuccessor(tile).second;
			}
		}
	}

	updateVertex(tile);
	
	//Update all NEIGHBOURING cells by finding their new min RHS-values from CURRENT
	for (Tile* neighbour : getNeighbours(tile)){
		tile->cost = oldCost;
		oldCostToTile = calculateC(tile, neighbour);
		
		tile->cost = newCost;
		newCostToTile = calculateC(tile, neighbour);

		currentRHS = neighbour->rhs;
		otherG = tile->g;

		if(oldCostToTile > newCostToTile){
			if(neighbour != goal){
				neighbour->rhs = std::min(currentRHS, (newCostToTile + otherG));
			}
		} else if (currentRHS == (oldCostToTile + otherG)){
			if(neighbour != goal){
				neighbour->rhs = getMinSuccessor(neighbour).second;
			}
		}
		updateVertex(neighbour);
	}
	
	computeShortestPath();
}

bool GridWorld::computeShortestPath(){
	if(open.empty()){
		std::cout << "ERROR: No tiles to expand on... can't do anything" << std::endl;
		return false;
	}

	double currentRHS, otherG, previousG;

	while(!open.empty() && (compareKeys(open.front()->key, start->key = calculateKey(start)) || start->rhs != start->g)){
		Tile* current = open.front();
		//Notice that CURRENT wasn't pop/removed yet..

		KeyPair k_old = current->key;
		KeyPair k_new = calculateKey(current);

		currentRHS = current->rhs;
		otherG = current->g;

		/*std::cout << "Expanding:";
		current->info();
		std::cout << std::endl;*/

		if(compareKeys(k_old, k_new)){
			//This branch updates tile that were already in the OPEN list originally
			//This branch tends to execute AFTER the else branch
			current->key = k_new;
			make_heap(open.begin(), open.end(), GridWorld::compareTiles);

		} else if (otherG > currentRHS){
			//Majority of the execution will fall under this conditional branch as
			//it is undergoing normal A* pathfinding

			current->g = current->rhs;
			otherG = currentRHS;

			open.erase(open.begin());
			make_heap(open.begin(), open.end(), GridWorld::compareTiles);
			current->isOpen = false;

			for (Tile* neighbour : getNeighbours(current)){
				if(neighbour != 0){
					if(neighbour != goal){
						neighbour->rhs = std::min(neighbour->rhs, calculateC(current, neighbour) + otherG);
					}
					updateVertex(neighbour);
				}
			}

		} else {
			//Execution of this branch updates the tile during incremental search

			previousG = otherG;
			current->g = INFINITY;

			//Update CURRENT'S RHS
			if(current != goal){
				current->rhs = getMinSuccessor(current).second;
			}
			updateVertex(current);

			//Update NEIGHBOUR'S RHS to their minimum successor
			for (Tile* neighbour : getNeighbours(current)){
				if(neighbour != 0){
					if(neighbour->rhs == (calculateC(current, neighbour) + previousG) && neighbour != goal){
						neighbour->rhs = getMinSuccessor(neighbour).second;
					}
					updateVertex(neighbour);
				}
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
	bool isIncosistent = tile->rhs != tile->g;

	if(isIncosistent && tile->isOpen){
		tile->key = calculateKey(tile);
		make_heap(open.begin(), open.end(), GridWorld::compareTiles);
	
	}else if(isIncosistent && !tile->isOpen){
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
	return withinWorld(x,y) ? (world.at(y*size+x)) : NULL;
}

std::vector<GridWorld::Tile*> GridWorld::getNeighbours(Tile*& tile){
	std::vector<GridWorld::Tile*> neighbours;
	for (int dy = -1; dy <= 1; dy++){
		for (int dx = -1; dx <= 1; dx++){
			if(abs(dy) + abs(dx) != 0){
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

	for (Tile* neighbour : getNeighbours(tile)){
		double cost = calculateC(tile, neighbour);
		double g = neighbour->g;
			
		if(cost == INFINITY || g == INFINITY){
			continue;
		}
			
		if(cost + g < minCost){   //potential overflow?
			minTile = neighbour;
			minCost = cost + g;
		}
	}
	
	return TilePair(minTile, minCost);
}

double GridWorld::calculateH(GridWorld::Tile*& tile){
	unsigned int dx = labs(tile->x - start->x);
	unsigned int dy = labs(tile->y - start->y);

	if(dx > dy){
		std::swap(dx,dy);
	}

	return ((SQRT2-1) * dx + dy)*10;
}

double GridWorld::calculateC(GridWorld::Tile*& tileA, GridWorld::Tile*& tileB){
	if(tileA->cost == INFINITY || tileB->cost == INFINITY){
		return INFINITY;
	}

	if(labs(tileA->x - tileB->x) + labs(tileA->y - tileB->y) == 2){
		//These two tiles are diagonally adjacent to each other
		return SQRT2 * (tileA->cost + tileB->cost) / 2;
	}
	return (tileA->cost + tileB->cost) / 2;
}

GridWorld::KeyPair GridWorld::calculateKey(GridWorld::Tile*& tile){
	double key2 = std::min(tile->g, tile->rhs);
	double key1 = key2 + calculateH(tile) + km;
	//H-value should be re-calculated every call as it can change during incremental search

	return KeyPair(key1, key2);
}

GridWorld::Tile::Tile(unsigned int x, unsigned int y, double cost) : x(x), y(y){
	this->cost = cost;

	this->rhs = INFINITY;
	this->g = INFINITY;
	this->h = 0;

	this->isOpen = false;
}

GridWorld::Tile::Tile(Tile& other) : x(other.x), y(other.y){
	this->cost = other.cost;

	this->rhs = other.rhs;
	this->g = other.g;
	this->h = other.h;

	this->isOpen = other.isOpen;
}

void GridWorld::Tile::info() const{

	printf("[(%d, %d)  H: %.2lf, G: %.2lf, RHS: %.2lf, K:<%.2lf, %.2lf>]", this->x, this->y,
		this->h == INFINITY ? -1:this->h, 
		this->g == INFINITY ? -1:this->g,
		this->rhs == INFINITY ? -1:this->rhs, 
		this->key.first == INFINITY ? -1:this->key.first,
		this->key.second == INFINITY ? -1:this->key.second);
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
			double cost = getTileAt(x, y)->cost;
			if (cost == INFINITY){
				printf("-1 ");
			}else if (cost == INFLATION){
				printf("^^ ");
			}else{
				printf("%2.0lf ", cost);
			}
				
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