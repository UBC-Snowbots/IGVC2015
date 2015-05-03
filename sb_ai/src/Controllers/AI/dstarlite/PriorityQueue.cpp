#include <iostream>
#include "PriorityQueue.h"
#include "Key.h"

using namespace std;



// Constructor
MinHeap::MinHeap()
{
  size = 0;
  // assign empty values in frontier?
}





/** ==================== PRIVATE ==================== */

// Sorts the priority of the heap given the index of the displacement
void MinHeap::SortPriority(int loc)
{
	if (size == 1) {
		return;
	}
	
	else {
		
		int parent;
		parent = (loc - 1) / 2;
		
		while (parent >= 0) {	// check that it's not out of range

			if (keys[parent].cost > keys[loc].cost) {
				Swap(parent, loc); // if it doesn't swap, break the loop
			}
			else { break; }

			parent = (parent - 1) / 2;
		}
		
		return;
	}
}


// Swaps an index with another index within the priority heap
void MinHeap::Swap(int one, int two)
{
	int temp_loc;
	Key temp_key;
	
	temp_loc = locations[one];
	locations[one] = locations[two];
	locations[two] = temp_loc;
	
	temp_key = keys[one];
	keys[one] = keys[two];
	keys[two] = temp_key;
	
	return;
}
 
 
// Basically it's bfs
int MinHeap::Find(int location)
{
	for (int i = 0; i < size; i++) {
		if (locations[i] == location) { return i; }
	}
	// if not found, return -1
  return -1;
}


// Helper for Remove() and Pop() to delete location from min heap and sort
void MinHeap::DeleteFromMinHeap(int current_pos)
{
	int smallest, other;
	size--;
	locations[current_pos] = locations[size];
	keys[current_pos] = keys[size];
	
	locations[size] = -1;		// invalidate position in heap
	keys[size].value = 0;		// change to inf
	keys[size].cost = 0;		// change to inf
	
	smallest = (2*current_pos)+1;
	other = smallest+1;
	
	// only loop if left child is valid
	while (smallest < size) {
	
		//smallest_val = frontier[smallest];
		
		// check if right child is invalid
		if (other >= size) {
			if (keys[smallest].cost < keys[current_pos].cost) {
				Swap(current_pos, smallest);
				current_pos = smallest;
			}
			else { break; }
		}
		
		// otherwise compare both children and switch with smallest one
		else {

			//other_val = frontier[other];
		
			if (keys[smallest].cost > keys[other].cost) {
				smallest = other;
				other = other - 1;
			}
			
			if (keys[smallest].cost < keys[current_pos].cost) {
				Swap(current_pos, smallest);
				current_pos = smallest;
			}
		
			else {
				break;
			}	
		}	

		smallest = (2*current_pos)+1;
		other = smallest+1;
	}
	return;
}





/** =============== PUBLIC =================== */

// Removes a location from anywhere in the min heap (if it is in there)
void MinHeap::Remove(int location)
{
	int current_pos = Find(location);

	if (current_pos == -1) { return; }

 	DeleteFromMinHeap(current_pos);

  return;
}


// Inserts a location and its key into the min heap
void MinHeap::Insert(Key key, int location)
{
  int index = size;
  locations[index] = location;
  keys[index] = key;
  SortPriority(index);
  size++;
  return;
}


// Updates the location in the min heap with a new key
void MinHeap::Update(int location, Key key)
{
  int index = Find(location);
  if (index != -1) {
		locations[index] = location;
		keys[index] = key;
	}
  return;
}


// need to change to take parameters for storing TODO
int MinHeap::Pop()
{
	if (size == 0) { return -1; }
	int ret_val = Top();
	DeleteFromMinHeap(0);	// 0 is the min location on the min heap
	return ret_val;
}


// gets the min location
int MinHeap::Top()
{
  return locations[0];
}
	  

// Gets the min key
Key MinHeap::TopKey()
{
  return keys[0];
} 

