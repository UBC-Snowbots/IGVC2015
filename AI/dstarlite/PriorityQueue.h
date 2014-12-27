#pragma once
#include "Key.h"

class MinHeap
{

private:

// keeping track of locations and keys in different arrays for searching 
  int locations[100];
  Key keys[100];	// array to store the frontier; statically allocated?	
  int size;
  
  void SortPriority(int loc);	// helper fnc
  void Swap(int one, int two);
  int Find(int location); // find index for particular location

public:

  MinHeap();	// constructor
  void IncreaseSize();
  void Remove(int location);
  void Insert(Key key, int location);
  void Update(int location, Key key);
  int Pop();
  int Top();	  // gets the min location
  Key TopKey(); // Gets the min key

};
