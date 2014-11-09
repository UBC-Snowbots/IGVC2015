#pragma once

struct key  {
  int value;
  int cost;
};

class MinHeap
{

private:
// keeping track of locations and keys in different arrays for searching 
  int location[100];
  struct key keys[100];	// array to store the frontier; statically allocated?	
  void Sort();	// helper fnc

public:
  void Remove(int location);
  void Insert(struct key);
  void Update(int location, struct key key);
  void Pop(int location);
  int Top();	  // gets the min location
  struct key TopKey(); // Gets the min key
}
