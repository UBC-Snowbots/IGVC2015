#include <iostream>
#include "PriorityQueue.h"

using namespace std;

MinHeap::MinHeap()
{
  size = 0;
  // assign empty values in frontier?
}


int MinHeap::GetSize()
{
  return size;
}

void MinHeap::IncreaseSize()
{
  size += 1;
  return;
}

void MinHeap::Remove(int location)
{
  return;
}


void MinHeap::Insert(Key key, int location)
{
  int index = GetSize();
  locations[index] = location;
  keys[index] = key;
  return;
}


void MinHeap::Update(int location, Key key)
{
  return;
}


int MinHeap::Pop()
{
  return locations[0];
}


// gets the min location
int MinHeap::Top()
{
  return 0;
}
	  

// Gets the min key
Key MinHeap::TopKey()
{
  return keys[0];
} 


int main() 
{
  Key test_key;
  test_key.value = 0;
  test_key.cost = 10;
  MinHeap test;
  test.Insert(test_key, 9);
  test_key = test.TopKey();
  cout << "Key value: " << test_key.value << endl;
  cout << "Key cost: " << test_key.cost << endl; 
  cout << "Location: " << test.Pop() << endl;
  return 0;
}
