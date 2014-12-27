#include <iostream>
#include "PriorityQueue.h"
#include "Key.h"

using namespace std;

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
