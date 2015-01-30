#include <iostream>
#include "PriorityQueue.h"
#include "Key.h"

using namespace std;

int main() 
{
  Key test_key_1, test_key_2, test_key_3, test_key_4;
  
  test_key_1.value = 0;
  test_key_1.cost = 10;
  
  test_key_2.value = 4;
  test_key_2.cost = 89;
  
  test_key_3.value = 23;
  test_key_3.cost = 5;
  
  test_key_4.value = 22;
  test_key_4.cost = 9;
  
  MinHeap test_heap;
  
  test_heap.Insert(test_key_2, 49);
  test_heap.Insert(test_key_1, 9);
  test_heap.Insert(test_key_3, 42);

  // Pop()
  cout << test_heap.Pop() << endl;
  cout << test_heap.Pop() << endl;
  cout << test_heap.Pop() << endl;
  
  // Pop() returns null value (-1)
  cout << test_heap.Pop() << endl;
  cout << test_heap.Pop() << endl;
  cout << test_heap.Pop() << endl;

  test_heap.Insert(test_key_4, 7);
  test_heap.Insert(test_key_1, 9);
  test_heap.Insert(test_key_3, 42);
  test_heap.Insert(test_key_2, 39);
 
	test_heap.Remove(7);
	test_heap.Remove(30);
	
	test_key_3.cost = 3;
	
	test_heap.Update(42, test_key_3);
	
	cout << "Top key: " << test_heap.TopKey().cost << endl;
	
	cout << test_heap.Pop() << endl;
  cout << test_heap.Pop() << endl;
  cout << test_heap.Pop() << endl;
  cout << test_heap.Pop() << endl;
  
  return 0;
}
