#include "Key.h"
#include <iostream>

using namespace std;

Key* CalculateKey(int location) {
	Key * key = new Key;
	key->value = 0;	// TODO
	key->cost = 0;	// TODO
	return key;
}


void SetValue(Key* k, int val)
{
	k->value = val;
	return;
}

void SetCost(Key* k, int cost)
{
	k->cost = cost;
	return;
}
