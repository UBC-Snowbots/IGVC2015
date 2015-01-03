#pragma once

struct Key  {
  int value;
  int cost;
  
  Key(): value(0), cost(0) {}
};

Key* CalculateKey(int location);
void SetValue(Key* k, int val);
void SetCost(Key* k, int cost);
