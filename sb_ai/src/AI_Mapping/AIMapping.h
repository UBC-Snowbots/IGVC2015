//#pragma once
// this class contains the functions for mapping a local map to a global map

// This and its respective main file can be compiled using this:
// g++ AIMapping.h AIMapping.cpp -Wall -o a     // THIS COMPILES
// ./a                                          // THIS RUNS IT

struct Vector2
{
  int x;
  int y;
};

class AIMapping
{
  private:
    
    // These can be set using public functions. 
    // Or you can make these public variables and access them directly.
    int* global_map;
    int* local_map;
    struct Vector2 global_map_size;
    struct Vector2 local_map_size;
    
    
    /** 
    TODO : struct Vector2* FUNCTION(int index);
    Purpose: Translates array index to map coordinates 
    */
    
    /**
    TODO : int FUNCTION(struct Vector2* mapCoord);
    Purpose: Translates map coordinates to array index
    */
  
  
  public:
    // classes and functions exposed to other classes
    
    /** TODO: void FUNCTION(
    parameters depend on what you need... 
    > position offset (struct Vector2)
    > orientation (int / float)
    > robot's current position on global map (struct Vector2)
    > etc
    )
    Purpose: To copy data from the local map onto the global map
    */
    
};
