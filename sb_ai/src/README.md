All classes of algorithms are found in the AI folder.

TEST / SIMULATION FILES: 

map_sim.cpp: This is a node that simulates the retrieval of a map from the lidar. The output format is sb_msgs/AISimMap.msg.
** width is the width size of the map grid
** height is the height size of the map grid
** map_grid is a vector that stores the probability values of the map (and is essentially the map itself). It is size width*height.

gps_sim.cpp: This is a node that simulates the retrieval of our current gps position from the gps. The output format is sb_msgs/AISimGPS.msg.
** Position (x,y) = (longitude, latitude)

test_output.cpp: This is the test file that publishes to the car command topic to test the robot's reponse to a simulated AI instruction.

ALGORITHMS:

sb_ai.h: This is a header file of information that any of the sb_ai algorithm nodes would need, such as the name of the topics they are subscribing and publishing to.

sb_astar.cpp: Not implemented yet. This is the node that runs A* Search.

sb_dijkstra.cpp: Implemented, but still requires gps data. This node searches a map using Dijkstra's algorithm. 

sb_dstarlite.cpp: Not implemented yet. This is the node that runs the D* Lite Search.
