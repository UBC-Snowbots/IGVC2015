/*
 * This is supposed to be used for generating the local map and publishing it.
 * The assumption is local map data (possibly from the LIDAR) is processed here.
 *
 */

#include "ros/ros.h"
#include <iostream>
#include "sb_msgs/LocalMap.h"
#include <stdlib.h>
#include <time.h>

using namespace std;

static const string NODE_NAME = "local_mapping_node";
static const string LOCAL_MAP_PUB_TOPIC = "local_map";

sb_msgs::LocalMap sim_map;

int main(int argc, char **argv)
{
        ros::init(argc, argv, NODE_NAME);
        ros::NodeHandle n;
        ros::Publisher local_map_pub = n.advertise<sb_msgs::LocalMap>(LOCAL_MAP_PUB_TOPIC, 100);

        ros::Rate loop_rate(1);
	

	//Simulated values for now. This is probably where we subscribe to the hokuyo messages
	
        sim_map.x = 0;
        sim_map.y = 0;
	sim_map.rotation = 0;
	int x = 1;
        while (ros::ok()) {
		//Simulated Values
		sim_map.map.clear();
		srand(time(NULL));
		sim_map.map.push_back(rand()%2);
		sim_map.map.push_back(rand()%2);
		sim_map.map.push_back(rand()%2);
		sim_map.map.push_back(rand()%2);

		if (x % 2 == 0)
		{
			sim_map.x = 1;
			sim_map.y = 1;
			sim_map.rotation = 1;
		}

		else
		{
			sim_map.x = 0;
			sim_map.y = 0;
			sim_map.rotation = 0;	
		}
		//End of Simulated

                cout << "[local_mapping_node]" << x << endl;
		ROS_INFO("x = %d", sim_map.x);
		ROS_INFO("y = %d", sim_map.y);
		ROS_INFO("rotation = %d", sim_map.rotation);
		for(int i = 0; i<4; i++)
			ROS_INFO("map[%d] = %d", i,sim_map.map[i]);
		x++;
                local_map_pub.publish(sim_map);
		ros::spinOnce();
                loop_rate.sleep();
        }
}

