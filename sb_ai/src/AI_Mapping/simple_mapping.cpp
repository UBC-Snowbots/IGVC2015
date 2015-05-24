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
	
        sim_map.pose.x = 0;
        sim_map.pose.y = 0;
	sim_map.pose.theta = 0;
	int x = 1;
        while (ros::ok()) {
		//Simulated Values
		sim_map.map.data.clear();
		srand(time(NULL));
		sim_map.map.data.push_back(int8_t(rand()%2));
		sim_map.map.data.push_back(int8_t(rand()%2));
		sim_map.map.data.push_back(int8_t(rand()%2));
		sim_map.map.data.push_back(int8_t(rand()%2));

		if (x % 2 == 0)
		{
			sim_map.pose.x = 1;
			sim_map.pose.y = 1;
			sim_map.pose.theta = 1;
		}
		else
		{
			sim_map.pose.x = 0;
			sim_map.pose.y = 0;
			sim_map.pose.theta = 0;	
		}
		//End of Simulated

                ROS_INFO("\n[local_mapping_node]");
		cout << "sim_map.pose.x = " << sim_map.pose.x << endl;
		cout << "sim_map.pose.y = " << sim_map.pose.y << endl;
		cout << "sim_map.pose.theta = " << sim_map.pose.theta << endl;
//		ROS_INFO("pose.x = %d", sim_map.pose.x); //This line used to work
//		ROS_INFO("pose.y = %d", sim_map.pose.y);
//		ROS_INFO("pose.theta = %d", sim_map.pose.theta);
		for(int i = 0; i<4; i++)
			cout << "map[" << i << "] = " << int(sim_map.map.data[i]) << endl; //Should cast to int8_t but does not work
//			ROS_INFO("map[%d] = %d", i,sim_map.map[i]);
		x++;
                local_map_pub.publish(sim_map);
		ros::spinOnce();
                loop_rate.sleep();
        }
}

