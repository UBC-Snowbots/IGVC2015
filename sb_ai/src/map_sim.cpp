#include "ros/ros.h"
#include "std_msgs/String.h"
#include <string>
#include <iostream>
#include "sb_ai.h"
#include "sb_msgs/AISimMap.h"

using namespace std;

static const string NODE_NAME = "test_map";
static const string MAP_PUB_TOPIC = "ai_map";

sb_msgs::AISimMap sim_map;
//int x;

vector<int8_t> RandomMap()
{
	vector<int8_t> map (100, 0);
	//x = map.at(10);
	return map;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, NODE_NAME);
	ros::NodeHandle n;
	ros::Publisher map_pub = n.advertise<sb_msgs::AISimMap>(MAP_PUB_TOPIC, 100);
	ros::Rate loop_rate(10);

	//sim_map.width = 10;
	//sim_map.height = 10;
	//sim_map.map_grid = RandomMap();
	
	while (ros::ok()) {
		map_pub.publish(sim_map);
		cout << "Running.." << endl;
		loop_rate.sleep();
	} 

	return 0;
}
