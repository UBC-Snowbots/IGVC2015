#include "ros/ros.h"
#include "WaypointManager.h"

int main(int argc, char** argv){
	ros::init(argc, argv, "waypoint_manager");
	
	ros::NodeHandle self;
	sb_waypoint_manager::WaypointManager manager(self);
	
	ros::Rate loop_rate(10);
	
	while(ros::ok()){
		manager.updateGPS();
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	return 0;
}

