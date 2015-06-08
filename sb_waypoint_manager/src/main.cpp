#include <ros/ros.h>
#include <sb_msgs/MoveCommand.h>
#include <sb_msgs/Waypoint.h>
#include <sb_gps/Gps_Service.h>
#include <iostream>
#include <list>
#include <fstream>


std::list<sb_msgs::MoveCommand> read_file(const std::string& name){
	std::ifstream in(name.c_str());
	std::list<sb_msgs::MoveCommand> result;
	while(in.good()){
		sb_msgs::MoveCommand tmp;
		in >> tmp.lat >> tmp.lon;
		tmp.spd = 0.3;
		result.push_back(tmp);
	}
	return result;
}

sb_msgs::Waypoint position;
sb_msgs::MoveCommand current_command;
std::list<sb_msgs::MoveCommand> list;
ros::Publisher publisher;

bool at_waypoint(const sb_msgs::MoveCommand& pt){
	sb_gps::Gps_Service srv;
	srv.request.lat1 = position.lat;
	srv.request.lon1 = position.lon;
	srv.request.lat2 = pt.lat;
	srv.request.lon2 = pt.lon;
	if(ros::service::call("GPS_SERVICE",srv)){
		if(srv.response.distance < 11){
			return true;
		}
	}else{
		std::cout << "could not access gps service" << std::endl;
	}
	return false;
}

void onNewWaypoint(const sb_msgs::Waypoint& pos){
	position = pos;
	if(at_waypoint(current_command)){
		if(list.empty()){
			std::cout << "reached end of waypoints" << std::endl;
			current_command.spd = 0;
		}else{
			current_command = list.front();
			list.pop_front();
			publisher.publish(current_command);
		}
	}
}

int main(int argc, char** argv){

	ros::init(argc, argv, "waypoint_manager");

	ros::NodeHandle self;

	ros::Rate loop_rate(10);

	publisher = self.advertise<sb_msgs::MoveCommand>("move_command",100);

	self.subscribe("GPS_COORD",100,onNewWaypoint);

	list = read_file("input.txt");

	if(list.empty()){
		std::cout << "input empty, exit immediately" << std::endl;
		return 0;
	}
	std::cout <<  "fdfsdfasfdsa " << list.front() << std::endl;
	current_command = list.front();
	list.pop_front();
	

	
	while(ros::ok()){
	publisher.publish(current_command);
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	return 0;
}

