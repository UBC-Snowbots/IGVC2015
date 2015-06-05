#include "generate_global_map.hpp"
#include <algorithm> 
#include <iostream>

sb_msgs::Waypoint GPS_ORIGIN_GLOBAL_MSG;


uint32_t COURSE_WIDTH = 200;
uint32_t COURSE_HEIGHT = 100;
float MAP_RESOLUTION = 0.5;

// Make this global so we can unsubscribe after processing and storing the gpsOrigin to the global variable
ros::Subscriber gpsOriginSubscriber;

void gpsOriginSubscriberCallback(
		const sb_msgs::Waypoint::ConstPtr& gpsOriginMsg);

void GlobalMapSubscriberCallback(const const nav_msgs::OccupancyGrid::ConstPtr& globalMap);

int main(int argc, char **argv) {

	ros::init(argc, argv, "test_generate_global_map");

	ros::Time::init();

	ros::Rate loopRate(5); //10hz loop rate

	while (ros::ok()) {

		ros::NodeHandle n;

		// We subscribe to this to get the GPS origin, this will run once and then the callback will close the subscriber
		gpsOriginSubscriber = n.subscribe(WAYPOINT_TOPIC, 1000,
				&gpsOriginSubscriberCallback);

		GenerateGlobalMap generateGlobalMap(GPS_ORIGIN_GLOBAL_MSG, COURSE_WIDTH,
				COURSE_HEIGHT, MAP_RESOLUTION);

		//// Create Test Local Map
		nav_msgs::OccupancyGrid localMap;
		localMap.info.width = 4;
		localMap.info.height = 4;
		localMap.data.assign(16, 0);
		localMap.data[5] = 1;

		printf("Local Map:\n");
		for (int i = 0; i < localMap.data.size(); i++) {

			if ((i + 1) % localMap.info.width == 0) {

				printf("%d ", localMap.data[i]);
				printf("\n");

			} else {

				printf("%d ", localMap.data[i]);

			}

		}

		//// Create Test Waypoint Msg
		sb_msgs::Waypoint gpsOrigin;
		gpsOrigin.lon = 100;
		gpsOrigin.lat = 40;

		//// Create Test Gps_info Msg
		sb_msgs::Gps_info;
		Gps_info.angle = 70; //FIXME MIGHT NEED TO CONVERT THIS TO RADIAN NOT SURE WHAT GPS_INFO OUTPUTS

		// Testing the subscribers in the class
		ros::Publisher localMapPublisher = n.advertise < nav_msgs::OccupancyGrid
				> (VISION_TOPIC, 1000);
		ros::Publisher waypointPublisher = n.advertise < sb_msgs::Waypoint
				> (WAYPOINT_TOPIC, 1000);
		ros::Publisher gpsInfoPublisher = n.advertise < sb_msgs::Gps_info
				> (GPS_INFO_TOPIC, 1000);

		// No clue if this works
		generateGlobalMap.TransformLocalToGlobal();

		ros::Subscriber globalMap = n.subscribe(GLOBAL_MAP_TOPIC, 1000,
				&GlobalMapSubscriberCallback);

		ros::spin(); //ros spin is to ensure that ros threading does not leave subscribes un processed

		loopRate.sleep();

	}

	return 0;

}

void gpsOriginSubscriberCallback(
		const sb_msgs::Waypoint::ConstPtr& gpsOriginMsg) {

	GPS_ORIGIN_GLOBAL_MSG.lon = gpsOriginMsg->lon;
	GPS_ORIGIN_GLOBAL_MSG.lat = gpsOriginMsg->lat;
	gpsOriginSubscriber.shutdown();

}

void GlobalMapSubscriberCallback(const const nav_msgs::OccupancyGrid::ConstPtr& globalMap) {

	// TODO CHECK TO SEE IF THE GLOBAL COORD FOR THE OBSTACLE IN THE TEST LOCAL MAP EXISTS AND IS THE CORRECT COORDINATES

}
