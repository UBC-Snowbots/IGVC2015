#include "generate_global_map.hpp"
#include <iostream>

int main(int argc, char **argv) {

  ros::init(argc, argv, "test_generate_global_map");

  ros::Time::init();

  ros::Rate loopRate(5); //10hz loop rate

  while (ros::ok()){

    GenerateGlobalMap generateGlobalMap;

    generateGlobalMap.testDoSomething();

    ros::NodeHandle n;

    nav_msgs::OccupancyGrid localMap;
    localMap.info.width = 4;
    localMap.info.height = 4;
    int tempArray[] = {0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0};
    localMap.data.assign(tempArray, tempArray + 16);
    ROS_INFO("YOLO");

    printf("Local Map:\n");
    for(int i = 0; i < localMap.data.size(); i++) {

      if((i+1) % localMap.info.width == 0) {

        printf("%d ", localMap.data[i]);
        printf("\n");

      } else {

        printf("%d ", localMap.data[i]);

      }

    }

    // Testing the subscribers in the class
    ros::Publisher localMapPublisher = n.advertise<nav_msgs::OccupancyGrid>(VISION_TOPIC, 1000);
    ros::Publisher waypointPublisher = n.advertise<sb_msgs::Waypoint>(WAYPOINT_TOPIC, 1000);
    ros::Publisher gpsInfoPublisher = n.advertise<sb_msgs::Gps_info>(GPS_INFO_TOPIC, 1000);
    



    ros::spin(); //ros spin is to ensure that ros threading does not leave subscribes un processed

    loopRate.sleep();

  }

  return 0;

}

