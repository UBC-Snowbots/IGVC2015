#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "nav_msgs/OccupancyGrid.h"

void gpsSubscriberCallback(const geometry_msgs::Pose2D::ConstPtr& pose2DMsg) {

  

}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "sb_mapping");

  ros::NodeHandle n;

  ros::Publisher pose2DPublisher = n.advertise<geometry_msgs::Pose2D>("sb_ai_pose2D", 1000);
  ros::Publisher occupancyGridPublisher = n.advertise<nav_msgs::OccupancyGrid("sb_ai_occupancy_grid", 1000);

  ros::Subscriber gpsSubscriber = n.subscribe("sb_gps", 1000, gpsSubscriberCallback);

  ros::Rate loop_rate(10);

  while (ros::ok())
  {

    geometry_msgs::Pose2D pose2DMsg;
    nav_msgs::OccupancyGrid occupancyGridMsg;

    // Temporarily setting random values
    pose2DMsg.x = 0;
    pose2DMsg.y = 0;
    pose2DMsg.theta = 0;
    pose2DPublisher.publish(pose2DMsg);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;

}

