#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <cmath>
#include <iostream>

#define MAX_ANG_VEL 0.3f
#define MAX_LIN_VEL 0.3f
#define NODE_NAME "path_smoothing"
#define ELSA_PUB_TOPIC "elsa_command"

struct Points
{
  float x;
  float y;
};

Points next_points[2];
float elsa_yaw = 0.0f;
float elsa_speed = 0.0f;
float target_yaw = 0.0f;
geometry_msgs::Twist elsa_command; 

void GetVelocity(float robot_yaw, int robot_x, int robot_y, Points* next_targets, float robot_speed)
{
  // Check for null 
  if (!next_targets) return; 
  
  float dist_x, dist_y, ang_dist_sign;
  float target_yaw, ang_dist;
  dist_x = next_targets[1].x - next_targets[0].x;
  dist_y = next_targets[1].y - next_targets[0].y;

  target_yaw = atan(dist_y/dist_x);
  ang_dist = target_yaw - robot_yaw; // need to fix negatives
  ang_dist_sign = (int) ang_dist;
  ang_dist_sign /= abs(ang_dist_sign);  // check that this actually works
  ang_dist = abs(ang_dist);

  if (ang_dist > 120.0f)
  {
    elsa_command.angular.z = 0.3 * ang_dist_sign;
    elsa_command.linear.y = 0.1;
  }

  else if (ang_dist > 80.0f)
  {
    elsa_command.angular.z = 0.25 * ang_dist_sign;
    elsa_command.linear.y = 0.15;
  }
  
   else if (ang_dist > 40.0f)
  {
    elsa_command.angular.z = 0.2 * ang_dist_sign;
    elsa_command.linear.y = 0.2;
  }
  
  else if (ang_dist > 10.0f)
  {
    elsa_command.angular.z = 0.15 * ang_dist_sign;
    elsa_command.linear.y = 0.25;
  }
  
  else
  {
    elsa_command.angular.z = 0;
    elsa_command.linear.y = 0.3;
  }
}
                   

int main (int argc, char** argv)
{
  elsa_command.linear.x = 0;
  elsa_command.linear.y = 0;
  elsa_command.linear.z = 0;
  elsa_command.angular.x = 0;
  elsa_command.angular.y = 0;
  elsa_command.angular.z = 0;

  Points first, second;
  first.x = 0;
  first.y = 0;
  second.x = 0;
  second.y = 0;
  next_points[0] = first;
  next_points[1] = second;

  ros::init(argc, argv, NODE_NAME);
  ros::NodeHandle nh;
  
  ros::Publisher elsa_pub = nh.advertise<geometry_msgs::Twist>(ELSA_PUB_TOPIC, 10);
  
  ros::Rate loop_rate(10);
  
  while (ros::ok())
  {
    GetVelocity(0, 0, 0, next_points, 0);
    elsa_pub.publish(elsa_command);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
