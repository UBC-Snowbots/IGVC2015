#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#define MAX_ANG_VEL 0.3f
#define MAX_LIN_VEL 0.3f

struct Points
{
  float x;
  float y;
};

geometry_msgs::Twist GetVelocity(Points* next_targets, float elsa_yaw, geometry_msgs::Twist &elsa_command);




