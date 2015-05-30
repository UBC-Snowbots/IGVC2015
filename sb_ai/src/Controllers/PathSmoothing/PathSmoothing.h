#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "../AI/structures.h"

#define MAX_ANG_VEL 0.3f
#define MAX_LIN_VEL 0.3f

geometry_msgs::Twist GetVelocity(Location* next_targets, float elsa_yaw, geometry_msgs::Twist &elsa_command);
