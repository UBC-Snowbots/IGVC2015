#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

namespace AI_Utilities
{
  struct Location
  {
	  int x;
	  int y;
  };

  static const float MAX_ANG_VEL = 0.3f;
  static const float MAX_LIN_VEL = 0.3f;
  geometry_msgs::Twist GetVelocity(Location* next_targets, float elsa_yaw, geometry_msgs::Twist &elsa_command);

}


