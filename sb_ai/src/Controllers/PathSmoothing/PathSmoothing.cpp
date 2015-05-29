#include <cmath>
#include <iostream>
#include "PathSmoothing.h"

geometry_msgs::Twist GetVelocity(Location* next_targets, float elsa_yaw)
{
  geometry_msgs::Twist elsa_command;
  elsa_command.linear.x = 0;
  elsa_command.linear.y = 0;
  elsa_command.linear.z = 0;
  elsa_command.angular.x = 0;
  elsa_command.angular.y = 0;
  elsa_command.angular.z = 0;
  
  // Check for null 
  if (!next_targets) return elsa_command; 
  
  float dist_x, dist_y, ang_dist_sign;
  float ang_dist;
  float target_yaw = 0.0f;
  
  dist_x = next_targets[1].x - next_targets[0].x;
  dist_y = next_targets[1].y - next_targets[0].y;

  target_yaw = atan(dist_y/dist_x);
  ang_dist = target_yaw - elsa_yaw; // need to fix negatives
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
  
  return elsa_command;
}
                  
