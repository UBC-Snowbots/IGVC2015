#pragma once

#include "ros/ros.h"
#include <geometry_msgs/Twist.h>

namespace basic
{

  class ControllerBase
  {

  public:

	  // Override this to do your set-up (registering handlers and such)
	  ControllerBase() {}
	
	  // Override this if you want to specify a specific rate at which you want update() called.
	  virtual int GetClockSpeed() { return 10; }
	  
	  // This method is called by the main loop
	  virtual geometry_msgs::Twist Update() = 0;

	  // This method is only callable with vision
	  virtual int getVisionStatus() { return 1; };

	  // 
	  virtual double getPriority() {return 1.0;};

	  virtual int lastWaypoint(void){return 0;};
	  
	  // Deconstructor
	  virtual ~ControllerBase(){}
  };

}

