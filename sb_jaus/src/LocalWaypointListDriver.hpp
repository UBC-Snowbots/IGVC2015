#pragma once

#include <jaus/mobility/drivers/LocalWaypointListDriver.h>
#include "LocalWaypointDriver.hpp"

class LocalWaypointListDriver: public JAUS::LocalWaypointListDriver{
	private:
	JAUS::LocalWaypointDriver* driver;
	public:
	LocalWaypointListDriver(JAUS::LocalWaypointDriver* driver): driver(driver) {}
	void ExecuteList(const double speedMetersPerSecond);
};

