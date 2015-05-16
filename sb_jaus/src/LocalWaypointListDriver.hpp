#pragma once

#include <jaus/mobility/drivers/LocalWaypointListDriver.h>
#include "LocalWaypointDriver.hpp"

class LocalWaypointListDriver: public JAUS::LocaLWaypointListDriver{
	private:
	LocalWaypointDriver* driver;
	public:
	LocalWaypointListDriver(LocalWaypointDriver* driver): driver(driver) {}
	void ExecuteList(const double speedMetersPerSecond);
};

