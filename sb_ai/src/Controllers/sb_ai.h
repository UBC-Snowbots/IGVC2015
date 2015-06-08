#pragma once

#include <sstream>
#include <iostream>
#include <fstream>
#include <string>
#include <math.h>
#include <stdlib.h>

// Constant variables
static const std::string AI_NODE_NAME = "ai";
static const std::string PUB_TOPIC = "lidar_nav";
static const std::string MAP_SUB_TOPIC = "local_map";
static const std::string GPS_SUB_TOPIC = "GPS_COORD";
static const std::string COMPASS_SUB_TOPIC = "robot_state";

// Switching between AI modes
static const std::string MODE_PARAM = "mode";

// Variable hacks for pathfinding map
// All are relative to global map position, not gps position
static const float START_ANGLE = 0.0;
static const float START_POS_X = 0.5; // From left side of map
static const float START_POS_Y = 0.8; // From top of map
static const float MAP_PADDING = 20; // Padding on all sides (m)
static const float MAP_WIDTH = 200; // meters
static const float MAP_HEIGHT = 100; // meters
static const float RESOLUTION = 0.05f; // 5cm^2 per grid cell

// For D* Lite
static const int INTERSECTION_RADIUS = 2;
static const int SCAN_RADIUS = 4;

