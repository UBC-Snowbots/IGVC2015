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
static const std::string GPS_SUB_TOPIC = "gps_state";
static const std::string COMPASS_SUB_TOPIC = "compass_state";

// Switching between AI modes
static const std::string MODE_PARAM = "mode";
