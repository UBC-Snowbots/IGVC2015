#pragma once

#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"
#include "sensor_msgs/Image.h"
#include "nav_msgs/OccupancyGrid.h" 
#include <string>
#include <algorithm>

using namespace std;
//Input: (image data to subscribe to, publisher name of occupancy grid)
class ConvertSensorImage {

	private:
		std_msgs::Header header;
		uint32_t height;
		uint32_t width;
		
		string encoding;
		uint8_t is_bigendian;
		uint32_t step;
		double resolution;
		std::vector<int8_t> data;
		nav_msgs::OccupancyGrid grid;
		
		
	public:
		ConvertSensorImage(	std_msgs::Header imgheader,
					uint32_t imgheight, 
					uint32_t imgwidth, 
					string imgencoding,
					uint8_t img_bigendian,
					uint32_t imgstep,
					std::vector<uint8_t> imgdata,
					double imgresolution=480.0 //(cell/m)
					);
//		std::vector<uint8_t> resize(double new_resolution); //(cell/m)
		nav_msgs::OccupancyGrid convert();
		
		
		
};
