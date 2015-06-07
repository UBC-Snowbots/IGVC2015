#include "image_to_grid.hpp"

	ConvertSensorImage::ConvertSensorImage(std_msgs::Header imgheader,uint32_t imgheight, uint32_t imgwidth, string imgencoding,uint8_t img_bigendian,uint32_t imgstep, std::vector<uint8_t> imgdata, double imgresolution)
	{

		header = imgheader;
		height = imgheight;
		width = imgwidth;
		encoding = imgencoding;
		is_bigendian = img_bigendian;
		step = imgstep;
//		data(imgdata.begin(),imgdata.end());
		for (std::vector<uint8_t>::iterator it=imgdata.begin();it<imgdata.end();it++)
		{
			data.push_back(*it);
		}
	//	copy(imgdata.begin(),imgdata.end(),data.begin());
		nav_msgs::OccupancyGrid grid;
		resolution = imgresolution;

//		cout << height << endl;
//		cout << header.stamp << endl;

	}

//	std::vector<uint8_t> ConvertSensorImage::resize(double new_resolution)
//	{
//		
//	}

	nav_msgs::OccupancyGrid ConvertSensorImage::convert()
	{
		grid.header=header;
		grid.info.height=height;
		grid.info.width=width;
		grid.data=data;
		return grid;
	}


void ConvertSensorImage::printMap(){
	ROS_INFO("Printing grid, which is %d x %d", height, width);
	for(int i = 0; i < height; i++){
		for(int j = 0; j < width; j++){
<<<<<<< HEAD
			cout << static_cast<int>(data[i * width + j]) << " ";
=======
			//cout << ((int)(data[j * width + i]) == 0) ? '0':'1' << " ";
			printf("%d ", ((int)(data[j * width + i]) == 0)? 0:1);
			
>>>>>>> b7ad7c2fc501ccdacd05546dd0e29370416527ee
		}
		cout << endl;
	}
	cout << endl;
}
