#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "AI/Utilities/Utilities.h"
#include "sb_msgs/RobotState.h"
#include "sb_msgs/Waypoint.h"

// Constant variables
static const std::string AI_NODE_NAME = "ai";
static const std::string PUB_TOPIC = "lidar_nav";
static const std::string MAP_SUB_TOPIC = "local_map";
static const std::string GPS_SUB_TOPIC = "GPS_COORD";
static const std::string COMPASS_SUB_TOPIC = "robot_state";

// Variable hacks for pathfinding map
// All are relative to global map position, not gps position
static const float START_ANGLE = 90.0;
static const float START_POS_X = 0.5; // From left side of map
static const float START_POS_Y = 0.8; // From top of map
static const float MAP_PADDING = 20; // Padding on all sides (m)
static const float MAP_WIDTH = 200; // meters
static const float MAP_HEIGHT = 100; // meters
static const float RESOLUTION = 0.05f; // 5cm^2 per grid cell

// Initialize variable hacks
int gps_count = 0;
int compass_count = 0;
int global_width = (MAP_WIDTH + MAP_PADDING * 2) / RESOLUTION;
int global_height = (MAP_HEIGHT + MAP_PADDING * 2) / RESOLUTION;
int global_size = global_width * global_height;
int origin_x = (int) (global_width * START_POS_X);
int origin_y = (int) (global_height * START_POS_Y);
float origin_lat = 42.677969;
float origin_long = 83.195365;

// Current state variables
float long_pos = origin_long; 
float lat_pos = origin_lat; 
float orientation = 0.0f;  
int x_pos = 0;
int y_pos = 0;
  
float global_orientation = 0; // assigned after we get gps reading
nav_msgs::OccupancyGrid global_map;


void MapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map)
{
  int global_x, global_y, local_x, local_y;
  int localMapSize = map->info.width * map->info.height; 
  int global_i, test;
  // Loop through the vision map
  for (int index = 0; index < localMapSize; index++) 
  {
    local_x = AI_Utilities::ConvertIndexToLocalXCoord(index, map->info.width) - ((map->info.width / 2) - 1);

    local_y = (map->info.height - 1) - AI_Utilities::ConvertIndexToLocalYCoord(index, map->info.width);
    
	  global_x = cos(orientation) * local_x
			        - sin(orientation) * local_y
			        + AI_Utilities::GetGlobalIndexX(origin_long, long_pos, origin_x, RESOLUTION);
			  
	  global_y = sin(orientation) * local_x
			        + cos(orientation) * local_y
			        + AI_Utilities::GetGlobalIndexY(origin_lat, lat_pos, origin_y, RESOLUTION);

	  // Update global map with 0/1 to show that an obstacle dne/exists
	  global_i = AI_Utilities::ConvertXYCoordToIndex(global_x, global_y, global_width);
	  global_map.data[global_i] =
			  map->data[index];
			  
		/*
		if (index == 0 || index == 239 || index == 28560 || index == 28799 || index == 28679)
		{ 
		  std::cout << "Index: " << index << std::endl;
		  std::cout << "Global index: " << AI_Utilities::ConvertXYCoordToIndex(global_x, global_y, global_width) << std::endl;
		  std::cout << "Global x: " << global_x << ", Global y: " << global_y << std::endl;
		  std::cout << "Local x: " << local_x << ", Local y: " << local_y << std::endl;
		}
    */
  }
}


// 42.677969 lat
// 83.195365 long
void GpsCallback(const sb_msgs::Waypoint::ConstPtr& gps)
{
  //lat_pos -= 0.0001;
  //long_pos += 0.0001;
  //std::cout << "Lat: " << lat_pos << ", Long: " << long_pos << std::endl;
  return;

  if (!gps) { ROS_INFO("Null gps data."); return; } // null check
  
  long_pos = gps->lon;
  lat_pos = gps->lat;
  
  if (gps_count == 0)
  {
    origin_long = gps->lon;
    origin_lat = gps->lat;
    gps_count++; 
  }
  
  else if (gps_count < 10) 
  {
    origin_long += gps->lon;
    origin_long /= 2;
    origin_lat += gps->lat;
    origin_lat /= 2;
    gps_count++; 
  }
  
  else 
  {
    long_pos = gps->lon;
    lat_pos = gps->lat;
  }
}


void CompassCallback(const sb_msgs::RobotState::ConstPtr& compass)
{
  orientation = 180;
  orientation *= AI_Utilities::PI / 180;
  return;

  if (!compass) { ROS_INFO("Null compass data."); return; } // null check
  
  if (compass_count == 0)
  {
    global_orientation = compass->compass;
  }
  
  else if (compass_count < 10)
  {
    global_orientation += compass->compass;
    global_orientation /= 2;
  }
  
  else 
  {
    orientation = compass->compass;
    orientation -= global_orientation;
    
    if (orientation > 180)
    {
      orientation -= 360;
    }
    
    if (orientation < -180)
    {
      orientation += 360;
    }
    
    orientation = 75;
    orientation *= AI_Utilities::PI / 180;
  }
}


// Execution 
int main(int argc, char **argv)
{
  ros::init(argc, argv, "global_mapping_node");
  ros::NodeHandle nh;
  ros::Rate loop_rate(10);
  
  // Initialize subscribers
	ros::Subscriber map_sub = nh.subscribe(MAP_SUB_TOPIC, 10, MapCallback);
	ros::Subscriber gps_sub = nh.subscribe(GPS_SUB_TOPIC, 10, GpsCallback);
	ros::Subscriber compass_sub = nh.subscribe(COMPASS_SUB_TOPIC, 10, CompassCallback);
  ros::Publisher global_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("global_map_test", 2);

	// Initialize test global map pub
	global_map.data.assign(global_size, 0);
	global_map.info.origin.position.x = 0;
	global_map.info.origin.position.y = 0;
	global_map.info.origin.position.z = 0;
	global_map.info.origin.orientation.x = 0;
  global_map.info.origin.orientation.y = 0;
  global_map.info.origin.orientation.z = 0;
  global_map.info.origin.orientation.w = 0;
  global_map.info.width = global_width;
  global_map.info.height = global_height;
  global_map.info.resolution = RESOLUTION;
  
  while (ros::ok())
  {
    global_map_pub.publish(global_map);
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  return 0;
}
