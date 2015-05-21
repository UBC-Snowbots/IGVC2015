//Author: Vincent Yuan & Nick Wu
//Date: May 19, 2015
//Purpose: GPS Data processing 
//Input: Compass Data (sb_driver node), GPS Data (sb_gps node) 
//Output: long, lat, distance(relative to waypoint), theta (relative to waypoint)
//Things needed to be implemented: compass callback function, struct 

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <string>
#include <iostream>
#include "sb_ai.h"
#include <geometry_msgs/Twist.h>
#include <math.h>


using namespace std; 

#define EARTH_RADIUS 6378.137
#define PI 3.14159265

struct waypoint {
  double lon;
  double lat;
  waypoint(){
    lon=lat=0;
  }
  waypoint(double x,double y){
	lon = x; 
	lat = y;
	}
};
double *NMEA; //To hold received suscription message
double angleCompass;
bool moveStatus;
bool goal;
bool msg_flag = false;
double d = 0; //distance in metres from currentWaypoint to targetWaypoint
double theta = 0; //theta is the angle the robot needs to turn from currentWaypoint to targetWaypoint
waypoint CurrentWaypoint,TargetWaypoint,LastWaypoint;
geometry_msgs::Twist twist_msg;
int next_move, prev_move = 0;
waypoint off;

void startGps(void){
  double lon = 49.262511;
  double lat = -123.248619;
  off.lon = lon - CurrentWaypoint.lon;
  off.lat = lat - CurrentWaypoint.lat;
  return;
}

void gpsSubHandle(const std_msgs::String::ConstPtr& msg){
	char a[100];
	char nmea_type[7];
	char temp[10];
	int i = 0;
	char lon_dir, lat_dir;
	string str = msg->data;
	cout.precision(13);
	while (str[i] != '\0') { a[i] = str[i]; i++; }
	for (i = 0; i < 6; i++)
		nmea_type[i] = a[i];
	string msg_type(nmea_type);
	cout << msg_type << endl;
	if (msg_type[4] == 'G' && msg_type[5] == 'A'){
		msg_flag = true;
		for (i = 18; i < 30; i++) 
			temp [i-18] = a[i];
		lon_dir = a[31];
		//cout << lon_dir << endl;
		if (lon_dir == 'N')
			CurrentWaypoint.lon = atof(temp)/100 - off.lon;
		else if (lon_dir == 'S')
			CurrentWaypoint.lon = (-1)*(atof(temp)/100 - off.lon);
		for (i = 33; i < 45; i++)
			temp [i-33] = a[i];
		lat_dir = a[47];
		//cout << lat_dir << endl;
		if (lat_dir == 'E')
			CurrentWaypoint.lat = atof(temp)/100 - off.lat;
		else if (lat_dir == 'W')
			CurrentWaypoint.lat = (-1)*(atof(temp)/100 - off.lat);
	}
	else{
		msg_flag = false;
		cout << "Did not receive proper gps data" << endl;
		}
		
  return;
 }
bool checkGoal (waypoint CurrentWaypoint, waypoint TargetWaypoint){
  if (CurrentWaypoint.lon == TargetWaypoint.lon){
    if (CurrentWaypoint.lat == TargetWaypoint.lat){
      return true;
    }
    else
      return false;
  }
  else
    return false;
}

int NextMoveLogic (int prev_move, double distance, double angle){
	int next_move; 
	if (checkGoal) 
		return 0;
	else{
		if (angle < 0.0) 
			return 0.2; 
		else if (angle > 0.0)
			return -0.2;
		else{
			if (distance > 100)
					return 0.5;
			else if (distance < 100)
				return 0.2; 
			else 
				cout << "Error in NextMoveLogic: " << distance << endl;}	
}
}
geometry_msgs::Twist GetTwistMsg(int next_move) 
{
	geometry_msgs::Twist twist;

	twist.linear.x = 0;
	twist.linear.y = 0;
	twist.linear.z = 0;
		
	twist.angular.x = 0;
	twist.angular.y = 0;
	twist.angular.z = 0;

	if (next_move == -1 || next_move == 1) { twist.linear.x = next_move; }
	if (next_move == -2 || next_move == 2) { twist.linear.y = next_move; }
	cout << "Next move: " << next_move << endl;
	return twist;
}

double createDistance (void){
	/*
	Input Parameter: void
	Output: distance in meteres from currentWaypoint to targetWaypoint
	Purpose: calculates distance from target waypoints
	Link:http://stackoverflow.com/questions/19412462/getting-distance-between-two-points-based-on-latitude-longitude-python
	*/

	//requires CurrentWaypoint.lat, CurrentWaypoint.lon, TargetWaypoint.lat, TargetWaypoint.lon
	double toRad = PI / 180;

	double lat1 = CurrentWaypoint.lat * toRad;
	double lon1 = CurrentWaypoint.lon * toRad;
	double lat2 = TargetWaypoint.lat * toRad;
	double lon2 = TargetWaypoint.lon * toRad;
	//from http://www.movable-type.co.uk/scripts/latlong.html, Distance formula (using haversine)
	double a = sin((lat2 - lat1) / 2)*sin((lat2 - lat1) / 2) + cos(lat1) * cos(lat2) * sin((lon2 - lon1) / 2)*sin((lon2 - lon1) / 2);
	double c = 2 * atan2(sqrt(a), sqrt(1 - a));
	double d = 6371000 * c;
	return d;
}

double createAngle(){
	/*
	Input Parameter:
	1. direction of robot from North (0-359 degrees)
	??
	2. x cordinates of TargetWaypoint in metres
	3. y cordinates of TargetWaypoint in metres
	Output: the direction the robot needs to turn (-180 < theta < 180) 
	Purpose: calculates angle of robot to target waypoint ((-180) to 180 degrees)
	Author: Nick Wu
	*/
	double x = 1, y = 1; //x and y cordinates in metres, this needs to be calculated or passed in a paramaters

	double theta = 0, angleWaypoint = 180; //theta: angle robot needs to turn, angleWaypoint goal angle from North
	double r = sqrt(x*x + y*y); //r = distance from the robot to waypoint
	double angleGoal = 180/PI * (acos(abs(y) / r)); //reference angle with respect to y-axis

	if (x > 0 && y >= 0) //goal angle in quad 1
		angleWaypoint = angleGoal;
	else if (x >= 0 && y < 0) //goal angle in quad 4
		angleWaypoint = (180 - angleGoal);
	//checking special condition under quad 1 and 4
	if (angleCompass > angleWaypoint + 180)
		theta = 360 - angleCompass + angleWaypoint;

	if (x < 0 && y <= 0) //goal angle in quad 3
		angleWaypoint = angleGoal + 180;
	else if (x < 0 && y > 0) //goal angle in quad 2 
		angleWaypoint = 360 - angleGoal;
	//checking special condition under quad 3 and 2 
	if (angleCompass <= angleWaypoint - 180){
		if (x < 0 && y <= 0) //checks for quad 3 
			theta = angleGoal - angleCompass - 180;
		else if (x < 0 && y > 0) //checks for quad 2
			theta = -angleCompass - angleGoal;
	}

	//for any other condtions
	if (theta == 0){
		theta = angleWaypoint - angleCompass;
	}
	return theta;
}
int main (int argc, char **argv){
  double gps_distance,gps_angle = 0.0;
  ros::init(argc, argv, AI_NODE_NAME); //initialize access point to communicate
  ros::NodeHandle nh; //create handle to this process node, NodeHandle is main access point to communication with ROS system. First one intializes node
  ros::Subscriber gps_Sub = nh.subscribe("GPS_USB_DATA", 20, gpsSubHandle);
  ros::Publisher car_pub = nh.advertise<geometry_msgs::Twist>(PUB_TOPIC, 100);
  ros::Publisher gps_pub_d = nh.advertise<std_msgs::String>("GPS_DISTANCE",50);
  ros::Publisher gps_pub_a = nh.advertise<std_msgs::String>("GPS_ANGLE",50);

  ros::Rate loop_rate(5); //10hz loop rate
/*  while (!msg_flag){
		cout << "Waiting for GPS Fix" << endl;	
		loop_rate.sleep();
	}
  startGps();*/
  while (ros::ok()){

	  if (msg_flag){
	    	  ROS_INFO ("Position:");
		  cout << "Current longitude: " << CurrentWaypoint.lon << " Current latitude: " << CurrentWaypoint.lat << endl;
	}
	  else{
		ROS_INFO("No Fix:"); 
  	}
	   //twist out 
	  gps_distance = createDistance();
	  gps_pub_d.publish(gps_distance); //d out 
	  gps_angle = createAngle();
	  gps_pub_a.publish(gps_angle); //a out 

	  //next_move = NextMoveLogic(prev_move,gps_distance,gps_angle);
	  twist_msg = GetTwistMsg(next_move);
	  car_pub.publish(twist_msg);

	  ros::spinOnce(); //ros spin is to ensure that ros threading does not leave suscribes un processed
	  loop_rate.sleep();
}
    return 0;
}

