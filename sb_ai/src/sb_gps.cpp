//Author: Vincent Yuan & Nick Wu
//Date: May 19, 2015
//Purpose: GPS Data processing 
//Input: Compass Data (sb_driver node), GPS Data (sb_gps node) 
//Output: long, lat, distance(relative to waypoint), theta (relative to waypoint)
//Things needed to be implemented: compass callback function, struct 
//Need to be within 1m radius 

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sb_msgs/Waypoint.h"
#include "sb_msgs/Gps_info.h"
#include <string>
#include <iostream>
#include "Controllers/sb_ai.h"
#include <geometry_msgs/Twist.h>
#include <math.h>
#include "sb_msgs/compass.h"


using namespace std; 
using namespace sb_msgs;

#define PI 3.14159265


double angleCompass;
bool moveStatus;
bool goal;
bool msg_flag = false;
double d = 0; //distance in metres from currentWaypoint to targetWaypoint
double theta = 0; //theta is the angle the robot needs to turn from currentWaypoint to targetWaypoint
sb_msgs::Waypoint CurrentWaypoint,LastWaypoint,off;
geometry_msgs::Twist twist_msg;
sb_msgs::Gps_info pub_data; //Angle and distance are being published
int next_move, prev_move = 0;
sb_msgs::Waypoint TargetWaypoint;
static double const EART_RADIUS = 6378.137;

void startGps(void){
  double lon = 49.262368;
  double lat = -123.248591;
  off.lon = abs(lon - 49.157463);
  off.lat = abs(lat - (-123.149175022));
  return;
}

void gpsSubHandle(const std_msgs::String::ConstPtr& msg){
	char a[100];
	char nmea_type[7];
	char temp[10];
	int i = 0;
	char lon_dir, lat_dir;
	string str = msg->data;
	while (str[i] != '\0') { a[i] = str[i]; i++; }
	for (i = 0; i < 6; i++)
		nmea_type[i] = a[i];
	string msg_type(nmea_type);
	if (msg_type[4] == 'G' && msg_type[5] == 'A'){
		msg_flag = true;
		for (i = 18; i < 30; i++) 
			temp [i-18] = a[i];
		lon_dir = a[31];
		//cout << lon_dir << endl;
		if (lon_dir == 'N')
			CurrentWaypoint.lon = atof(temp)/100 + off.lon;
		else if (lon_dir == 'S')
			CurrentWaypoint.lon = (-1)*(atof(temp)/100 + off.lon);
		for (i = 33; i < 45; i++)
			temp [i-33] = a[i];
		lat_dir = a[47];
		//cout << lat_dir << endl;
		if (lat_dir == 'E')
			CurrentWaypoint.lat = atof(temp)/100 + off.lat;
		else if (lat_dir == 'W')
			CurrentWaypoint.lat = (-1)*(atof(temp)/100 + off.lat);
	}
	else{
		msg_flag = false;
		cout << "Did not receive proper gps data" << endl;
		}
		
  return;
 }
bool checkGoal (Waypoint CurrentWaypoint, Waypoint TargetWaypoint){
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

int NextMoveLogic (double distance, double angle){
	int next_move; 
	if (checkGoal) 
		return 0;
	else{
		if (angle < 0.0){
			return 2; 
			cout << "turn right" << endl;}
		else if (angle > 0.0){
			return -2;
			cout << "turn left" << endl;}
		else{
			if (distance > 100){
				return 50;
				cout << "forward quickly" << endl;}
			else if (distance < 100){
				return 20; 
				cout << "forward slowly" << endl;}
			else 
				cout << "Error in NextMoveLogic: " << distance << endl;
			}	
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

	if (next_move == 0){
	return twist;
	}
	else if (next_move < 10){
	twist.angular.z = (double)next_move/10;
	return twist;
	}
	else if (next_move > 10){
	twist.linear.y = (double)next_move/100;
	return twist;
	}
	
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

double createAngle(void){
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

sb_msgs::Gps_info createdata(void){

  sb_msgs::Gps_info data; 
  data.distance = createDistance();
  data.angle = createAngle();
	
  return data;  
}

void compassSubHandle (sb_msgs::compass compass){

	angleCompass = compass.compass;
	return;

}

int main (int argc, char **argv){

  ros::init(argc, argv, AI_NODE_NAME); //initialize access point to communicate
  ros::NodeHandle nh; //create handle to this process node, NodeHandle is main access point to communication with ROS system. First one intializes node
  ros::Subscriber gps_Sub = nh.subscribe("GPS_USB_DATA", 20, gpsSubHandle);
  ros::Publisher car_pub = nh.advertise<geometry_msgs::Twist>(PUB_TOPIC, 100);
  ros::Publisher gps_pub = nh.advertise<sb_msgs::Gps_info>("GPS_DATA",50);
  ros::Publisher coord_pub = nh.advertise<sb_msgs::Waypoint>("GPS_COORD", 100);
  ros::Subscriber compass_Sub = nh.subscribe ("COMPASS_DATA", 1, compassSubHandle);

  ros::Rate loop_rate(5); //10hz loop rate
	cout.precision(13);
/*  while (!msg_flag){
		cout << "Waiting for GPS Fix" << endl;	
		loop_rate.sleep();
	}*/

	TargetWaypoint.lon = 49.261928;
	TargetWaypoint.lat = -123.2487812;
  	startGps();
  while (ros::ok()){
	
	 if (msg_flag){
	    	  ROS_INFO ("Position:");
		  cout << "Current longitude: " << CurrentWaypoint.lon << " Current latitude: " << CurrentWaypoint.lat << endl;
		  cout << "distance" << pub_data.distance << endl;
		  cout << "angle" << pub_data.angle << endl;
	}
	  else{
		ROS_INFO("No Fix:"); 
  	}
	   //twist out 
	  pub_data = createdata();
	  gps_pub.publish(pub_data); //a out 
	  cout << "angle" << pub_data.angle << endl;
	  coord_pub.publish(CurrentWaypoint);

	  next_move = NextMoveLogic(pub_data.distance,pub_data.angle);
	  twist_msg = GetTwistMsg(next_move);
	  car_pub.publish(twist_msg);

	  ros::spinOnce(); //ros spin is to ensure that ros threading does not leave suscribes un processed
	  loop_rate.sleep();
}
    return 0;
}

