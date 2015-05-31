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
sb_msgs::Waypoint CurrentWaypoint,LastWaypoint,off, buffWaypoint, avgWaypoint;
geometry_msgs::Twist twist_msg;
sb_msgs::Gps_info pub_data; //Angle and distance are being published
int next_move, prev_move = 0;
sb_msgs::Waypoint TargetWaypoint;
static double const EARTH_RADIUS = 6378.137;
long double buffer [10];

void startGps(void /*double lon, double, lat */){
  //double lon = 49.262368;
  //double lat = -123.248591;
  off.lon = (49.157435497 - 49.26231834);
  off.lat = (-123.149139703 - (-123.24871396));
  return;
}

void gpsSubHandle(const std_msgs::String::ConstPtr& msg){
	char a[100];
	char nmea_type[7];
	char temp[10];
	int i = 0;
	char lon_dir, lat_dir;
	string str = msg->data;
	while (str[i] != '\0') { a[i] = str[i]; i++; } //move everything into char array for access 
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
			CurrentWaypoint.lon = atof(temp)/100 - off.lon;
		else if (lon_dir == 'S')
			CurrentWaypoint.lon = (-1)*(atof(temp)/100) - off.lon;
		for (i = 33; i < 45; i++)
			temp [i-33] = a[i];
		lat_dir = a[47];
		//cout << lat_dir << endl;
		if (lat_dir == 'E')
			CurrentWaypoint.lat = atof(temp)/100 - off.lat;
		else if (lat_dir == 'W')
			CurrentWaypoint.lat = (-1)*(atof(temp)/100) - off.lat;
	}
	else{
		msg_flag = false;
		cout << "Did not receive proper gps data" << endl;
		}
		
  return;
 }
bool checkGoal (void){
cout<<fabs(CurrentWaypoint.lon - TargetWaypoint.lon)*100000 <<endl;
  if ((int)fabs(CurrentWaypoint.lon - TargetWaypoint.lon)*100000 < 2 && (int)fabs(CurrentWaypoint.lon - TargetWaypoint.lon)*100000 > 0){	
	cout<<fabs(CurrentWaypoint.lat - TargetWaypoint.lat)*100000 <<endl;
    if ((int)fabs(CurrentWaypoint.lat - TargetWaypoint.lat)*100000 < 2 && (int)fabs(CurrentWaypoint.lon - TargetWaypoint.lon)*100000 > 0){
	
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
	if (checkGoal()) {
	cout << "Arrived at waypoint" << endl;
		return 0;}
	else{
		if (angle < 0.0){
			cout << "turn right" << endl;
			return 2; 
			}
		else if (angle > 0.0){
			cout << "turn left" << endl;	
			return -2;
			}
		else{
			if (distance > 100){
				cout << "forward quickly" << endl;
				return 50;
			}
			else if (distance < 100){
				cout << "forward slowly" << endl;
				return 20;
			}
			else 
				cout << "Error in NextMoveLogic: " << distance << endl;
		    }	
		}
}
geometry_msgs::Twist GetTwistMsg(int next_move){
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
	double a = sin((long double)((lat2 - lat1) / 2))*sin(((lat2 - lat1) / 2)) + cos(lat1) * cos(lat2) * sin((lon2 - lon1) / 2)*sin((lon2 - lon1) / 2);
	double c = 2 * atan2(sqrt(a), sqrt(1 - a));
	double d = EARTH_RADIUS * c;
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
  cout << "Current longitude: " << avgWaypoint.lon << " Current latitude: " << avgWaypoint.lat << endl;
  cout << "distance" << data.distance << endl;
  cout << "angle" << data.angle << endl;
  return data;  
}

void compassSubHandle (sb_msgs::compass compass){
	angleCompass = compass.compass;
}
//bool service
void setWaypoints (sb_msgs::Waypoint& wp,double lon, double lat){
	wp.lon = lon; 
	wp.lat = lat;
}
void print (int color, const std::strings& message){cout << \033[1; << color << "m" << message << "\033[0m" << endl;}
void print (int color, double value){cout << \033[1; << color << "m" << value << "\033[0m" << endl;}
void print (int color, const std::strings& message, double value){cout << \033[1; << color << "m" << message <<  value << "\033[0m" << endl;}
void print (const std::strings& message){cout << strings << endl;}
void print (const std::strings& message, double value){cout << message << value << endl;}

int main (int argc, char **argv){
	int avg_count = 0;
	bool calibrate = false;
  ros::init(argc, argv, AI_NODE_NAME); //initialize access point to communicate

  ros::NodeHandle nh; //create handle to this process node, NodeHandle is main access point to communication with ROS system. First one intializes node
  ros::Subscriber gps_Sub = nh.subscribe("GPS_USB_DATA", 20, gpsSubHandle);
  ros::Publisher car_pub = nh.advertise<geometry_msgs::Twist>(PUB_TOPIC, 100);
  ros::Publisher gps_pub = nh.advertise<sb_msgs::Gps_info>("GPS_DATA",50);
  ros::Publisher coord_pub = nh.advertise<sb_msgs::Waypoint>("GPS_COORD", 100);
  ros::Subscriber compass_Sub = nh.subscribe ("robot_state", 20, compassSubHandle);
  ros::ServiceClient client = nh.serviceClient<sb_srv::gps_service>("GPS_SERVICE");

  ros::Rate loop_rate(10); //10hz loop rate
  cout.precision(13);	
	setWaypoints (TargetWaypoint,49.26238123,-123.24871402);
	setWaypoints (BuffWaypoint,0.0,0.0);

  while (ros::ok()){
	
	if (client.call(srv)){
		cout << srv.response.d << endl;
		pub_data.distance = srv.response.d;
		print(32,"Distance Caluclated: ", pub_data.distance);
	}
	else{
		print(31, "Service Failed");
	}
	
	if (calibrate){	 
		if (msg_flag){
			if (avg_count = 10){
				setWaypoints(avgWaypoint, (buffWaypoint.lon/10.0), (buffWaypoint.lat/10.0));
				 avg_count = 0;
			}
			else{
				  setWaypoints(buffWaypoint, CurrentWaypoint.lon, CurrentWaypoint.lat);
				  avg_count ++;
			}
			 ROS_INFO ("Position:");
			  			  pub_data = createdata();
			  
				//Publish Data 
			
			  	  gps_pub.publish(pub_data); //a out 
				  coord_pub.publish(CurrentWaypoint);
			  	  next_move = NextMoveLogic(pub_data.distance,pub_data.angle);
			  	  twist_msg = GetTwistMsg(next_move);
			  	  car_pub.publish(twist_msg);
				//Published Data
		}
		  else 
			  ROS_INFO("No Fix:"); 
			}
	else {	

		  printcolor (31, "Calibrating");
			if (msg_flag)
				if (avg_count = 10){ 
				  avgWaypoint.lon = buffWaypoint.lon/10.0;
				  avgWaypoint.lat = buffWaypoint.lat/10.0;
				  calibrate = true;
				  avg_count = 0;
				  startGps();	
			 	  cout <<"\x1b[1;31m" << off.lon << " and " << off.lat<< "\x1b[0m" << endl;
				}			  
				else{
				  setWaypoints(buffWaypoint,CurrentWaypoint.lon,CurrenWaypoint.lat);
				  avg_count ++;
				}
			else{ 
			 ROS_INFO("No Fix:");
			printcolor (31,"Waiting for fix for calibration");				
			//cout <<"\x1b[1;31m Waiting for fix for calibration \x1b[0m" << endl;
		}
	
	   //twist out 
	  
	  ros::spinOnce(); //ros spin is to ensure that ros threading does not leave suscribes un processed
	  loop_rate.sleep(); 
}}

    return 0;
}

