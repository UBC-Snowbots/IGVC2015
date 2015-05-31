//Author: Vincent Yuan & Nick Wu
//Date: May 19, 2015
//Purpose: GPS Data processing 
//Input: Compass Data (sb_driver node), GPS Data (sb_gps node) 
//Output: long, lat, distance(relative to waypoint), theta (relative to waypoint)
//Things needed to be implemented: compass callback function, struct 
//Need to be within 1m radius 

#include "GpsController.hpp"

using namespace std; 

namespace ai
{
  GpsController::GpsController(ros::NodeHandle& nh)
  {
    // Subscribers
    gps_Sub = nh.subscribe(GPS_SUB_TOPIC, 20, &GpsController::GpsCallback, this);
    state_Sub = nh.subscribe (COMPASS_SUB_TOPIC, 1, &GpsController::CompassCallback, this);
    // Publishers
    gps_pub = nh.advertise<sb_msgs::Gps_info>(GPS_PUB_TOPIC,50);
    coord_pub = nh.advertise<sb_msgs::Waypoint>(WAYPOINT_PUB_TOPIC, 100);
    
    client = nh.serviceClient<sb_gps::Gps_Service>(GPS_SERV_TOPIC);
  
    // Initialize variables
    msg_flag = false;
    d = 0;
    theta = 0;
    prev_move = 0;
    avg_count = 0;
    TargetWaypoint.lon = 49.26238123;
	  TargetWaypoint.lat = -123.24871402;
	  buffWaypoint.lon = 0.0;
	  buffWaypoint.lat = 0.0;	
		calibration.lon = 49.26231834;
		calibration.lat = -123.24871396; //The point we are calibrating on
	  
	  // Set precision of the numbers we print
	  cout.precision(13);
		GpsController::calibrate();
  }
  
  
  geometry_msgs::Twist GpsController::Update()
  {
    /*if (client.call(srv))
    {
		  cout << srv.response.d << endl;
		  pub_data.distance = srv.response.d;
		  cout << "\033[1;32m" << "Distance Calculated: " << pub_data.distance << "\033[0m" << endl;
	  }
	  else { cout << "\033[1;31m" << "Service Failed" << "\033[0m" << endl; }
	*/

		    GpsController::calcwaypoint();
			  ROS_INFO ("Position:");
			  cout << "Current longitude: " << avgWaypoint.lon << " Current latitude: " << avgWaypoint.lat << endl;
				
			  pub_data = Createdata();
			  cout << "distance" << pub_data.distance << endl;
			  cout << "angle" << pub_data.angle << endl;
				//Publish Data 
				
			  gps_pub.publish(pub_data); //a out 
				coord_pub.publish(CurrentWaypoint);
			  next_move = NextMoveLogic(pub_data.distance,pub_data.angle);
			  twist_msg = GetTwistMsg(next_move);
			  return twist_msg;
				//Published Data
		  

	  
	  
	//dont include sleep, loop, spin 
  }


  void GpsController::StartGps() /*double lon, double, lat */
  {
  //double lon = 49.262368;
  //double lat = -123.248591;
  offWaypoint.lon = (49.157435497 - 49.26231834);
  offWaypoint.lat = (-123.149139703 - (-123.24871396));
  }

  void GpsController::GpsCallback(const std_msgs::String::ConstPtr& msg)
  {
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
			  CurrentWaypoint.lon = atof(temp)/100 - offWaypoint.lon;
		  else if (lon_dir == 'S')
			  CurrentWaypoint.lon = (-1)*(atof(temp)/100) - offWaypoint.lon;
		  for (i = 33; i < 45; i++)
			  temp [i-33] = a[i];
		  lat_dir = a[47];
		  //cout << lat_dir << endl;
		  if (lat_dir == 'E')
			  CurrentWaypoint.lat = atof(temp)/100 - offWaypoint.lat;
		  else if (lat_dir == 'W')
			  CurrentWaypoint.lat = (-1)*(atof(temp)/100) - offWaypoint.lat;
	  }
	  else{
		  msg_flag = false;
		  cout << "Did not receive proper gps data" << endl;
		  }
		
    return;
   }
 
 
  bool GpsController::CheckGoal()
  {
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

  int GpsController::NextMoveLogic (double distance, double angle){
	  int next_move; 
	  if (CheckGoal()) {
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
				if (distance < 0){
					cout << "Distance was not calculated" << distance << endl;
					return 0;
				}
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

  geometry_msgs::Twist GpsController::GetTwistMsg(int next_move) 
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

  double GpsController::CreateDistance (){
		double distance; 
	 	srv.request.lon1 = TargetWaypoint.lon;
		srv.request.lat1 = TargetWaypoint.lat;
 	 	srv.request.lon2 = avgWaypoint.lon;
		srv.request.lat2 = avgWaypoint.lat;
		if (client.call(srv)){
		distance = srv.response.distance;
		print (34, "Distance: ", distance);
		}
		else {
		ROS_ERROR("SRV Failed");
		return -1;
		}
		return distance;
	}	

  double GpsController::CreateAngle(void){
		double dx = (TargetWaypoint.lon - CurrentWaypoint.lon)*111302.62;
		double dy = (TargetWaypoint.lat - CurrentWaypoint.lat)*110574.61;
		long double theta, GeoNorth;
		int quad; 
		//1.converting magnetic north to geo north 
		GeoNorth = angleCompass - MAG_DECL;
		if (GeoNorth < -179)
				GeoNorth = 180 + (GeoNorth + 179);
		//2.Move 0 to north of robot
		GeoNorth = GeoNorth - 90;
		if (GeoNorth < 180) 
				GeoNorth = 180 + (GeoNorth + 179);
		//3.Decide Qudrant using deltalon, deltalat
		if (dx > 0 && dy > 0) quad = 1;
		else if (dx > 0 && dy < 0) quad = 2; 
		else if (dx < 0 && dy < 0) quad = 3; 
		else quad = 4; 
		//4.Arctan dy/dx to find angle from "x-axis" (E-W)
		if (quad == 1 || quad == 4)
		theta = atan(dx/dy);
		else 
		theta = 90 + atan(dy/dx);
		//5.How much GeoNorth needs to rotate left or right to get to theta
		if (GeoNorth > theta) 
			theta = theta - GeoNorth; 
		else theta = GeoNorth - theta;

		return theta;
  }

  sb_msgs::Gps_info GpsController::Createdata()
  {

    sb_msgs::Gps_info data; 
  data.distance = ai::GpsController::CreateDistance();
  data.angle = ai::GpsController::CreateAngle();
  cout << "Current longitude: " << avgWaypoint.lon << " Current latitude: " << avgWaypoint.lat << endl;
  cout << "distance" << data.distance << endl;
  cout << "angle" << data.angle << endl;
  return data; 
  }

  void GpsController::CompassCallback(const sb_msgs::RobotState::ConstPtr& state){
	  angleCompass = state->compass;
  }

	void GpsController::print (int color, const std::string& message){cout << "\033[1;" << color << "m" << message << "\033[0m" << endl;}
	void GpsController::print (int color, double value){cout << "\033[1;" << color << "m" << value << "\033[0m" << endl;}
	void GpsController::print (int color, const std::string& message, double value){cout << "\033[1;" << color << "m" << message <<  value << "\033[0m" << endl;}
	void GpsController::print (const std::string& message){cout << message << endl;}
	void GpsController::print (const std::string& message, double value){cout << message << value << endl;}
	void GpsController::print (int color, const std::string& message, double value, double value2){cout << "\033[1;" << color << "m" << message << value << "<<" << value2 << "\033[0m" << endl;}	
	
  void GpsController::calibrate (void){
	 printf ("Calibrating. . . .");
	 int i = 0;
	while ( i < 10) {
		if (msg_flag){
					setWaypoints(buffWaypoint,(buffWaypoint.lon +CurrentWaypoint.lon),(buffWaypoint.lat+CurrentWaypoint.lat));
		i++;
			}
		else
			cout << "\b\b\b\b\b\b\b. . . .";
		}	
		setWaypoints (offWaypoint, buffWaypoint.lon/10.0, buffWaypoint.lat/10.0);
	print(35, "Calibration Points: ", offWaypoint.lon, offWaypoint.lat);
	setWaypoints (offWaypoint, (offWaypoint.lon - calibration.lon), (offWaypoint.lat - offWaypoint.lat));
	print(35, "Using: ", offWaypoint.lon, offWaypoint.lat);	
	return;
	}

	void GpsController::setWaypoints (sb_msgs::Waypoint& wp,double lon, double lat){
	wp.lon = lon; 
	wp.lat = lat;
}

	void GpsController::setWaypoints (sb_msgs::Waypoint& wp1, sb_msgs::Waypoint& wp2){
	wp1.lon = wp2.lon;
	wp1.lat = wp2.lat;
	}

void GpsController::calcwaypoint (void){
	 printf ("Calculating Next Waypoint. . . .");
	 int i = 0;
	while ( i < 10) {
		if (msg_flag){
					setWaypoints(buffWaypoint,(buffWaypoint.lon +CurrentWaypoint.lon),(buffWaypoint.lat+CurrentWaypoint.lat));
		i++;
			}
		else
			cout << "\b\b\b\b\b\b\b . . . .";
		}	
		setWaypoints (avgWaypoint, buffWaypoint.lon/10.0, avgWaypoint.lat/10.0);
	print(35, "NextPoint: ", avgWaypoint.lon, avgWaypoint.lat);
	return;

	
}

//end of namespace
}














