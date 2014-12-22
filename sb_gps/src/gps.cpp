#include <stio.h>
#include <stlib.h>
#include <math.h> 
#include <gps.h> 


// temp callback
void gpsCallback(const std_msgs::String::ConstPtr& msg) // changed to string
{
	//string a = "B,4.293845e+08,-5.394857e+09,3945.33";

	char a[64];
	char temp[8];
	int x, y, i = 0, j;
	string str = msg->data;
  
	while (str[i] != '\0') { a[i] = str[i]; i++; }
	if (a[0] != 'B' && a[1] != ',') { cout << "no B" << endl; return; }  
	if( a[10] != 'e' && a[24] != 'e') { cout << "no e" << endl; return; } 
	if( a[14] != ',' && a[28] != ',') { cout << "no comma" << endl; return; }
	if( a[33] != '.') { cout << "no compass" << endl; return; }

	for (i = 2, j=0; i<9 || j<7; i++, j++) { 
		if (a[i]=='.') { j--; }
		else { temp[j] = a[i]; }
	}
	last_waypoint.long_x = current_waypoint.long_x;
	current_waypoint.long_x = atoi(temp); 
	for (i=15, j=0; i<10 || j<8; i++, j++) {
		if (a[i] == '.') { j--; }
		else { temp[j] = a[i]; }
	}
	last_waypoint.lat_y = current_waypoint.lat_y;
	current_waypoint.lat_y = atoi(temp);

	//current_direction = atof(msg->data.substr(29).c_str());
	// up to 6 decimal precision ~10cm, error ~2m
	cout << current_waypoint.lat_y << endl;
	cout << current_waypoint.long_x << endl;
	//cout << current_direction << endl;
}


int main(int argc, char **argv) 
{

	// hard code current waypoint 
	current_waypoint.long_x = 0; // keep like this until we start receiving gps data
	current_waypoint.lat_y = 0;
	last_waypoint.long_x = 0;
	last_waypoint.lat_y = 0;
	//current_direction = 0;

	ros::init(argc, argv, GPS_NODE_NAME); // start node
	ros::NodeHandle nh; // main access point to communication with ROS system. First one initializes node.

	ros::Publisher gps_test_pub = nh.advertise<geometry_msgs::Twist>(GPS_TEST_TOPIC, 20); 
	ros::Publisher gps_data_pub = nh.advertise<sb_msgs::CarCommand>(GPS_OUTPUT_TOPIC, 20);
	ros::Subscriber gps_sub = nh.subscribe(GPS_INPUT_TOPIC, 20, gpsCallback); 

	ros::Rate loop_rate(5); // 10hz loop rate

	// outputs twist msg
	geometry_msgs::Twist t_msg;
	t_msg.linear.x = 0;
	t_msg.linear.y = 0.0;
	t_msg.linear.z = 0;
	t_msg.angular.x = 0;
	t_msg.angular.y = 0;
	t_msg.angular.z = 0;

	GetWaypoint();

	// outputs carcommand msg
	sb_msgs::CarCommand cc_msg;
	cc_msg.throttle = 0.3;
	cc_msg.steering = 0;
	cc_msg.priority = 0;

	while (ros::ok()) 
	{
		//CalculateAngle(); // calculate angular.z
		steeringTest();
		t_msg.angular.z = steering;
		cc_msg.steering = steering;
		gps_test_pub.publish(t_msg);
		gps_data_pub.publish(cc_msg);

		ROS_INFO("\nLinear.x = %f\nLinear.y = %f\nAngular.z = %f\n", t_msg.linear.x, t_msg.linear.y, t_msg.angular.z);
		cout << "Curr Lat: " << current_waypoint.long_x << endl;
	        cout << "Curr Long: " << current_waypoint.lat_y << endl;
		cout << "Last long: " << last_waypoint.long_x << endl;
		cout << "Last lat: " << last_waypoint.lat_y << endl;
		cout << "Goal Lat: " << goal_waypoint.long_x << endl;
	        cout << "Goal Long: " << goal_waypoint.lat_y << endl;
		cout << "Curr angle: " << angle << endl;
		cout << "Last angle: " << last_angle << endl;
		//cout << "Compass: " <<  << endl;

		CheckWaypointStatus();
		if (isFinished) { return 0; }
		ros::spinOnce();
		loop_rate.sleep(); // sleep for 10hz
		GetWaypoint();
	}
	return 0;
}


// Get the next waypoint
void GetWaypoint() 
{
	if (c == 0 || isAtGoal) {
		if (c+2 > size) { isFinished = true; return; }
		goal_waypoint.long_x = waypoints_list[c];
		goal_waypoint.lat_y = waypoints_list[c+1];
		c += 2;
		isAtGoal = false;
		return;
	}
	return;
}


// Checks if we are sufficiently close to the waypoint
void CheckWaypointStatus() 
{
	// estimated to within ~11cm
	if (current_waypoint.long_x == goal_waypoint.long_x && current_waypoint.lat_y == goal_waypoint.lat_y) 
	{ 
		isAtGoal = true; 
	}
}


// Calculates Euclidean distance from position to goal
void CalculateAngle()
{
	 
/*
	x_dist = goal_waypoint.long_x - current_waypoint.long_x;
	y_dist = goal_waypoint.lat_y - current_waypoint.lat_y;
	if (x_dist == 0 && y_dist == 0) { steering = 0; return; }


	if (current_waypoint.long_x == 0 || current_waypoint.lat_y == 0) { steering = 0; return; }
	else {
		cout << y_dist << " " << x_dist << " " << endl;
		if (current_waypoint.long_x == goal_waypoint.long_x) {
			if (y_dist / abs(y_dist) == -1) { angle = 180.00; }
			else { angle = 0.00; }
		}
		else if (current_waypoint.lat_y == goal_waypoint.lat_y) {
			if (x_dist / abs(x_dist) == -1) { angle = 270.00; }
			else { angle = 90.00; }
		}
		else {
			if (y_dist / abs(y_dist) == -1) { angle = atan(abs(y_dist)/abs(x_dist)); angle = (angle*180)/3.1415; }
			if (y_dist / abs(y_dist) == 1) { angle = atan(abs(x_dist)/abs(y_dist)); angle = (angle*180)/3.1415; }

			if (x_dist / abs(x_dist) == -1) {
				if (y_dist / abs(y_dist) == -1) { angle += 180.00; }
				else { angle += 270.00; } 		
			}
			if (x_dist / abs(x_dist) == 1) { // if x positive
				if (y_dist / abs(y_dist) == -1) { angle += 90.0; }
			}
		}
	}


// ****************************************************************

	int x = current_waypoint.long_x - last_waypoint.long_x;
	int y = current_waypoint.lat_y - last_waypoint.lat_y;
	cout << "x: " << x << " y: " << y << endl;
	if (abs(x) < 1 && abs(y) < 1) { last_angle = 0; }

	if (last_waypoint.long_x == 0 || last_waypoint.lat_y == 0) { last_angle = 0; }
	else {
		if (abs(x) < 1) {
			if (abs(y) < 1) { last_angle = 0; }
			else if (y < 0) { last_angle = 180.00; }
			else { last_angle = 0.00; }
		}
		else if (abs(y) < 1) {
			if (abs(x) < 1) { last_angle = 0; }
			else if (x < 0) { last_angle = 270.00; }
			else { last_angle = 90.00; }
		}
		else {
			if (y < 0) { last_angle = atan(abs(y)/abs(x)); last_angle = (last_angle*180)/3.1415; }
			if (y > 0) { last_angle = atan(abs(x)/abs(y)); last_angle = (last_angle*180)/3.1415; }

			if (x < 0) {
				if (y < 0) { last_angle += 180.00; }
				else { last_angle += 270.00; } 		
			}
			if (x > 0) { // if x positive
				if (y < 0) { last_angle += 90.0; }
			}
		}
	}

Curr Lat: 4267818
Curr Long: -8319548
Last long: 4267818
Last lat: -8319548
Goal Lat: 4267821
Goal Long: -8319514
Curr angle: 0
Last angle: 0


// ***************************************************************



	if (abs(last_angle - angle) == 0 || abs(last_angle - angle) == 360) { steering = 0; return; }
	else if (last_angle < angle && abs(last_angle - angle) < 180) { steering = -0.1; return; }
	else if (last_angle > angle && abs(last_angle - angle) < 180) { steering = 0.1; return; }
	else if (last_angle < angle && abs(last_angle - angle) >= 180) { steering = 0.1; return; }
	else { steering = -0.1; return; }
	return;*/
}	 

void steeringTest(void)
{         

//scaling factors:111.412 lat, 78.847 long
// 
      
 	float longscale = 111.412/78.847;
	
/*
	current_waypoint.lat_y = current_waypoint.lat_y/10;	
	last_waypoint.lat_y =last_waypoint.lat_y/10;
	goal_waypoint.lat_y = goal_waypoint.lat_y/10;
*/
	if(current_waypoint.long_x != 0)
{
	 dx = (current_waypoint.long_x-last_waypoint.long_x)*longscale;
	 x = (goal_waypoint.long_x-current_waypoint.long_x)*longscale;
}

if(current_waypoint.lat_y != 0)
{
	
	 dy = current_waypoint.lat_y-last_waypoint.lat_y;	
	 y = goal_waypoint.lat_y-current_waypoint.lat_y;
}

if(current_waypoint.long_x !=0 || current_waypoint.lat_y !=0)
	{
	RunCount++;
	cout<<"UPDATE COUNT"<< RunCount<<endl;
	int lquadrant = 0;
	int cquadrant = 0;

	if((x>=0)&&(y>0)) cquadrant = 1;
	if((x<0)&&(y>=0)) cquadrant = 2;
	if((x<=0)&&(y<0)) cquadrant = 3;
	if((x>0)&&(y<=0)) cquadrant = 4;
	
	if((dx>=0)&&(dy>0)) lquadrant = 1;
	if((dx<0)&&(dy>=0)) lquadrant = 2;
	if((dx<=0)&&(dy<0)) lquadrant = 3;
	if((dx>0)&&(dy<=0)) lquadrant = 4;

	float ltheta = abs(atan(dy/dx));
	float ctheta =  abs(atan(y/x));
	float current_angle = 0;
	float l_angle = 0;
	
	if( cquadrant == 1)current_angle = ctheta;
	else if( cquadrant == 2)current_angle = 180-ctheta;
	else if( cquadrant == 3)current_angle = 180+ctheta;
	else if( cquadrant == 4)current_angle = 360-ctheta;
	
	if( lquadrant == 1)l_angle = ltheta;
	else if( lquadrant == 2)l_angle = 180-ltheta;
	else if( lquadrant == 3)l_angle = 180+ltheta;
	else if( lquadrant == 4)l_angle = 360-ltheta;

	if(current_angle-l_angle > 0) steering = 0.2;
	else if(current_angle-l_angle < 0) steering = -0.2;
	else if(current_angle-l_angle == 0) steering = 0.0;
	
	if(steering > 0)cout<<"HEADING LEFT"<<endl;
	if(steering < 0)cout<<"HEADING RIGHT"<<endl;
	if(steering == 0)cout<<"HEADING STRAIGHT"<<endl;
	
	}
	return;
}

// Retrieves waypoints from the waypoints file.
int* ReturnWaypoints() 
{
	string output;
	int count = 0;
	int array_size;
	int* waypoints_array = NULL;
	ifstream waypoints_file ("/home/mecanum/snowbots_ws/src/sb_gps/WaypointsTxt/practice1.txt");
	if (waypoints_file.is_open())
	{
		while (getline(waypoints_file, output))
		{
			if (count == 0) {
				array_size = atoi(output.c_str())*2;
				waypoints_array = new int [array_size];
				count++;
			}
			
			else { 
				waypoints_array[count-1] = atoi(output.c_str());
				count++;
			}
		}
		waypoints_file.close();
	}

	else
	{
		ROS_INFO("Unable to open file");
	}

	return waypoints_array;
}	
    

