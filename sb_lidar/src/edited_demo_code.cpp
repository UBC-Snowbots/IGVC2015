/*
 * lidar_force_nav
 * Intelligent Ground Vehicle Challenge 2014
 * Oakland University - Rochester, Michigan
 * June 2014
 * 
 * UBC Snowbots -- Team Avalanche
 * University of British Columbia
 *
 */

#include <unistd.h>
#include <math.h>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <sb_msgs/CarCommand.h>

 using namespace ros;
 using namespace std;

//Global constatns
 static const double PI		 = 3.1415265;
static const double IGNORE_ANGLE = PI; //pi radians = 180 degrees
static const int    OFFSET_RAYS = 30;        // offset from central ray
static const double REDZONE      = 0.5;			// originally 0.5
static const double ORANGEZONE   = 1.0;			// originally 1
static const double SLOW_SPEED	 = 0.15;		// originally 0.1
static const double SPEED_LIMIT  = 0.3; 		// originally 0.3

//ros related constants
static const string NODE_NAME       = "lidar_node";

static const string SUBSCRIBE_TOPIC = "scan";
static const string PUBLISH_TOPIC   = "lidar_nav";
static int LOOP_FREQ = 30;


geometry_msgs::Vector3 directions;
sb_msgs::CarCommand car_command;

int danger = 0;

double clamp (double in, double cap)
{
	if      ( in >  cap) return cap;
	else if ( in < -1 * cap) return (-1 * cap);
	else                 return in;	
}

// add object structure??


//call back function
void callback(const sensor_msgs::LaserScanConstPtr& msg_ptr)
{
	int num_rays = msg_ptr->ranges.size();
	double x_total = 0.0;
	double y_total = 0.0;
	double x_nearest = 30.0;
	double y_nearest = 0.0;
	int valid_rays = 0;
	int backup = 0;		// range 0-2, backs up robot when =2
	cout << "angle min"<< (msg_ptr->angle_min) << endl;
	cout << "angle increment" << (msg_ptr->angle_increment) << endl;


// Count number of valid ray & calculates x and y totals
	for(int i =0; i < num_rays;i++)
	{
		float angle = msg_ptr->angle_min + i*(msg_ptr->angle_increment);
		float dist = msg_ptr->ranges[i];
		if(angle < -IGNORE_ANGLE)
		{
			continue;
		}
		if(angle > IGNORE_ANGLE)
		{
			continue;
		}
		if(dist == msg_ptr->range_max)
		{
			continue;
		}

		if (dist != 0 && isinf(dist) == 0 && isnan(dist) == 0 && dist < 5)
		{
			//cout<< "distance:" << dist <<endl;
			
			float force = -1.0/dist;
			if (isnan(cos(angle)) == false && isnan(sin(angle)) == false)
			{
			//	x_total += force * cos(angle);

				// detect the nearest object
				if(dist < x_nearest){
					x_nearest = dist;
					y_nearest += force * sin(angle); 	// -300 to 300
				}

				// sum up all the vectors
				x_total += 1 / (force * cos(angle));	
				y_total += force * sin(angle);
				valid_rays++;
			}
		}		
	}

	if(valid_rays <= 0)
	{
		ROS_FATAL("No valid rays found");
		danger = 0;
		car_command.throttle =  SPEED_LIMIT;
		car_command.steering =   0;
		car_command.priority = 0.5;
		//return;
	}

	if (valid_rays != 0)
	{

	/*	float min_force = -1/4;
		float max_force = -1/0.02;
		
	*/	

		
		if (danger == 0)		
		{
			// find the average throttle given the sum of all vectors
			car_command.throttle =  -1 * x_total / valid_rays / 20;
		}

		// find the average steering given sum of all vectors
		car_command.steering =   -1 * y_total / valid_rays / 20;

		// cap the throttle?
		if ( car_command.throttle > 3500 )
			car_command.throttle = SLOW_SPEED;


		// steer away from object in front and center of lidar
		if(abs(y_nearest) < 10)
				car_command.steering = -1 * y_nearest/20;			// turn in direction away from the nearest obstacle
			

			// set priority
			car_command.priority = 0.5;

			
		/*figure out where to steer*/


		/*/ if the object is on the side of robot, do not steer.
		/if (abs(y_total) < 5) 
			car_command.steering = 0;

		else if (y_total < 110)
			{
			if (y_total < 0) 
				car_command.steering = -0.2;	// turn left
			else car_command.steering = 0.2;
			}

		else if (y_total > 110)
			{
			if   (y_total < 0) 
				car_command.steering = -0.2;

			else car_command.steering = 0.2;
			}*/
			
			
			
			// restrict the throttle and steering
			car_command.throttle = clamp(car_command.throttle, SPEED_LIMIT);
			car_command.steering = clamp(car_command.steering, 1);
			
			// for every valid ray, update carcommand
			ROS_INFO("x_total:  %f\t y_total:  %f\t\nthrottle: %f\t steering: %f", x_total, y_total, car_command.throttle, car_command.steering);	

		} 

	// check for blockages
		
		for(int i =num_rays/2-OFFSET_RAYS; i < num_rays/2+OFFSET_RAYS; i++)
		{
			float angle = msg_ptr->angle_min + i*(msg_ptr->angle_increment);
			float dist = msg_ptr->ranges[i];

			if (dist < REDZONE)
			{
			// stop moving forward
				car_command.throttle = 0;
				car_command.priority = 1;
				danger = 2;		
				backup++;
			break;	// added break
		}
		else if (dist < ORANGEZONE)
		{
			car_command.throttle = SLOW_SPEED;
			car_command.priority = 0.8;
			danger = 1; // back up until orange zone?
			backup = 0;
			break;	// added breaks
		}
		else
		{
			danger = 0;
			backup = 0;	// or back up until green zone?
		}

	// back up mechanism
		if(backup < 1){
				// delay 3 seconds
			usleep(3000000);
				// back up slowly
			car_command.throttle = -SLOW_SPEED;
				// reset signal
			//backup = 0;
		}

	}

	
}





/*converts car command to twist message*/
geometry_msgs::Twist twist_converter(sb_msgs::CarCommand cc)
{
	geometry_msgs::Twist twist;
	geometry_msgs::Vector3 Linear;
	geometry_msgs::Vector3 Angular;
	
	
	twist.linear.x = 0;
	twist.linear.y = -cc.throttle*1.5;
	//twist.linear.y = 0;// changed to zero for demos
	twist.linear.z = 0;

	twist.angular.x = 0;
	twist.angular.y = 0;
	twist.angular.z = -cc.steering*1.5;

	return twist;	
}


int main (int argc, char** argv)
{
	init(argc, argv,NODE_NAME);
	NodeHandle n;
	

	Subscriber lidar_state = n.subscribe(SUBSCRIBE_TOPIC,20,callback);
	
	Publisher car_pub = n.advertise<geometry_msgs::Twist>(PUBLISH_TOPIC,1);
	
	Rate loop_rate(LOOP_FREQ);
	ROS_INFO("ready to go");
	
	ROS_INFO("going");
	while(ros::ok())
	{
		geometry_msgs::Twist twistMsg = twist_converter(car_command);
	//	ROS_INFO("the twist vector is %f , %f", twistMsg.linear.y, twistMsg.angular.z);
	//	ROS_INFO("CarCommand {throttle: %0.2f , steering: %0.2f , priority: %0.2f}", car_command.throttle, car_command.steering, car_command.priority); 
		car_pub.publish(twistMsg);
		ros::spinOnce();
		loop_rate.sleep();	
	}
	ROS_INFO("shutting down node");
	
	return 0;
}

