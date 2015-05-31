#include "LidarController.hpp"

 using namespace ros;
 using namespace std;

 namespace ai{
 
 //Global constatns
static const double PI		 = 3.1415265;
static const double IGNORE_ANGLE = PI; 		//ignore rays beyond this angle
static const int    OFFSET_RAYS = 30;        // offset from central ray
static const double REDZONE      = 1;			// originally 0.5
static const double ORANGEZONE   = 1.5;			// originally 1
static const double SLOW_SPEED	 = 0.15;		// originally 0.1
static const double NORMAL_SPEED  = 0.25;
static const double SLOW_TURN	 = 0.15;
static const double SPEED_LIMIT  = 0.3; 		// originally 0.3
static const double TURN_LIMIT  = 0.4;
static const double THROTTLE_CONST = -1;
static const double STEERING_CONST  = -2;
static const unsigned int MICROSECOND = 2000000;	// sleep time
static const long US_IN_MS = 1000;
static const long S_IN_MS = 1000;

//ros related constants
static const std::string SUBSCRIBE_TOPIC = "scan";
static const std::string PUBLISH_TOPIC   = "lidar_nav";
static int LOOP_FREQ = 30;

 	// main function
	LidarController::LidarController(ros::NodeHandle& nh): 
	danger(0),
	backup(0), 
	timeDiffce(0.0)
	{

		// initialize some values
		prevObjectDist.x = 0;
		prevObjectDist.y = 0;
		prevObjectDist.z = 0;

		currentObjectDist.x = 0;
		currentObjectDist.y = 0;
		currentObjectDist.z = 0;

		gettimeofday(&prev_time, NULL);
		//Subscriber 
		lidar_state = nh.subscribe(SUBSCRIBE_TOPIC,20,&LidarController::callback,this);
		
	}

	// TODO: write a publisher for velocity 
	//(see sb_lidar::lidar_v1.cpp for detail)
	// do you need a new nodehandle for another publisher?
	geometry_msgs::Twist LidarController::Update()
	{
		
		geometry_msgs::Twist twistMsg = twist_converter(car_command);		// convert the car_command into a twistMsg
		
		std::cout<<"Throttle: " <<twistMsg.linear.y<<"   Steering: "<<  twistMsg.angular.z <<endl;
		
		ros::spinOnce();
		
		return twistMsg;
	}
	
	int LidarController::GetClockSpeed(){
		return LOOP_FREQ;
	}

	// callback	function
	void LidarController::callback(const sensor_msgs::LaserScanConstPtr& msg_ptr){
		
		int num_rays = msg_ptr->ranges.size();
		double x_total = 0.0;
		double y_total = 0.0;
		double x_nearest = 30.0;
		double y_nearest = 0.0;
		int valid_rays = 0;

	// get the time between each scan
		gettimeofday(&current_time, NULL);
		timeDiffce = (current_time.tv_sec - prev_time.tv_sec) * S_IN_MS ;		// seconds passed (in ms)
		timeDiffce += (current_time.tv_usec - prev_time.tv_usec)/ US_IN_MS ;	// microseconds passed (in ms)

	// if no time passed, don't update previous time
		if(timeDiffce != 0)
		prev_time = current_time;

		cout << "time difference : " << timeDiffce << "ms" << endl;


	// Count number of valid ray & calculates x and y totals
		for(int i =0; i < num_rays;i++)
		{
			float angle = msg_ptr->angle_min + i*(msg_ptr->angle_increment);
			float dist = msg_ptr->ranges[i];

			if(angle < -IGNORE_ANGLE)
				continue;
		
			if(angle > IGNORE_ANGLE)
				continue;
			
			if(dist == msg_ptr->range_max)
				continue;
	    
			if (dist != 0 && isinf(dist) == 0 && isnan(dist) == 0 && dist < 5 && dist > 0.1)	// check for valid distance
			{
				//cout<< "distance:" << dist <<endl;
				
				float force = -1.0/dist;

				if (isnan(cos(angle)) == false && isnan(sin(angle)) == false)
				{
					// detect the nearest object
					if(dist < x_nearest){
						x_nearest = dist;
						y_nearest = angle; 	// range from -.78(i.e. -pi/4) to .78(i.e. pi/4)
					
						currentObjectDist = convertPolar(x_nearest,y_nearest);

					}

					// sum up all the vectors
					x_total += 1 / (force * cos(angle));					
	        		y_total += force * cos(angle) * (angle/abs(angle));		// y_total is greatest if object is close and in center

					valid_rays++;											// count the number of rays that detect obj within 5m
				}
			}		
		}

		/*check the lidar velocity using a reference object*/
		cout << "nearest x is:" << x_nearest << endl;
		cout << "nearest y is:" << y_nearest << endl;

		cout << "currentObjectDist is (" << currentObjectDist.x << ", " << currentObjectDist.y << ")" <<endl;

		if(timeDiffce > 0){
		velocity = calculateVelocity(prevObjectDist, currentObjectDist);
		
		cout << "prevObjectDist is: x= " << prevObjectDist.x << ", y= " << prevObjectDist.y << endl;
		
		prevObjectDist = currentObjectDist;

		cout << "velocity is : x= " << velocity.x << ", y= " << velocity.y << " m/s"<<endl;
		}


		// if there are no objects nearby
		if(valid_rays <= 0)
		{
			ROS_FATAL("No valid rays found, vicinity is clear");
			danger = 0;
			car_command.throttle =  SPEED_LIMIT;
			car_command.steering = 0;						// go straight
			//r_command.priority = 0.5;
		}
		else
		// if there are objects nearby
		// if (valid_rays != 0)
		{

			// find the average steering given sum of all vectors
			car_command.steering =   -1 * y_total / valid_rays / 20 ;


			// steer away from object in front and center of lidar
			if(abs(y_nearest) < 0.0873) 					// if object is within 5 degrees from centre
			{
				car_command.steering = SLOW_TURN;			// turn right
				car_command.throttle = SLOW_SPEED;
				cout << "Object is in centre, defaulting to turning right" << endl;
			}
				

		} 

		// set default priority
		car_command.priority = 0.5;


		// check for blockages and sets throttle accordingly
		for(int i = num_rays/2-OFFSET_RAYS; i < num_rays/2+OFFSET_RAYS ; i++)
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
					//cout << "In REDZONE"<< endl;
			}
			else if (dist < ORANGEZONE)
			{
				car_command.throttle = SLOW_SPEED;
				car_command.priority = 0.8;
				danger = 1; 
				//backup = 0;
				//cout << "In ORANGEZONE"<< endl;
			}
			else
			{
				danger = 0;
				backup = 0;	// back up until green zone
	      		car_command.throttle = NORMAL_SPEED;
				//cout << "In GREENZONE"<< endl;
			}

		// back up functionality
		if(backup > 1){
				// delay 3 seconds
			cout << "Backing up: starting wait"<< endl;

			//if(backup > 2)
			usleep(MICROSECOND);	// sleep for 2 seconds

			cout << "Backing up: ended wait, starting to move"<< endl;
			
			// back up slowly
			car_command.throttle = -SLOW_SPEED;
			car_command.steering = 0.0;
			
     	 	break;			// exit the loop if backing up, i.e. in red or orange zone
			}
		}

		// restrict the throttle and steering
		car_command.throttle = clamp(car_command.throttle, SPEED_LIMIT);
		car_command.steering = clamp(car_command.steering, TURN_LIMIT);

	}


	/* clamp function sets an upperbound(cap) for the inputs (in)*/
	double LidarController::clamp (double in, double cap)
	{
		if      ( in >  cap) return cap;
		else if ( in < -1 * cap) return (-1 * cap);
		else                 return in;	
	}


	/*converts car command to twist message*/
	geometry_msgs::Twist LidarController::twist_converter(sb_msgs::CarCommand cc)
	{
		geometry_msgs::Twist twist;
		geometry_msgs::Vector3 Linear;
		geometry_msgs::Vector3 Angular;
		
		twist.linear.x = 0;
		twist.linear.y = cc.throttle * THROTTLE_CONST;	// the constant allows adjustments to throttle
		//twist.linear.y = 0;// changed to zero for demos
		twist.linear.z = 0;

		twist.angular.x = 0;
		twist.angular.y = 0;
		twist.angular.z = cc.steering * STEERING_CONST;  // the constant allows adjustments to steering

		return twist;	
	}

	geometry_msgs::Vector3 LidarController::convertPolar(double r, double theta){
	geometry_msgs::Vector3 xyCoord;

	xyCoord.x = r*sin(theta);
	xyCoord.y = r*cos(theta);
	xyCoord.z = 0.0;

	return xyCoord;
	}

	// calculates velocity given object distance
	geometry_msgs::Vector3 LidarController::calculateVelocity(geometry_msgs::Vector3 dist1, geometry_msgs::Vector3 dist2){
		geometry_msgs::Vector3 result;

		result.x = -(dist2.x - dist1.x)/timeDiffce*S_IN_MS;
		result.y = -(dist2.y - dist1.y)/timeDiffce*S_IN_MS;
		result.z = 0.0;

		return result;
	}
	
}

