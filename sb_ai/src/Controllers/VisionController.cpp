#include "VisionController.hpp"

using namespace cv; 
using namespace std; 
using namespace ros; 

namespace ai{
static const std::string PUBLISH_TOPIC2 = "vision_nav";
static const std::string SUBSCRIBE_TOPIC3 ="image_normal";
const int MSG_QUEUE_SIZE = 20;

int const upperBound = 255;
int const lowerBound = 180;
int const max_BINARY_value = 255;
int const threshold_value = 195;
int const kChiHigher = 5000;
int const kChiLower = 0;
//const sensor_msgs::ImageConstPtr& msg;


VisionController::VisionController(ros::NodeHandle& nh):
	blur_value(8),
	dx(0),
	dy(0),
	steeringOut(0),
	steering(0),
	steeringIncrement(0.1), //TODO: is this steering too strong?
	lowsteeringIncrement(0.1),
	priority(0),
	direction(0),
	noLinesWait(0),
	throttle(0.2), // TODO: is this throttle too strong?
	lowThrottle(0.2),
	count(0)
{

	//image = imread("/home/mecanum/Pictures/course1.jpg", 1);
	image_transport::ImageTransport it(nh);
    
    ROS_INFO("subbing");
    sub = it.subscribe(SUBSCRIBE_TOPIC3, 1, &VisionController::imageCallback, this);
    
    ROS_INFO("subbed"); 	                      
}

geometry_msgs::Twist VisionController::Update(){
	//imageCallback();
	if(!image.empty())
	{
		ROS_WARN("image obtained");
	    //detectLines();
	    getDirection();
	    displayWindows();
    }else ROS_INFO("image empty");
	twist.angular.z = steeringOut;
	twist.linear.y = throttle;
	ROS_INFO("Vision Published a twist : y linear- %f, z angular - %f",twist.linear.y,twist.angular.z);
	count++;
	return twist;
}


void VisionController::imageCallback(const sensor_msgs::Image::ConstPtr& msg){
	static unsigned int counter = 1;
	ROS_INFO("image callback run");
	try{
		ROS_INFO("Recieved an image for the %d time", counter);		
		image = cv_bridge::toCvShare(msg, "mono8")->image;
		image_thresholded = image.clone();
		image_thresholded.convertTo(image_direction32, CV_32F);
		cvtColor(image_direction32, image_direction, CV_GRAY2RGB);
		//displayWindows();
		//cv::imshow("subscribed", cv_bridge::toCvShare(msg, "mono8")->image);
		if(cv::waitKey(30) == 27){
			ROS_INFO("Shutdown event recieved");

			ros::shutdown();
		}
	}catch (cv_bridge::Exception& e){
		ROS_ERROR("Could not convert from '%s' to 'mono8'.", msg->encoding.c_str());
	}
	counter++;
}


void VisionController::getDirection(void) {
/*
	int rows2Check = 14;
	int minConstraint = 6; // need this many, or more point to define a line
	int distanceBetweenRows = image.rows / 20;
	int const startRow = image.rows / 2 + distanceBetweenRows*4; //TODO: adjust for camera angle
	int row = startRow;
*/
	int rows2Check = 100;
	int minConstraint = 40; // need this many, or more point to define a line
	int distanceBetweenRows = -image.rows / rows2Check;
	int const startRow = 0;//image.rows/3;//image.rows/2+distanceBetweenRows*19; //TODO: adjust for camera angle
	//int const startRow = distanceBetweenRows*19; 	
	int row = startRow;

	Point points[rows2Check][4];
	int transitions[rows2Check];

	int lastValue = (image_thresholded.at<uchar>(row, 0)) % 2;
	//Check how many transitions occur in each row and put into array
	int currentValue = 0;

	image_direction = image_thresholded.clone();

	for (int i = 0; i < rows2Check; i++) {
		transitions[i] = countLines(row, 1);
		row = row - distanceBetweenRows;
	}
	//cout << "Transitions"<<transitions <<endl;
	row = startRow;

	/* Categorize each transition
	 1. left side of left line
	 2. right side of left line
	 3. left side of right line
	 4. right side of right line
	 */

	for (int i = 0; i < rows2Check; i++) {

		//Initialize variables
		lastValue = (image_thresholded.at<uchar>(row, 0)) % 2;
		int left = 0;
		Point centre;

		if (transitions[i] == 0) {
			//cout << "Error: No lines detected at row" << i << endl;
		} else if (transitions[i] > 4) {
			//cout << "Error: More than 2 lines detected at row" << i << endl; // consider adjusting thresholding values here
		} else {
			for (int f = 0; f < image_thresholded.cols; f++) {
				//lastL = L;
				//lastR = R;
				//if (lastL && !lastR)
				currentValue = image_thresholded.at<uchar>(row, f) % 2;
				if (currentValue != lastValue) {
					//cout << "Transition at x = " << f << ", y = " << row	<< endl;
					centre.x = f;
					centre.y = row;
					//cout << "Point at = " << centre << endl;

					//circle(image_direction, centre, 20, CV_RGB(250, 100, 255),
					//		1, 8, 0);
					//cout << "Current Value: " << currentValue << " Last Value"
					//		<< lastValue << endl;

					//Here comes the fun!!!! Value of black is 1 and value of white is 0...
					// This parts is pretty nasty, but works quite beautifully!
					// 1 transition
					if (transitions[i] == 1) {
						if ((lastValue == 1) && (currentValue == 0))
							points[i][1] = centre;
						else if ((lastValue == 0) && (currentValue == 1))
							points[i][2] = centre;
						left = 0;
					} else if (transitions[i] == 2) // this part right now just checks which side things are on
							{
						if (left == 0) {
							if ((lastValue == 1) && (currentValue == 0))
								points[i][1] = centre; //RoL
							else if ((lastValue == 0) && (currentValue == 1))
								{
								if (f <= image_thresholded.cols / 2)
									points[i][0] = centre; //LoL
								if (f > image_thresholded.cols / 2)
									points[i][2] = centre; //LoR
								}
							left = 1;
						} else if (left == 1) {
							if ((lastValue == 1) && (currentValue == 0)) {
								if (f <= image_thresholded.cols / 2)
									points[i][1] = centre; // RoL
				    				if (f > image_thresholded.cols / 2)
									points[i][3] = centre; //LoL
							} else if ((lastValue == 0) && (currentValue == 1))
								points[i][2] = centre; //LoR
						}
					} else if (transitions[i] == 3) {
						if (left == 0) {
							if ((lastValue == 1) && (currentValue == 0))
								points[i][1] = centre; // RoL
							else if ((lastValue == 0) && (currentValue == 1))
								points[i][0] = centre; //LoL
							left = 1;
						} else if (left == 1) {
							if ((lastValue == 1) && (currentValue == 0))
								points[i][1] = centre; // RoL
							else if ((lastValue == 0) && (currentValue == 1))
								points[i][2] = centre; //LoR
							left = 2;
						} else if (left == 2) {
							if ((lastValue == 1) && (currentValue == 0))
								points[i][3] = centre; // RoL
							else if ((lastValue == 0) && (currentValue == 1))
								points[i][2] = centre; //LoR
						}
					} else if (transitions[i] == 4) {
						if (left == 0) {
							if ((lastValue == 0) && (currentValue == 1))
								points[i][0] = centre; //LoL
							left = 1;
						} else if (left == 1) {
							if ((lastValue == 1) && (currentValue == 0))
								points[i][1] = centre; // RoL
							left = 2;
						} else if (left == 2) {
							if ((lastValue == 0) && (currentValue == 1))
								points[i][2] = centre; // RoL
							left = 3;
						} else if (left == 3) {
							if ((lastValue == 1) && (currentValue == 0))
								points[i][3] = centre; // RoL
						}
					}
				}
				lastValue = currentValue;
			}

			//cout<<"Left = " << left << "Row: " << row << endl;

		}
		drawLine(row);
		row = row - distanceBetweenRows;
		left = 0;
	}

	//SUPER GHETTO STUFFS
	float slope0 = 0;
	float slope1 = 0;
	float slope2 = 0;
	float slope3 = 0;
	float yIntercept0 = 0;
	float yIntercept1 = 0;
	float yIntercept2 = 0;
	float yIntercept3 = 0;
	float chiSquared0 = 100000;
	float chiSquared1 = 100000;
	float chiSquared2 = 100000;
	float chiSquared3 = 100000;
	// 0 aka Left of Left -----------------------------

	Mat Xchecked0;
	Mat Ychecked0;
	Mat Bchecked0;
	int X0Count = 0;
	Bchecked0 = Mat::ones(2, 1, CV_32F);

	for (int i = 0; i < rows2Check; ++i) {
		if (points[i][0].x > 0) {
			float Xrow[] = { points[i][0].x, 1 };
			float Yrow[] = { points[i][0].y };
			Mat newRowX = Mat(1, 2, CV_32F, Xrow).clone();
			Xchecked0.push_back(newRowX);
			Mat newRowY = Mat(1, 1, CV_32F, Yrow).clone();
			Ychecked0.push_back(newRowY);
			X0Count++;
		}
	}

	//cout << "X0 checked" << Xchecked0 << endl;
	//cout << "Y0 checked" << Ychecked0 << endl;
	if (X0Count >= minConstraint) {
		solve(Xchecked0, Ychecked0, Bchecked0, DECOMP_QR);

		//cout << "B0 checked = " << endl << Bchecked0 << endl;

		// Next we need to calculate how much we want to shift by: using the line equation obtained in matrix B we can calculate the
		// y-intercept
		slope0 = Bchecked0.at<float>(0, 0);
		yIntercept0 = Bchecked0.at<float>(1, 0);
		//cout << "slope0 = " << slope0 << endl;
		//cout << "yIntercept0 = " << yIntercept0 << endl;

		float xIntercept0 = -yIntercept0 / slope0;
		//cout << "xIntercept0 = " << xIntercept0 << endl;

		if(yIntercept0 != 0)chiSquared0 = chiSquared(Xchecked0,Ychecked0, yIntercept0, slope0, X0Count,0);
        //cout<<"chiSquared0:"<<chiSquared0<<endl;
		//if ((yIntercept0 != 0) && (chiSquared0 < CHISQUAREDTHRESH)){
			//Draw Line
		if ((yIntercept0 != 0)&& (chiSquared0 > kChiLower) &&(chiSquared0 < kChiHigher )){
		//if(yIntercept0 != 0){
			Point Intercept0 = Point(xIntercept0, 0);
			Point Intercept640 = Point((640 - yIntercept0) / slope0, 640);
			line(image_direction, Intercept0, Intercept640,
					CV_RGB(250, 100, 255), 10, CV_AA);

		}else slope0 = 0;
	}
		// 1 aka Right of Left
		Mat Xchecked1;
		Mat Ychecked1;
		Mat Bchecked1;
		int X1Count = 0;
		Bchecked1 = Mat::ones(2, 1, CV_32F);
		for (int i = 0; i < rows2Check; ++i) {
			if (points[i][1].x > 0) {
				float Xrow[] = { points[i][1].x, 1 };
				float Yrow[] = { points[i][1].y };
				Mat newRowX = Mat(1, 2, CV_32F, Xrow).clone();
				Xchecked1.push_back(newRowX);
				Mat newRowY = Mat(1, 1, CV_32F, Yrow).clone();
				Ychecked1.push_back(newRowY);
				X1Count++;
			}
		}

		//cout << "X1 checked" << Xchecked1 << endl;
		//cout << "Y1 checked" << Ychecked1 << endl;
		if (X1Count >= minConstraint) {
			solve(Xchecked1, Ychecked1, Bchecked1, DECOMP_QR);

		//	cout << "B1 checked = " << endl << Bchecked1 << endl;
			// Next we need to calculate how much we want to shift by: using the line equation obtained in matrix B we can calculate the
			// y-intercept
			slope1 = Bchecked1.at<float>(0, 0);
			yIntercept1 = Bchecked1.at<float>(1, 0);
		//	cout << "slope1 = " << slope1 << endl;
		//	cout << "yIntercept1 = " << yIntercept1 << endl;

			float xIntercept1 = -yIntercept1 / slope1;
		//	cout << "xIntercept1 = " << xIntercept1 << endl;

		if(yIntercept1 != 0)chiSquared1 = chiSquared(Xchecked1,Ychecked1, yIntercept1, slope1, X1Count,1);
        //cout<<"chiSquared1:"<<chiSquared1<<endl;
		//	if ((yIntercept1 != 0) && (chiSquared1 < CHISQUAREDTHRESH)) {
			if ((yIntercept1 != 0)&& (chiSquared1 > kChiLower) &&(chiSquared1 < kChiHigher )){
			//if(yIntercept1 != 0){
				//Draw Line
				Point Intercept0 = Point(xIntercept1, 0);
				Point Intercept640 = Point((640 - yIntercept1) / slope1, 640);
				line(image_direction, Intercept0, Intercept640,
						CV_RGB(250, 100, 255), 10, CV_AA);
			}else slope1 = 0;
		}
		// 2 aka Left of Right
		Mat Xchecked2;
		Mat Ychecked2;
		Mat Bchecked2;
		int X2Count = 0;
		Bchecked2 = Mat::ones(2, 1, CV_32F);
		for (int i = 0; i < rows2Check; ++i) {
			if (points[i][2].x > 0) {
				float Xrow[] = { points[i][2].x, 1 };
				float Yrow[] = { points[i][2].y };
				Mat newRowX = Mat(1, 2, CV_32F, Xrow).clone();
				Xchecked2.push_back(newRowX);
				Mat newRowY = Mat(1, 1, CV_32F, Yrow).clone();
				Ychecked2.push_back(newRowY);
				X2Count++;
			}
		}

		//cout << "X2 checked" << Xchecked2 << endl;
		//cout << "Y2 checked" << Ychecked2 << endl;

		if (X2Count >= minConstraint) {
			solve(Xchecked2, Ychecked2, Bchecked2, DECOMP_QR);

		//	cout << "B2 checked = " << endl << Bchecked2 << endl;
			//want to stream line this by putting points directly into Mat, but for now just going to get something that works

			// Next we need to calculate how much we want to shift by: using the line equation obtained in matrix B we can calculate the
			// y-intercept
			slope2 = Bchecked2.at<float>(0, 0);
			yIntercept2 = Bchecked2.at<float>(1, 0);
		//	cout << "slope2 = " << slope2 << endl;
		//	cout << "yIntercept2 = " << yIntercept2 << endl;

			float xIntercept2 = -yIntercept2 / slope2;
		//	cout << "xIntercept2 = " << xIntercept2 << endl;

			if(yIntercept2 != 0)chiSquared2 = chiSquared(Xchecked2,Ychecked2, yIntercept2, slope2, X2Count,2);
			//cout<<"chiSquared2:"<<chiSquared2<<endl;
			//if ((yIntercept2 != 0)&&(chiSquared2 < CHISQUAREDTHRESH)) {
			//if(yIntercept2 != 0){
			if ((yIntercept2 != 0)&& (chiSquared2 > kChiLower) &&(chiSquared2 < kChiHigher )){
				//Draw Line
				Point Intercept0 = Point(xIntercept2, 0);
				Point Intercept640 = Point((640 - yIntercept2) / slope2, 640);
				line(image_direction, Intercept0, Intercept640,
						CV_RGB(250, 100, 255), 10, CV_AA);
			}else slope2 = 0;
		}
		//3 aka Right of Right

		Mat Xchecked3;
		Mat Ychecked3;
		Mat Bchecked3;
		int X3Count = 0;
		Bchecked3 = Mat::ones(2, 1, CV_32F);
		for (int i = 0; i < rows2Check; ++i) {
			if (points[i][3].x > 0) {
				float Xrow[] = { points[i][3].x, 1 };
				float Yrow[] = { points[i][3].y };
				Mat newRowX = Mat(1, 2, CV_32F, Xrow).clone();
				Xchecked3.push_back(newRowX);
				Mat newRowY = Mat(1, 1, CV_32F, Yrow).clone();
				Ychecked3.push_back(newRowY);
				X3Count++;
			}
		}

		//want to stream line this by putting points directly into Mat, but for now just going to get something that works
		//cout << "X3 checked" << Xchecked3 << endl;
		//cout << "Y3 checked" << Ychecked3 << endl;
		if (X3Count >= minConstraint) {
			solve(Xchecked3, Ychecked3, Bchecked3, DECOMP_QR);
		//	cout << "B3 checked = " << endl << Bchecked3 << endl;

			// Next we need to calculate how much we want to shift by: using the line equation obtained in matrix B we can calculate the
			// y-intercept
			slope3 = Bchecked3.at<float>(0, 0);
			yIntercept3 = Bchecked3.at<float>(1, 0);
		//	cout << "slope3 = " << slope3 << endl;
		//	cout << "yIntercept3 = " << yIntercept3 << endl;
			float xIntercept3 = -yIntercept3 / slope3;
		//	cout << "xIntercept3 = " << xIntercept3 << endl;
			if(yIntercept3 != 0)chiSquared3 = chiSquared(Xchecked3,Ychecked3, yIntercept3, slope3, X3Count,3);
		    //cout<<"chiSquared3:"<<chiSquared3<<endl;
			if ((yIntercept3 != 0)&& (chiSquared3 > kChiLower) &&(chiSquared3 < kChiHigher )) {
		//	if(yIntercept3 != 0){
				//Draw Line
				Point Intercept0 = Point(xIntercept3, 0);
				Point Intercept640 = Point((640 - yIntercept3) / slope3, 640);
				line(image_direction, Intercept0, Intercept640,
						CV_RGB(250, 100, 255), 10, CV_AA);
			}else slope3 = 0;

		}

			Xchecked0.release();
			Ychecked0.release();
			Bchecked0.release();
			Xchecked1.release();
		    Ychecked1.release();
			Bchecked1.release();
			Xchecked2.release();
			Ychecked2.release();
			Bchecked2.release();
			Xchecked3.release();
		    Ychecked3.release();
			Bchecked3.release();


			cout << "Slope 0:"<< slope0<<endl;
			cout << "Slope 1:"<< slope1<<endl;
			cout << "Slope 2:"<< slope2<<endl;
			cout << "Slope 3:"<< slope3<<endl;

		// Crikey!
		float minSlope = 0.0001;
		bool LoL = (slope0 < -minSlope)||(slope0 > minSlope ); // these values we chosen to reduce errors
		bool RoL = (slope1 < -minSlope)||(slope1 > minSlope );
		bool LoR = (slope2 < -minSlope)||(slope2 > minSlope );
		bool RoR = (slope3 < -minSlope)||(slope3 > minSlope );
		bool R; //  right line detected?
		bool L; // left line detected?

		if (LoL && RoL) L = 1;
		if (LoR && RoR) R = 1;

        cout<<"Lines detected"<<endl;
        if(LoL) cout<<"Left of Left"<<endl;
        if(RoL) cout <<"Right of Left"<<endl;
        if(LoR) cout << "Left of Right" <<endl;
        if(RoR) cout <<"Right of Right" <<endl;
        /*
		// Perform HitTest if hit set to 1 & move away
		int robotPosy = image_thresholded.cols + 30; //TODO: alter this parameter
		if(LoL||RoL||LoR||RoR) priority = 0;
		if (LoL && ((robotPosy - yIntercept0)/ slope0) > 1/3*image_thresholded.cols){
			priority = 1;
			throttle = lowThrottle;
			steering = lowsteeringIncrement;
			cout<<"GOING LEFT"<<endl;
		}
		else if (RoL && ((robotPosy - yIntercept1)/ slope1) > 1/3*image_thresholded.cols){
			priority = 1;
			throttle = lowThrottle;
			steering = lowsteeringIncrement;
			cout<<"GOING LEFT"<<endl;
		}
		else if (LoR && ((robotPosy - yIntercept2)/slope2) < 2/3*image_thresholded.cols){
			priority = 1;
			throttle = lowThrottle;
			steering = -lowsteeringIncrement;
			cout<<"GOING RIGHT"<<endl;
		}
		else if (RoR && ((robotPosy - yIntercept3)/slope3) < 2/3*image_thresholded.cols){
			priority = 1;
			throttle = lowThrottle;
			steering = -lowsteeringIncrement;
			cout<<"GOING RIGHT"<<endl;
		}
		else if (!LoL&&!RoL&&!LoR&&!RoR)
		{
			cout<<"NO LINES DETECTED, GOING STRAIGHT"<<endl;
			noLinesWait++;
			throttle = lowThrottle;
			//steering = steering *(-1); // added this in now

			//waitKey(1); //Should this be here? Consult with others
			//if (noLinesWait > 100) //TODO: is this enough?
			//{
			//	priority = -1;
			//	noLinesWait = 0;
			//}
		}
		*/
		//if (priority == 1) {cout<<" ABOUT TO HIT LINE" << endl;return;}
		//if (priority == -1) {cout<<"NO LINES DETECTED FOR EXTENDED PEROID SWITCHING TO GPS" << endl; return;}
		// Otherwise perform direction test and move

		double leftSlope;
		double rightSlope;
		double left_y_intercept;
		double right_y_intercept; 
		double x_cross;

	    if (L) 
		{
		  leftSlope = (slope0+slope1)/2;
		  left_y_intercept =(yIntercept0+yIntercept1)/2;
		}
		else if (LoL)
		{ 
			leftSlope = slope0;
			left_y_intercept = yIntercept0;
		}
		else if (LoR)
		{ 
			leftSlope = slope1;
			left_y_intercept = yIntercept1;
		}
		if (R) 
		{
		  rightSlope = (slope2+slope3)/2;
		  right_y_intercept =(yIntercept2+yIntercept3)/2;
		}
		else if (LoR)
		{ 
			rightSlope = slope2;
			right_y_intercept = yIntercept2;
		}
		else if (RoR)
		{ 
			rightSlope = slope3;
			right_y_intercept = yIntercept3;
		}
			


       	if ((L && R) ||((LoL || RoL) && (LoR || RoR)))
		{
			cout<<"Two lines detected"<<endl;
			//direction = (leftSlope + rightSlope)/2;
			//TODO: calculate direction
			x_cross = (left_y_intercept-right_y_intercept)/(rightSlope-leftSlope);
			direction =  image.cols/2-x_cross; //  left if positive, right if negative
		}
		else {
		if (L || LoL || RoL)
		{
	 		cout<<"One line on left"<<endl;
            //TODO:Calculate direction 
            if((left_y_intercept>(image.cols/3)) && (leftSlope>0)) 
            	direction = - left_y_intercept + image.cols/2;
            else direction = 0;
		}
	    if(R ||LoR||RoR)
	    {
	    	cout<<"One line on right"<<endl;
	    	if((right_y_intercept<(image.cols*2.0/3)) && (leftSlope<0)) 
            	direction = - right_y_intercept + image.cols/2;
            else direction = 0;
		}
		else if (!LoL&& !RoL && !LoR && !RoR)
        {
        	cout<<"No lines detected"<<endl;
			direction = 0;
		}
		else cout <<"unaccounted logic"<<endl;
        }
        //New steering function
        cout << "direction:" << direction<<endl;

        // Steer according to direction 
		if ((direction > 0)&&(direction <10))
			steering = lowsteeringIncrement;
		else if (direction > 10) 
			steering = steeringIncrement;
		else if ((direction < 0)&&(direction > -10))
			steering = -lowsteeringIncrement;
		else if ((direction < 0))
			steering = -steeringIncrement;
	
	
		if (direction == 0 ) {
			steering = 0;
		}

		//Cap steering at maximum value
		if (steering > 1)
			steering = 1;
		if (steering < -1)
			steering = -1;
        
        cout<<"steering:"<<steering<< endl;
		steeringOut = steering;
		if (steering < 0)cout << "HEADING RIGHT" << endl;
		if (steering > 0)cout << "HEADING LEFT" << endl;
		if (steering == 0)cout << "HEADING STRAIGHT" << endl;

		cout << "Steering = " << steeringOut << endl;
		cout << "Throttle = " << throttle <<endl;

	}
	/*
	 float(calculateDirection){

	 }
	 */
	float VisionController::calculateDirection1(void) {
		int iterations = 2;
		int distanceBetweenRows = image.rows / 10;
		int startRow = image.rows / 2; //TODO: adjust for camera angle
		int row = startRow -	 distanceBetweenRows;

		Point point0;
		Point point1;
		int currentValue0;
		int lastValue0 = (image_thresholded.at<uchar>(row, 0)) % 2;
		int currentValue1;
		int lastValue1 = (image_thresholded.at<uchar>(row, 0)) % 2;

		for (int i = 0; i < image_thresholded.cols; i++) {
			//Mit = (image_thresholded.at<uchar>(row, i)) % 2;
			//if (Mit != lastValue) {
			//	transitionCount++;

			//lastValue = Mit;
		}
		return 0;
	}

// Detect slopes
	/*
	 transitions = transitions/iterations; //may have rounding error here

	 int lines = (transitions+1)/2*2;
	 cout<< "Lines: " << lines << endl;

	 if (transitions == 0)cout<< "Error: 0 lines detected, keep going in same direction"<<endl;
	 else if (transitions == 1);
	 else if (transitions == 2);
	 else if (transitions == 3);
	 else if (transitions == 4);
	 else if (transitions >= 5)cout<< "Error: Noise, more than 2 lines detected"<<endl;

	 cout<<"Direction: "<<direction<<endl;*/

	void VisionController::detectLines(void) {
		ROS_INFO("detectLines started");
		int row = image.rows / 2 - 52; // starting row for checking direction
		int betweenRow = 10;
		int x = 0;
		int y = 0;
        ROS_INFO("image loaded");

		int numLines = 0;
		for (int i = 0; i <= 1; i++) {
			//Draw lines on thresholded image, where it is being checked
			drawLine(row);
			ROS_INFO("line drawn");
			//findcentre(row);
			numLines = countLines(row, 0); // how many lanes are detected
			if (numLines == 0)
				ROS_INFO("Error: No lines visible"); //direction will stay as before
			if (numLines >= 3)
				cout << "Error: Noise, more than 3 lines detected" << endl; //direction will stay as before
			if (numLines == 2) {
				x = findcentre2(row); // if we detect two lines find the middle of the lane
				y = row;
			}
			if (numLines == 1) {
				x = findcentre1(row); // if we detect one lines find the middle of the lane
				y = row;
			}
			if (i == 0) {
				dx = x;
				dy = y;
			}
			if (i == 1) {
				dx = x - dx;
				dy = dy - y;
				cout << "Rise/Run: " << dy << "/" << dx << endl;
			}

			row = row - betweenRow;
		}
	}

	void VisionController::displayWindows(void) {

		//Display image
		//if(!image.empty())
		//{
		//namedWindow("Display Image", CV_WINDOW_NORMAL);
		//cvMoveWindow("Display Image", 400, 0);
		//imshow("Display Image", image);
	    //}
/*
		//Display blur
		namedWindow("Blur", CV_WINDOW_NORMAL);
		cvMoveWindow("Blur", 720, 0);
		imshow("Blur", image_blur);

		//Display after blur
		namedWindow("After Blur", CV_WINDOW_NORMAL);
		cvMoveWindow("After Blur", 400, 300);
		imshow("After Blur", image_blur2);
*/
		//Display binary threshold thresholded image
		if(!image_thresholded.empty()){
		namedWindow("Binary Threshold", CV_WINDOW_NORMAL);
		cvMoveWindow("Binary Threshold", 720, 300);
		imshow("Binary Threshold", image_thresholded);
	    }
/* 
		//Display Hue thresholded image
		namedWindow("Hue Otsu Threshold", CV_WINDOW_NORMAL);
		cvMoveWindow("Hue Otsu Threshold", 0, 300);
		imshow("Hue Otsu Threshold", image_HThresh);

		//Display Value thresholded image
		namedWindow("Value Threshold", CV_WINDOW_NORMAL);
		cvMoveWindow("Value Threshold", 720, 600);
		imshow("Value Threshold", image_VThresh);

		//Display H histogram
		namedWindow("H Histogram", CV_WINDOW_NORMAL);
		cvMoveWindow("H Histogram", 1150, 0);
		imshow("H Histogram", histogram_H);

		//Display S histogram
		namedWindow("S Histogram", CV_WINDOW_NORMAL);
		cvMoveWindow("S Histogram", 1150, 300);
		imshow("S Histogram", histogram_S);

		//Display V histogram
		namedWindow("V Histogram", CV_WINDOW_NORMAL);
		cvMoveWindow("V Histogram", 1150, 600);
		imshow("V Histogram", histogram_V);

		//Display direction image
		namedWindow("Canny", CV_WINDOW_NORMAL);
		cvMoveWindow("Canny", 0, 0);
		imshow("Canny", image_canny);
*/
		//Display direction image
		if(!image_direction.empty())
		{
		namedWindow("Direction", CV_WINDOW_NORMAL);
		cvMoveWindow("Direction", 0, 600);
		imshow("Direction", image_direction);
	    }
/*
//Display H image
		namedWindow("Hue", CV_WINDOW_NORMAL);
		cvMoveWindow("Hue", 720, 600);
		imshow("Hue", image_H);

		//Display S image
		namedWindow("Saturation", CV_WINDOW_NORMAL);
		cvMoveWindow("Saturation", 720, 600);
		imshow("Saturation", image_S);

		//Display Value thresholded image
		namedWindow("Value ", CV_WINDOW_NORMAL);
		cvMoveWindow("Value ", 720, 600);
		imshow("Value Threshold", image_V);

		//Display R image
		namedWindow("Red", CV_WINDOW_NORMAL);
		cvMoveWindow("Red", 720, 600);
		imshow("Red", image_R);

		//Display G image
		namedWindow("Green", CV_WINDOW_NORMAL);
		cvMoveWindow("Green", 720, 600);
		imshow("Green", image_G);

		//Display B image
		namedWindow("Blue", CV_WINDOW_NORMAL);
		cvMoveWindow("Blue", 720, 600);
		imshow("Blue", image_B);


		//Display B image

		namedWindow("No G", CV_WINDOW_NORMAL);
		cvMoveWindow("No G", 720, 600);
	    imshow("No G", image_noG);

*/

	}

//Draws a horizontal line at row
	void VisionController::drawLine(int row) {
		Point pt1, pt2;
		pt1.x = 0;
		pt1.y = row;
		pt2.x = image.cols;
		pt2.y = row;

		//line(image_direction, pt1, pt2, /*CV_RGB(0, 0, 0)*/ 0, 3, CV_AA);
	}

//Detects where line are and highlights them using circles
	void VisionController::findcentre(int row) {
		//TODO: for pixels in line find high low changeovers calculate centre of white lines then centre of path
		int Mit;
		int sum = 0;
		int count = 0;
		int average;
		for (int i = 0; i < image_thresholded.cols; i++) {
			Mit = (image_thresholded.at<uchar>(row, i)) % 2; // had problems with data type of binary image. Modulo works to get either 1 or 0
			cout << Mit << endl;
			if (Mit > 0) {
				//Highlight area where circle is detected
				Point centre;
				centre.x = i;
				centre.y = row;
				//circle(image_direction, centre, 5, CV_RGB(250, 100, 255), 1, 8,
				//		0);
				sum = sum + i;
				count++;
			}
		}
		average = sum / count;
		//Put large dot in average of white lines
		Point centre;
		centre.x = average;
		centre.y = row;
		//circle(image_direction, centre, 10, CV_RGB(250, 100, 255), 1, 8, 0);
	}

//Improved version of find direction when exactly 1 lane has been detected
	int VisionController::findcentre1(int row) {
		int Mit;
		int countWhite = 0;
		int centreWhite1 = 0;
		int centreLane;
		int transition = 0;
		int lastValue = (image_thresholded.at<uchar>(row, 0)) % 2;
		for (int i = 0; i < image_thresholded.cols; i++) {
			Mit = (image_thresholded.at<uchar>(row, i)) % 2; // had problems with data type of binary image. Modulo works to get either 1 or 0
			if (lastValue != Mit) {
				if (transition == 0)
					countWhite = i;
				else if (transition == 1) {
					centreWhite1 = (countWhite + i) / 2;
					cout << "centreWhite1: " << centreWhite1 << endl;
					countWhite = 0;
				}
				transition++;
			}
			lastValue = Mit;
		}
		if (centreWhite1 >= image_thresholded.cols / 2)
			centreLane = (centreWhite1) / 2;
		if (centreWhite1 < image_thresholded.cols / 2)
			centreLane = (centreWhite1 + image_thresholded.cols) / 2;
		cout << "CentreLane:" << centreLane << endl;
		//Put large dot in average of white lines
		Point centre;
		centre.x = centreLane;
		centre.y = row;
		//circle(image_direction, centre, 20, CV_RGB(250, 100, 255), 1, 8, 0);
		return centreLane;
	}

//Improved version of find centre when exactly 2 lanes have been detected
	int VisionController::findcentre2(int row) {
		int Mit;
		int countWhite = 0;
		int centreWhite1 = 0;
		int centreWhite2 = 0;
		int centreLane;
		int transition = 0;
		int lastValue = (image_thresholded.at<uchar>(row, 0)) % 2;
		for (int i = 0; i < image_thresholded.cols; i++) {
			Mit = (image_thresholded.at<uchar>(row, i)) % 2; // had problems with data type of binary image. Modulo works to get either 1 or 0
			if (lastValue != Mit) {

				if (transition == 0) {
					countWhite = i;
					centreWhite1 = countWhite;
				}

				else if (transition == 1) {
					centreWhite1 = (countWhite + i) / 2;
					countWhite = 0;
				} else if (transition == 2) {
					countWhite = i;
					centreWhite2 = countWhite;
				} else if (transition == 3) {
					centreWhite2 = (countWhite + i) / 2;
					countWhite = 0;
				}
				transition++;
			}
			lastValue = Mit;
		}
		centreLane = (centreWhite1 + centreWhite2) / 2;
		cout << "centreWhite1: " << centreWhite1 << endl;
		cout << "centreWhite2: " << centreWhite2 << endl;
		cout << "CentreLane:" << centreLane << endl;
		//Put large dot in average of white lines
		Point centre;
		centre.x = centreLane;
		centre.y = row;
		//circle(image_direction, centre, 20, CV_RGB(250, 100, 255), 1, 8, 0);
		return centreLane;
	}

// Counts the number of lines seen across the image
	int VisionController::countLines(int row, bool mode) {
		int Mit;
		int lineCount = 0;
		int transitionCount = 0;
		int lastValue = (image_thresholded.at<uchar>(row, 0)) % 2;
		for (int i = 0; i < image_thresholded.cols; i++) {
			Mit = (image_thresholded.at<uchar>(row, i)) % 2;
			if (Mit != lastValue) {
				transitionCount++;
			}
			lastValue = Mit;
		}
		lineCount = ceil(transitionCount / 2.0);
		//cout << "Number of Transitions: " << transitionCount << endl;
		//cout << "Number of Lines: " << lineCount << endl;
		if (mode == 0)
			return lineCount;
		if (mode == 1)
			return transitionCount;
	}

	/**
	 * This is similar to the implementation of Robert Laganière.
	 * See his book: OpenCV 2 Computer Vision Application Programming Cookbook.
	 */
	cv::Mat VisionController::showHistogram(const cv::Mat &inImage) {

		cv::MatND hist;
		// For a gray scale [0:255] we have 256 bins
		const int bins[1] = { 256 };
		const float hranges[2] = { 0.0, 255.0 };
		const float* ranges[1] = { hranges };
		const int channels[1] = { 1 };

		cv::calcHist(&inImage, 1, // histogram from 1 image only
				channels, cv::Mat(), // no mask is used
				hist, // the output histogram
				1, // 1D histogram
				bins, ranges // pixel value ranges
				);

		// Get min and max bin values
		double maxVal = 0;
		double minVal = 0;
		cv::minMaxLoc(hist, &minVal, &maxVal, 0, 0);
		// The image to display the histogram
		cv::Mat histImg(bins[0], bins[0], CV_8U, cv::Scalar(255));

		// Map the highest point to 95% of the histogram height to leave some
		// empty space at the top
		const int histHeight = bins[0];
		const int maxHeight = 0.95 * histHeight;

		cv::Mat_<float>::iterator it = hist.begin<float>();
		cv::Mat_<float>::iterator itend = hist.end<float>();

		int barPosition = 0;
		for (; it != itend; ++it) {
			float histValue = (*it);
			int barHeight = (histValue * maxHeight) / maxVal;
			cv::line(histImg,
					// start the line from the bottom, and go up based on the barHeight
					// Remember the (0,0) is the top left corner
					cv::Point(barPosition, histHeight),
					cv::Point(barPosition, histHeight - barHeight),
					cv::Scalar::all(0));
			barPosition++;
		}
		return histImg;
	}

	void VisionController::dump(const Mat &mat, const char* fname) {
		ofstream filestream;
		filestream.open(fname);
		filestream << mat << endl << endl;
		filestream.close();
	}

	void VisionController::showHSVHistograms(Mat image) {

		vector<Mat> channels;

		//cvtColor(image, image_histo, COLOR_RGB2HSV);
		image_histo = image;
		split(image_histo, channels);

		// And then if you like
		image_H = channels[0];
		image_S = channels[1];
		image_V = channels[2];

		// Calculate histograms of image
		//histogram_H = showHistogram2(image_H);
		histogram_H = showHistogram4(image_H, 180, 0);
		threshold(image_H, image_HThresh, 0, 255, THRESH_OTSU | THRESH_BINARY);

		//histogram_S = showHistogram3(image_S);
		histogram_S = showHistogram4(image_S, 255, 0);
		threshold(image_S, image_SThresh, 0, 255, THRESH_OTSU | THRESH_BINARY);

		histogram_V = showHistogram4(image_V, 255, 0);
		threshold(image_V, image_VThresh, 0, 255, THRESH_OTSU | THRESH_BINARY);

		Mat inGreenRange;
		Mat testRGB;
		//void inRange(InputArray src, InputArray lowerb, InputArray upperb, OutputArray dst)
		//cout<<image_H<<endl;

		inRange(image_H, lowerBound,upperBound,image_thresholded);


		//Split image into RGB
		vector<Mat> RGBchannels;
		Mat image_BnoG;
		//cvtColor(image, image_HSV,CV_RGB2HSV);
		//inRange(image_H,50, 70, testRGB);
		split(image, RGBchannels);
		image_R = RGBchannels[0];
		image_G = RGBchannels[1];
		image_B = RGBchannels[2];
		subtract(image_R, image_G, image_noG);
		subtract(image_B, image_G, image_BnoG);
		add(image_noG,image_BnoG, image_noG);
		//threshold(image_noG, image_thresholded, 7, max_BINARY_value, THRESH_BINARY);

		//merge(RGBchannels, image_noG);

	}


	cv::Mat VisionController::showHistogram2(const cv::Mat &inImage) {

		cv::MatND hist;
		// For a gray scale [0:255] we have 256 bins
		const int bins[1] = { 180 };
		const float hranges[2] = { 0.0, 180.0 };
		const float* ranges[1] = { hranges };
		const int channels[1] = { 0 };

		cv::calcHist(&inImage, 1, // histogram from 1 image only
				channels, cv::Mat(), // no mask is used
				hist, // the output histogram
				1, // 1D histogram
				bins, ranges // pixel value ranges
				);

		// Get min and max bin values
		double maxVal = 0;
		double minVal = 0;
		cv::minMaxLoc(hist, &minVal, &maxVal, 0, 0);
		// The image to display the histogram
		cv::Mat histImg(bins[0], bins[0], CV_8U, cv::Scalar(180));

		// Map the highest point to 95% of the histogram height to leave some
		// empty space at the top
		const int histHeight = bins[0];
		const int maxHeight = 0.95 * histHeight;

		cv::Mat_<float>::iterator it = hist.begin<float>();
		cv::Mat_<float>::iterator itend = hist.end<float>();

		int barPosition = 0;
		for (; it != itend; ++it) {
			float histValue = (*it);
			int barHeight = (histValue * maxHeight) / maxVal;
			cv::line(histImg,
					// start the line from the bottom, and go up based on the barHeight
					// Remember the (0,0) is the top left corner
					cv::Point(barPosition, histHeight),
					cv::Point(barPosition, histHeight - barHeight),
					cv::Scalar::all(0));
			barPosition++;
		}
		return histImg;
	}

	cv::Mat VisionController::showHistogram3(const cv::Mat &inImage) {

		cv::MatND hist;
		// For a gray scale [0:255] we have 256 bins
		const int bins[1] = { 256 };
		const float hranges[2] = { 0.0, 255.0 };
		const float* ranges[1] = { hranges };
		const int channels[1] = { 0 };

		cv::calcHist(&inImage, 1, // histogram from 1 image only
				channels, cv::Mat(), // no mask is used
				hist, // the output histogram
				1, // 1D histogram
				bins, ranges // pixel value ranges
				);

		// Get min and max bin values
		double maxVal = 0;
		double minVal = 0;
		cv::minMaxLoc(hist, &minVal, &maxVal, 0, 0);
		// The image to display the histogram
		cv::Mat histImg(bins[0], bins[0], CV_8U, cv::Scalar(255));

		// Map the highest point to 95% of the histogram height to leave some
		// empty space at the top
		const int histHeight = bins[0];
		const int maxHeight = 0.95 * histHeight;

		cv::Mat_<float>::iterator it = hist.begin<float>();
		cv::Mat_<float>::iterator itend = hist.end<float>();

		int barPosition = 0;
		for (; it != itend; ++it) {
			float histValue = (*it);
			int barHeight = (histValue * maxHeight) / maxVal;
			cv::line(histImg,
					// start the line from the bottom, and go up based on the barHeight
					// Remember the (0,0) is the top left corner
					cv::Point(barPosition, histHeight),
					cv::Point(barPosition, histHeight - barHeight),
					cv::Scalar::all(0));
			barPosition++;
		}
		return histImg;
	}

	cv::Mat VisionController::showHistogram4(const Mat &inImage, int numBins, int numChannels) {

		cv::MatND hist;
		// For a gray scale [0:255] we have 256 bins
		const int bins[1] = { numBins };
		const float hranges[2] = { 0.0, numBins / 1.0 };
		const float* ranges[1] = { hranges };
		const int channels[1] = { numChannels };

		cv::calcHist(&inImage, 1, // histogram from 1 image only
				channels, cv::Mat(), // no mask is used
				hist, // the output histogram
				1, // 1D histogram
				bins, ranges // pixel value ranges
				);

		// Get min and max bin values
		double maxVal = 0;
		double minVal = 0;
		cv::minMaxLoc(hist, &minVal, &maxVal, 0, 0);
		// The image to display the histogram
		cv::Mat histImg(bins[0], bins[0], CV_8U, cv::Scalar(255));

		// Map the highest point to 95% of the histogram height to leave some
		// empty space at the top
		const int histHeight = bins[0];
		const int maxHeight = 0.95 * histHeight;

		cv::Mat_<float>::iterator it = hist.begin<float>();
		cv::Mat_<float>::iterator itend = hist.end<float>();

		int barPosition = 0;
		for (; it != itend; ++it) {
			float histValue = (*it);
			int barHeight = (histValue * maxHeight) / maxVal;
			cv::line(histImg,
					// start the line from the bottom, and go up based on the barHeight
					// Remember the (0,0) is the top left corner
					cv::Point(barPosition, histHeight),
					cv::Point(barPosition, histHeight - barHeight),
					cv::Scalar::all(0));
			barPosition++;
		}
		return histImg;
	}


float VisionController::chiSquared(Mat Xchecked1,Mat Ychecked1, float yIntercept1,float slope1,int X1Count, int num)
{
	 float chisquared = 0;
			 float deltax = 0;
			 for (int a = 0; a<X1Count; a++)
			 {
				 deltax = deltax + pow( (Xchecked1.at<float>(a,0) - (Ychecked1.at<float>(a,0)-yIntercept1)/slope1),2);
			 }
			 float sigma = deltax/X1Count;
			 chisquared = deltax/(X1Count * pow(sigma,2));
			 cout << "chisquared"<<num<<":" <<  1/chisquared <<endl;
			 return 1/chisquared;
}

void VisionController::simpleDir()
{
	// check across middle

	// take average o
}

}

