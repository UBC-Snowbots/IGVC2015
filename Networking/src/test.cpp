/**
 * UBC SnowBots
 * 
 * test.cpp
 * 	this is a simple internal networking test - it publishes its output under the name "chatter", and spews content until killed
 * 
 * */

#include "ros/ros.h"
//#include "std_msgs/String.h"
#include "test/Test.h"

#include <sstream>

int main(int argc,char** argv){
        ros::init(argc, argv, "test_node");
        ros::NodeHandle n;
        
        ros::Publisher chatter_pub = n.advertise<test::Test>("chatter", 1000);
        
        ros::Rate loop_rate(10);
        
        int count = 0;
        
        while(ros::ok()){
                test::Test msg;
                std::stringstream ss;
                ss << "hello world" << count;
                msg.test = ss.str();
                
                ROS_INFO("%s", msg.test.c_str());
                
                chatter_pub.publish(msg);
                
                ros::spinOnce();
                
                loop_rate.sleep();
                
                ++count;
        }
        return 0;
}
