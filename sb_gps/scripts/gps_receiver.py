#!/usr/bin/python

#Author: Vincent Yuan 
#Purpose: serial adapter, because c++ is too hard to serial communication 
#Date: May 17, 2015
#Reminder: chmod +x gps_receiver.py 
#Run: rosrun sb_gps gps_ receiver.py 
from sb_gps.srv import *
import serial
import rospy
from std_msgs.msg import String
import math

def handle_gps_service(req,res):
	print "Calculating distance between lon: %d , lat: %d and lon: %d , lat: %d"%(req.lon1, req.lat1,req.lon2,req.lat2)
	CurrentWaypoint  = [req.lon1,req.lat1]; 
	TargetWaypoint = [req.lon2,req.lat2];
	print CurrentWaypoint
	toRad = 3.14169365/180
	lat1 = CurrentWaypoint[0] * toRad;
	lon1 = CurrentWaypoint[1] * toRad;
	lat2 = TargetWaypoint[0] * toRad;
	lon2 = TargetWaypoint[1] * toRad;

	a = math.sin((lat2 - lat1) / 2)*math.sin((lat2 - lat1) / 2) + math.cos(lat1) * math.cos(lat2) * math.sin((lon2 - lon1) / 2)*math.sin((lon2 - lon1) / 2);
	c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a));
	res.distance = 6371000 * c;
	

	
def main (): 
	pub = rospy.Publisher ('GPS_USB_DATA', String, queue_size=10)
	serv = rospy.Service('GPS_SERVICE', Gps_Service, handle_gps_service)
	rospy.init_node('sb_gps')
	rate = rospy.Rate(10) # 10HZ
	link = serial.Serial(port="/dev/ttyUSB0",baudrate=1000000)
	while not rospy.is_shutdown():
		rospy.loginfo(link.readline())
		pub.publish(link.readline())
		rate.sleep()

if __name__ == '__main__':
	try: 
		main()
	except rospy.ROSInterruptException:
		pass

