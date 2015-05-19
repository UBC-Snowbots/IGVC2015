#!usr/bin/python2.7

#Author: Vincent Yuan 
#Purpose: serial adapter, because c++ is too hard to serial communication 
#Date: May 17, 2015

import serial
import rospy
from std_msgs.msg import String

pub = rospy.Publisher ('GPS_USB_DATA', String, queue_size=10)
rospy.init_node('sb_gps')
rate = rospy.Rate(10) # 10HZ
link = serial.Serial(port="/dev/ttyUSB0",baudrate=1000000)


while not rospy.is_shutdown():
	rospy.loginfo(link.readline())
	pub.publish(link.readline())
	rate.sleep()

if __name__ == '__main__':
	try: 
		talker()
	except rospy.ROSInterruptException:
		pass

