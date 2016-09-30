#!/usr/bin/env python
# Chengtao Wang, 09/22/2016
import rospy
from std_msgs.msg import Int64
from laser.msg import LaserData

def count_callback(counts):
	rospy.loginfo(rospy.get_caller_id() + "  The motor counts is: %d ", counts.data)

def laser_callback(laserdata):
	rospy.loginfo(rospy.get_caller_id() + "  The distance is: %d, the status is: %-10s", laserdata.dist, laserdata.status)

def process():
	rospy.init_node('laser_rec', anonymous=True)
	rospy.Subscriber('l_en_num', Int64, count_callback)
	rospy.Subscriber('l_data', LaserData, laser_callback)
	rospy.spin()

if __name__ == '__main__':
	process()
