#!/usr/bin/env python
# Chengtao Wang, 09/22/2016
import rospy
from std_msgs.msg import Int64

def callback(data):
	rospy.loginfo(rospy.get_caller_id() + " I heard %d", data.data)

def process():
	rospy.init_node('laser_rec', anonymous=True)
	rospy.Subscriber('l_en_num', Int64, callback)
	rospy.spin()

if __name__ == '__main__':
	process()
