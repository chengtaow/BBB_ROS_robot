#!/usr/bin/env python
# Chengtao Wang, 09/22/2016
import Adafruit_BBIO.GPIO as GPIO
import Adafruit_BBIO.PWM as PWM
import rospy
from std_msgs.msg import Int64

GPIO.setup("P9_12", GPIO.OUT)
GPIO.setup("P9_23", GPIO.OUT)
GPIO.setup("P9_30", GPIO.IN)
GPIO.setup("P9_27", GPIO.IN)
GPIO.output("P9_23", GPIO.HIGH)
global distance
distance = 0


def encoder_go1(channel):
	global distance
	distance += 1

def encoder_go2(channel):
	global distance
	distance += 1

GPIO.add_event_detect("P9_30", GPIO.BOTH, callback = encoder_go1)
GPIO.add_event_detect("P9_27", GPIO.BOTH, callback = encoder_go2)

def send_encoder():
	global distance
	pub = rospy.Publisher('l_en_num', Int64, queue_size=20)
	rospy.init_node('laser_motor', anonymous=True)
	rate = rospy.Rate(100)
	PWM.start("P9_16", 20, 50, 0)
	GPIO.output("P9_12", GPIO.LOW)
	while not rospy.is_shutdown():
		msg = Int64()
		msg.data = distance
		#int64(encoder_distance = distance)
		rospy.loginfo(msg.data)
		pub.publish(msg.data)
		rate.sleep()

if __name__ == '__main__':
	try:
		send_encoder()
	except rospy.ROSInterruptException:
		pass
		
