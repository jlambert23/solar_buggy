#!/usr/bin/env python

import serial
import rospy
from std_msgs.msg import String

serU = serial.Serial('/dev/ttyACM0', 9600)

def publish_sensor_data(data):
    port = data.data[0:4].strip('\r\n')
    distance = data.data[5:].strip('\r\n')
        
    pub = rospy.Publisher('ultrasonic_' + port, String, queue_size=10)
    pub.publish(distance)

def ultra():
    pub = rospy.Publisher('ultrasonic_1010', String, queue_size=10)

    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        if serU.in_waiting > 0:
        	dist = serU.readline()

        	#rospy.loginfo(dist)
        	pub.publish(dist)
        	rate.sleep()

# Currently using pubsub for simulated serial data. Will replace as arduino uno is implemented.
def read_serial():
    rospy.Subscriber('ultrasonic', String, publish_sensor_data)
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('read_serial', anonymous=True)
    
    try:
        ultra()
    except rospy.ROSInterruptException:
        pass
