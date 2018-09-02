#!/usr/bin/env python

import serial
import rospy
from std_msgs.msg import String

serU = serial.Serial('/dev/ttyACM0', 9600)

def ultra():


    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        if serU.in_waiting > 0:
            data = serU.readline()

            port = data[0:4].strip('\r\n')
            distance = data[5:].strip('\r\n')

            pub = rospy.Publisher('ultrasonic_' + port, String, queue_size=10)
            pub.publish(distance)

            #rospy.loginfo(data)
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('read_serial', anonymous=True)
    
    try:
        ultra()
    except rospy.ROSInterruptException:
        pass
