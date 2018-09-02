#!/usr/bin/env python

import serial
import rospy
from get_config import config
from std_msgs.msg import String

serU = serial.Serial('/dev/ttyACM0', 9600)
ports = config['ports']


def ultra():
    rate = rospy.Rate(10) # 10hz

    pub = {
        ports['left']: rospy.Publisher('ultrasonic_' + ports['left'], String, queue_size=10),
        ports['front_left']: rospy.Publisher('ultrasonic_' + ports['front_left'], String, queue_size=10),
        ports['front_right']: rospy.Publisher('ultrasonic_' + ports['front_right'], String, queue_size=10),
        ports['right']: rospy.Publisher('ultrasonic_' + ports['right'], String, queue_size=10)
    }
    
    while not rospy.is_shutdown():
        if serU.in_waiting > 0:
            data = serU.readline()

            port = data[0:4].strip('\r\n')
            distance = data[5:].strip('\r\n')

            pub[port].publish(distance)

            #rospy.loginfo(data)
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('read_serial', anonymous=True)
    
    try:
        ultra()
    except rospy.ROSInterruptException:
        pass
