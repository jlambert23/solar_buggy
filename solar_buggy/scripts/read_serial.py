#!/usr/bin/env python

import serial, json, rospy
from solar_buggy.msg import Ultrasonic, GPS
from std_msgs.msg import String

serU = serial.Serial('/dev/ttyACM0', 9600)

# input_sample = '{"ultrasonic":{"left":48,"right":69,"front_left":255,"front_right":250},"gps":{"velocity_north":2.7,"velocity_east":3.8,"dest_distance":743,"dest_bearing":1.73}}'

def read_serial():
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        if serU.in_waiting >= 0:
            continue

        try:
            data = json.loads(serU.readline())
            # data = json.loads(input_sample)

            ultra = data['ultrasonic']
            gps = data['gps']
            
        except Exception as e:
            raise Exception("Something happened while unpacking serial as json: " + str(e))
            
        ultra_pub = rospy.Publisher('ultra_serial', Ultrasonic, queue_size=10)
        ultra_pub.publish(left = ultra['left'], 
                          right = ultra['right'], 
                          frontleft = ultra['front_left'], 
                          frontright = ultra['front_right'])

        gps_pub = rospy.Publisher('gps_serial', GPS, queue_size=10)
        gps_pub.publish(velocity_east = gps['velocity_east'], 
                        velocity_north = gps['velocity_north'], 
                        dest_distance = gps['dest_distance'], 
                        dest_bearing = gps['dest_bearing'])

        rate.sleep()


if __name__ == '__main__':
    rospy.init_node('read_serial', anonymous=True)
    read_serial()
