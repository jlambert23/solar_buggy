#!/usr/bin/env python

import serial, json, rospy
from solar_buggy.msg import Ultrasonic
from geographic_msgs.msg import GeoPoint
from std_msgs.msg import String

serU = serial.Serial('/dev/ttyACM0', 9600)
# serU = serial.Serial('/dev/ttyS3', 9600)

input_sample = '{"ultrasonic":{"0":48,"1":69,"2":255,"3":250},"gps":{"destination": {"distance": 253, "bearing": 143.28}, "latitude": 20.17383, "longitude": -8.21838}}'

gps_pub = rospy.Publisher('gps', GeoPoint, queue_size=10)
ultra_pub = rospy.Publisher('ultrasonic', Ultrasonic, queue_size=10)

def read_serial():
    # rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        if serU.in_waiting == 0:
            continue

        try:
            data = json.loads(serU.readline())
            # data = json.loads(input_sample)
            

            distance = data['gps']['destination']['distance']
            bearing = data['gps'] ['destination']['bearing']
            
            gps_pub.publish(
                latitude = data['gps']['latitude'],
                longitude = data['gps']['longitude']
            )

            left_sensor = data['ultrasonic']['0']
            right_sensor = data['ultrasonic']['1']
            front_sensor = data['ultrasonic']['2']
            back_sensor = data['ultrasonic']['3']

            ultra_pub.publish(
                left = data['ultrasonic']['0'],
                right = data['ultrasonic']['1'],
                front = data['ultrasonic']['2'],
                back = data['ultrasonic']['3']
            )

        except ValueError:
            continue

        # rate.sleep()


if __name__ == '__main__':
    rospy.init_node('read_serial', anonymous=True)
    read_serial()
