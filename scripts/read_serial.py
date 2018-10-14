#!/usr/bin/env python

import serial, json, rospy
from solar_buggy.msg import Ultrasonic, RelationalWayPoint, Pose
from geographic_msgs.msg import GeoPoint
from std_msgs.msg import String

serU = serial.Serial('/dev/ttyACM0', 9600)
# serU = serial.Serial('/dev/ttyS3', 9600)

gps_pub = rospy.Publisher('gps', Pose, queue_size=10)
wp_pub = rospy.Publisher('waypoint', RelationalWayPoint, queue_size=10)
ultra_pub = rospy.Publisher('ultrasonic', Ultrasonic, queue_size=10)

# Definitely temporary...
def get_sample_data():
    import os

    script_dir = os.path.dirname(__file__)
    file_path = os.path.join(script_dir, '../testing/arduino.json')

    with open(file_path, 'r') as f:
        data = json.load(f)
    
    return data


def read_serial():
    while not rospy.is_shutdown():
        if serU.in_waiting == 0:
            continue

        try:
            data = json.loads(serU.readline())
            #data = get_sample_data()
            
            #coordinates = GeoPoint()
            #coordinates.latitude = data['gps']['latitude']
            #coordinates.longitude = data['gps']['longitude']

            # Publishing gps data.
            #gps_pub.publish(
            #    coordinates = coordinates
                #bearing = data['gps']['IMU']
            #)

            # Publishing waypoint data.
            #wp_pub.publish(
            #    distance = data['gps']['waypoint']['distance'],
            #    bearing = data['gps']['waypoint']['bearing']
            #)

            # Publishing ultrasonic data.
            ultra_pub.publish(
                back = data['ultrasonic']['0'],
                right = data['ultrasonic']['1'],
                front = data['ultrasonic']['2'],
                left = data['ultrasonic']['3'],
            )

        except ValueError:
            continue

        #rate = rospy.Rate(1)
        #rate.sleep()


if __name__ == '__main__':
    rospy.init_node('read_serial', anonymous=True)
    read_serial()
