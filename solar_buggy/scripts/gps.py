#!/usr/bin/env python

import rospy
import math

from solar_buggy.msg import GPS
from std_msgs.msg import String

pub = rospy.Publisher('gps_cmd', String, queue_size=10)

def handler(data):
    if data.dest_distance > 0:
        if data.dest_bearing == math.pi / 2:
            pub.publish('east')
        elif data.dest_bearing == 3 * math.pi / 2:
            pub.publish('west')
        else:
            buggy_bearing = 1 / math.tan(data.velocity_east / data.velocity_north)

            if buggy_bearing > data.dest_bearing:
                pub.publish('left')
            elif buggy_bearing < data.dest_bearing:
                pub.publish('right')
            else:
                pub.publish('straight')
    else:
        pub.publish('destination_reached')

if __name__ == '__main__':
    rospy.init_node('gps', anonymous=True)
    rospy.Subscriber('gps_serial', GPS, handler)
    rospy.spin()