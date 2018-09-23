#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from solar_buggy.msg import Ultrasonic
from get_config import config

pub = rospy.Publisher('ultra_cmd', String, queue_size=10)

sensors_clear = {
    'left': True,
    'front_left': True,
    'front_right': True,
    'right': True
}
    
def handler(ultra):
    sensors_clear['left'] = ultra.left > 30.0
    sensors_clear['front_left'] = ultra.frontleft > 30.0
    sensors_clear['front_right'] = ultra.frontright > 30.0
    sensors_clear['right'] = ultra.left > 30.0
    
    if not all(sensors_clear.values()):
        pub.publish('stop')
    else:
        pub.publish('sensors_clear')

if __name__ == '__main__':
    rospy.init_node('ultrasonic', anonymous=True)
    rospy.Subscriber('ultra_serial', Ultrasonic, handler)
    rospy.spin()