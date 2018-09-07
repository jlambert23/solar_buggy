#!/usr/bin/env python

import rospy
from solar_buggy.msg import Ultrasonic
from solar_buggy.srv import Controller

def all_clear(sensors_clear):
    return sensors_clear.left and sensors_clear.right and sensors_clear.frontleft and sensors_clear.frontright

def move_vehicle(sensors_clear):
    controller = rospy.ServiceProxy('controller', Controller)

    if all_clear(sensors_clear):
        controller('half_speed')

    elif sensors_clear.right and not sensors_clear.left:
        controller('veer_right')

    elif sensors_clear.left and not sensors_clear.right:
        controller('veer_left')

    else:
        controller('stop')

if __name__ == '__main__':
    rospy.init_node('move_vehicle', anonymous=True)
    rospy.Subscriber('sensors', Ultrasonic, move_vehicle)
    rospy.spin()