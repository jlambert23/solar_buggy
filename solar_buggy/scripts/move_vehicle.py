#!/usr/bin/env python

import rospy
from std_msgs.msg import Int8

left_sensor        = False
front_left_sensor  = False
front_right_sensor = False
right_sensor       = False

signal_go          = True
start_up           = True
side_check         = False
looking_from_left  = False
looking_from_right = False

def set_left_sensor(data):
    left_sensor = data.data
    print('left: ' + str(left_sensor))

def set_front_left_sensor(data):
    front_left_sensor = data.data
    print('front_left: ' + str(front_left_sensor))

def set_front_right_sensor(data):
    front_right_sensor = data.data
    print('front_right: ' + str(front_right_sensor))

def set_right_sensor(data):
    right_sensor = data.data
    print('right: ' + str(right_sensor))

if __name__ == '__main__':
    rospy.init_node('move_vehicle', anonymous=True)
    rospy.Subscriber('left_sensor', Int8, set_left_sensor)
    rospy.Subscriber('front_left_sensor', Int8, set_front_left_sensor)
    rospy.Subscriber('front_right_sensor', Int8, set_front_right_sensor)
    rospy.Subscriber('right_sensor', Int8, set_right_sensor)
    rospy.spin()