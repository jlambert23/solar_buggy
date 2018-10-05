#!/usr/bin/env python

import rospy
from solar_buggy.srv import Controller
from std_msgs.msg import String

controller = rospy.ServiceProxy('controller', Controller)

def ultra_handler(command):
    print(command.data)
    controller(command.data)

def gps_handler(command):
    if (command.data == 'destination_reached'):
        print('Not today ISIS!!')
        controller('stop')

    if (command.data != 'turn_around'):
        controller(command.data)

if __name__ == '__main__':
    rospy.init_node('priority', anonymous=True)
    #rospy.Subscriber('ultra_cmd', String, ultra_handler)
    rospy.Subscriber('gps_cmd', String, gps_handler)

    rospy.spin()

