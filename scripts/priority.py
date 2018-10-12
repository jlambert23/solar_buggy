#!/usr/bin/env python

import rospy
from solar_buggy.msg import Pose
from solar_buggy.srv import Controller
from std_msgs.msg import String

control_pub = rospy.Publisher('cmd_vel', Pose, queue_size=10)

def ultra_handler(pose):
    #print(pose)
    control_pub.publish(pose)

def gps_handler(command):
    if (command.data == 'destination_reached'):
        rospy.loginfo('Not today ISIS!!')
        #controller('stop')

    #if (command.data != 'turn_around'):
        #controller(command.data)

if __name__ == '__main__':
    rospy.init_node('priority', anonymous=True)
    rospy.Subscriber('ultra_vel', Pose, ultra_handler)
    #rospy.Subscriber('gps_vel', Pose, gps_handler)

    rospy.spin()

