#!/usr/bin/env python

import rospy
from solar_buggy.srv import Controller

try:
    controller = rospy.ServiceProxy('control_stop', Controller)
    controller()
    
    controller = rospy.ServiceProxy('control_left', Controller)
    controller()

    controller = rospy.ServiceProxy('control_veer_left', Controller)
    controller()

    controller = rospy.ServiceProxy('control_right', Controller)
    controller()

    controller = rospy.ServiceProxy('control_veer_right', Controller)
    controller()

    controller = rospy.ServiceProxy('control_full_speed', Controller)
    controller()

    controller = rospy.ServiceProxy('control_half_speed', Controller)
    controller()

    controller = rospy.ServiceProxy('control_reverse_full_speed', Controller)
    controller()

    controller = rospy.ServiceProxy('control_reverse_half_speed', Controller)
    controller()

    controller = rospy.ServiceProxy('control_rotate_left_half_speed', Controller)
    controller()

    controller = rospy.ServiceProxy('control_rotate_right_half_speed', Controller)
    controller()

except rospy.ServiceException, e:
        print "Service call failed: %s"%e