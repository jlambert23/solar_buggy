#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from solar_buggy.msg import Ultrasonic
from get_config import config

pub = rospy.Publisher('ultra_cmd', String, queue_size=10)

sensors_clear = {
    'left': True,
    'front': True,
    'right': True,
    'back': True
}

'''
bit string order (MSB -> LSB): back, right, left, front
'''

def handler(ultra):
    sensor_info = 0b0
    command = ''
    
    if ultra.front < 30.0:
        sensor_info |= 0b0001
    if ultra.left < 30.0:
        sensor_info |= 0b0010
    if ultra.right < 30.0:
        sensor_info |= 0b0100
    if ultra.back < 30.0:
        sensor_info |= 0b1000
    print sensor_info
    if (sensor_info == 7):
        command = 'reverse_full_speed'
    elif (sensor_info == 5): 
        command = 'left'
    elif (sensor_info == 3): 
        command = 'right'
    elif (sensor_info == 6): 
        command = 'full_speed'
    elif (sensor_info == 4): 
        command = 'veer_left'
    elif (sensor_info == 2): 
        command = 'veer_right'
        # Should reconfigure this to maybe incorporate info from other sensors
    elif (sensor_info == 1): 
        command = 'veer_left'
    else: 
        command = 'full_speed'
    
    pub.publish(command)     

if __name__ == '__main__':
    rospy.init_node('ultrasonic', anonymous=True)
    rospy.Subscriber('ultrasonic', Ultrasonic, handler)
    rospy.spin()
