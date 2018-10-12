#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from solar_buggy.msg import Ultrasonic, Pose
from get_config import config

cmd_pub = rospy.Publisher('ultra_vel', Pose, queue_size=10)

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
    
    if ultra.front < 30.0:
        sensor_info |= 0b0001
    if ultra.left < 30.0:
        sensor_info |= 0b0010
    if ultra.right < 30.0:
        sensor_info |= 0b0100
    #if ultra.back < 30.0:
    #    sensor_info |= 0b1000
    #print sensor_info
    #print ultra

    pose = Pose()

    if (sensor_info == 7):
        # reverse full speed
        pose.linear_velocity = -64
    elif (sensor_info == 5): 
        # turn left
        pose.angular_velocity = -64
    elif (sensor_info == 3): 
        # turn right
        pose.angular_velocity = 64
    elif (sensor_info == 6): 
        # full speed
        pose.linear_velocity = 64
    elif (sensor_info == 4): 
        # veer left; not applicable with current setup
        pose.angular_velocity = -32
        pose.linear_velocity = 32
    elif (sensor_info == 2): 
        # veer right; not applicable with current setup
        pose.angular_velocity = 32
        pose.linear_velocity = 32

    # Should reconfigure this to maybe incorporate info from other sensors
    elif (sensor_info == 1): 
        # turn right
        pose.angular_velocity = 64
    else: 
        # full speed
        pose.linear_velocity = 64
    
    cmd_pub.publish(pose)

if __name__ == '__main__':
    rospy.init_node('ultrasonic', anonymous=True)
    rospy.Subscriber('ultrasonic', Ultrasonic, handler)
    rospy.spin()
