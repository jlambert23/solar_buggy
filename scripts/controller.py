#!/usr/bin/env python
import rospy
import serial
from solar_buggy.msg import Pose

# Serial port for sensors and motors
serialMotors  = serial.Serial('/dev/ttySAC0', 9600, timeout = 10)

left_offset = 2
right_offset = 0
offset = max(left_offset, right_offset)

motor = {}
motor['left'] = {
    'reverse': [1, 63],
    'stop': 64,
    'forward': [65, 127],
}
motor['right'] = {
    'reverse': [128, 191],
    'stop': 192,
    'forward': [193, 255]
}

def controller(pose):
    
    '''
    if pose.angular_velocity > 63:
        pose.angular_velocity = 63
    elif pose.angular_velocity < -63:
        pose.angular_velocity = -63
    if pose.linear_velocity > 63:
        pose.linear_velocity = 63
    elif pose.linear_velocity < -63:
        pose.linear_velocity = -63

    # rotate clockwise
    if pose.angular_velocity > 0:
        left_adjust = min(pose.linear_velocity + pose.angular_velocity, 63)
        right_adjust = pose.linear_velocity

    # rotate counter-clockwise
    elif pose.angular_velocity < 0:
        left_adjust = pose.linear_velocity
        right_adjust = min(pose.linear_velocity - pose.angular_velocity, 63)

    # go straight
    else:
        left_adjust = pose.linear_velocity
        right_adjust = pose.linear_velocity
    '''
    
    if pose.left_wheel_velocity > 63 - left_offset:
        pose.left_wheel_velocity = 63 - left_offset
    elif pose.left_wheel_velocity < -63 + left_offset:
        pose.left_wheel_velocity = -63 + left_offset

    if pose.right_wheel_velocity > 63 - right_offset:
        pose.right_wheel_velocity = 63 - right_offset
    elif pose.right_wheel_velocity < -63 + right_offset:
        pose.right_wheel_velocity = -63 + right_offset

    serialMotors.write(chr(int(motor['left']['stop'] + pose.left_wheel_velocity + left_offset)))
    serialMotors.write(chr(int(motor['right']['stop'] + pose.right_wheel_velocity))) 

def stop():
    rospy.loginfo('Stop')
    serialMotors.write(chr(int(motor['left']['stop'])))
    serialMotors.write(chr(int(motor['right']['stop'])))

if __name__ == '__main__':
    rospy.init_node('controller')
    rospy.Subscriber('cmd_vel', Pose, controller)

    while not rospy.is_shutdown():
        rospy.sleep(1.0)

    stop()
