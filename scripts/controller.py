#!/usr/bin/env python
import rospy
import serial
from solar_buggy.msg import Pose

# Serial port for sensors and motors
serialMotors  = serial.Serial('/dev/ttySAC0', 9600, timeout = 10)

motor = {}
motor['left'] = {
    'reverse': [1, 63],
    'stop': 64
    'forward': [65, 127],
}
motor['right'] = {
    'reverse': [128, 191],
    'stop': 192,
    'forward': [193, 255]
}

def controller(pose):
    # angular velocity and linear velocity do not co-exist at this time.
    # meaning, the vehicle does not turn and move forward at the same time.
    
    if pose.angular_velocity > 0:
        # rotate clockwise
        if pose.angular_velocity > 63:
            pose.angular_velocity = 63

        serialMotors.write(chr(motor['left']['stop'] + pose.angular_velocity))
        serialMotors.write(chr(motor['right']['stop']))

    elif pose.angular_velocity < 0:
        # rotate counter-clockwise
        if pose.angular_velocity < -63:
            pose.angular_velocity = -63

        serialMotors.write(chr(motor['right']['stop'] - pose.angular_velocity))
        serialMotors.write(chr(motor['left']['stop']))

    else:
        # go straight
        if pose.linear_velocity > 63:
            pose.linear_velocity = 63
        elif pose.linear_velocity < 0:
            pose.linear_velocity = 0

        serialMotors.write(chr(motor['right']['stop'] + linear_velocity))
        serialMotors.write(chr(motor['left']['stop'] + linear_velocity))
    
def stop():
    serialMotors.write(chr(motor['right']['stop']))
    serialMotors.write(chr(motor['left']['stop']))

if __name__ == '__main__':
    rospy.init_node('controller')
    rospy.Subscriber('cmd_vel', Pose, controller)

    while not rospy.is_shutdown():
        rospy.sleep(1.0)
    
    stop()