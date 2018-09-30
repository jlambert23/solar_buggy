#!/usr/bin/env python
import rospy
import serial
from get_config import config
from solar_buggy.srv import Controller

# Serial port for sensors and motors
serialMotors  = serial.Serial('/dev/ttySAC0', 9600, timeout = 10)

control = config['motor_controls']

def controller_server():
    rospy.init_node('controller')
    rospy.Service('controller', Controller, control_handler)

    while not rospy.is_shutdown():
        rospy.sleep(1.0)

    stop()

def control_handler(req):
    rospy.loginfo(req.command)

    if req.command == 'stop':
        stop()
    elif req.command == 'full_speed':
        full_speed()
    elif req.command == 'half_speed':
        half_speed()
    elif req.command == 'left':
        left()
    elif req.command == 'right':
        right()
    elif req.command == 'veer_left':
        veer_left()
    elif req.command == 'veer_right':
        veer_right()
    elif req.command == 'reverse_full_speed':
        reverse_full_speed()
    elif req.command == 'reverse_half_speed':
        reverse_half_speed()
    elif req.command == 'rotate_left_half_speed':
        rotate_left_half_speed()
    elif req.command == 'rotate_right_half_speed':
        rotate_right_half_speed()
    
    return True

# Command to stop the vehicle
def stop():
    serialMotors.write(chr(control['all_stop']))
    rospy.loginfo("STOP!")
    return True

# Command to move vehicle forward at full speed
def full_speed():
    serialMotors.write(chr(control['right']['full_speed']))
    serialMotors.write(chr(control['left']['full_speed']))
    rospy.loginfo("GO!")
    return True

# Command vehicle to slow down forward movement
def half_speed():
    serialMotors.write(chr(control['right']['half_speed']))
    serialMotors.write(chr(control['left']['half_speed']))
    rospy.loginfo("Slow Down!")
    return True

# Command vehicle to turn left
def left():
    serialMotors.write(chr(control['right']['full_speed']))
    serialMotors.write(chr(control['left']['stop']))
    rospy.loginfo("LEFT!")
    return True

# Command vehicle to turn right
def right():
    serialMotors.write(chr(control['right']['stop']))
    serialMotors.write(chr(control['left']['full_speed']))
    rospy.loginfo("RIGHT!")
    return True

# Command vehicle to turn slightly to the left
def veer_left():
    serialMotors.write(chr(control['right']['full_speed']))
    serialMotors.write(chr(control['left']['half_speed']))
    rospy.loginfo("Veer Left!")
    return True

# Command vehicle to turn slightly to the right
def veer_right():
    serialMotors.write(chr(control['right']['half_speed']))
    serialMotors.write(chr(control['left']['full_speed']))
    rospy.loginfo("Veer Right!")
    return True

# Check if there are objects detected from side sensors
def reverse_full_speed():
    serialMotors.write(chr(control['right']['rev_full_speed']))
    serialMotors.write(chr(control['left']['rev_full_speed']))
    rospy.loginfo("Reverse!")
    return True

# Check if there are objects detected from side sensors
def reverse_half_speed():
    serialMotors.write(chr(control['right']['rev_half_speed']))
    serialMotors.write(chr(control['left']['rev_half_speed']))
    rospy.loginfo("Reverse Half Speed!")
    return True

# Command vehicle to rotate at half speed to the left
def rotate_left_half_speed():
    serialMotors.write(chr(control['right']['half_speed']))
    serialMotors.write(chr(control['left']['rev_half_speed']))
    rospy.loginfo("Rotate Left!")
    return True

# Command vehicle to rotate at half speed to the right
def rotate_right_half_speed():
    serialMotors.write(chr(control['right']['rev_half_speed']))
    serialMotors.write(chr(control['left']['half_speed']))
    rospy.loginfo("Rotate Right!")
    return True

if __name__ == '__main__':
    controller_server()
