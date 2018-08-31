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
    rospy.Service('control_stop', Controller, stop)
    rospy.Service('control_left', Controller, left)
    rospy.Service('control_veer_left', Controller, veer_left)
    rospy.Service('control_right', Controller, right)
    rospy.Service('control_veer_right', Controller, veer_right)
    rospy.Service('control_full_speed', Controller, full_speed)
    rospy.Service('control_half_speed', Controller, half_speed)
    rospy.Service('control_reverse_full_speed', Controller, reverse_full_speed)
    rospy.Service('control_reverse_half_speed', Controller, reverse_half_speed)
    rospy.Service('control_rotate_left_half_speed', Controller, rotate_left_half_speed)
    rospy.Service('control_rotate_right_half_speed', Controller, rotate_right_half_speed)
    rospy.spin()


# Command to stop the vehicle
def stop(request):
    serialMotors.write(chr(control['all_stop']))
    serialMotors.flush()
    rospy.loginfo("STOP!")
    return True

# Command vehicle to turn left
def left(request):
    serialMotors.write(chr(control['right']['full_speed']))
    serialMotors.write(chr(control['left']['stop']))
    serialMotors.flush()
    rospy.loginfo("LEFT!")
    return True

# Command vehicle to turn slightly to the left
def veer_left(request):
    serialMotors.write(chr(control['right']['full_speed']))
    serialMotors.write(chr(control['left']['half_speed']))
    serialMotors.flush()
    rospy.loginfo("Veer Left!")
    return True

# Command vehicle to turn right
def right(request):
    serialMotors.write(chr(control['right']['stop']))
    serialMotors.write(chr(control['left']['full_speed']))
    serialMotors.flush()
    rospy.loginfo("RIGHT!")
    return True

# Command vehicle to turn slightly to the right
def veer_right(request):
    serialMotors.write(chr(control['right']['half_speed']))
    serialMotors.write(chr(control['left']['full_speed']))
    serialMotors.flush()
    rospy.loginfo("Veer Right!")
    return True

# Command to move vehicle forward at full speed
def full_speed(request):
    serialMotors.write(chr(control['right']['full_speed']))
    serialMotors.write(chr(control['left']['full_speed']))
    serialMotors.flush()
    rospy.loginfo("GO!")
    return True

# Command vehicle to slow down forward movement
def half_speed(request):
    serialMotors.write(chr(control['right']['half_speed']))
    serialMotors.write(chr(control['left']['half_speed']))
    serialMotors.flush()
    rospy.loginfo("Slow Down!")
    return True

# Check if there are objects detected from side sensors
def reverse_full_speed(request):
    serialMotors.write(chr(control['right']['rev_full_speed']))
    serialMotors.write(chr(control['left']['rev_full_speed']))
    serialMotors.flush()
    rospy.loginfo("Reverse!")
    return True

# Check if there are objects detected from side sensors
def reverse_half_speed(request):
    serialMotors.write(chr(control['right']['rev_half_speed']))
    serialMotors.write(chr(control['left']['rev_half_speed']))
    serialMotors.flush()
    rospy.loginfo("Reverse Half Speed!")
    return True

# Command vehicle to rotate at half speed to the left
def rotate_left_half_speed(request):
    serialMotors.write(chr(control['right']['half_speed']))
    serialMotors.write(chr(control['left']['rev_half_speed']))
    serialMotors.flush()
    rospy.loginfo("Rotate Left!")
    return True

# Command vehicle to rotate at half speed to the right
def rotate_right_half_speed(request):
    serialMotors.write(chr(control['right']['rev_half_speed']))
    serialMotors.write(chr(control['left']['half_speed']))
    serialMotors.flush()
    rospy.loginfo("Rotate Right!")
    return True

if __name__ == '__main__':
    controller_server()
