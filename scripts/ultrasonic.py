#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from solar_buggy.msg import Ultrasonic, Pose

HIGH_THRESHOLD = 35.0
MID_THRESHOLD = 26.25
LOW_THRESHOLD = 17.5

MAX_ANGULAR_VEL = 15.0

cmd_pub = rospy.Publisher('ultra_vel', Pose, queue_size=10)

'''
bit string order (MSB -> LSB): back, right, left, front

sensor order is:
    right: 1
    front: 2
    left: 3
'''

class UltraNode:

    def __init__(self):
        self.left_wheel = 0.0
        self.right_wheel = 0.0
        self.inconsistencies = 0
        self.warning = False

    def update_pose(self, pose):
        self.left_wheel = pose.left_wheel_velocity
        self.right_wheel = pose.right_wheel_velocity

    def handler(self, ultra):
        pose = Pose()
        pose.source = 'ultrasonic'
        pose.status = 1
        sensor_info = 0b0

        if ultra.front < LOW_THRESHOLD or ultra.left < LOW_THRESHOLD or ultra.right < LOW_THRESHOLD:
            self.inconsistencies = 0

            if not self.warning:
                self.left_wheel = 0
                self.right_wheel = 0
                self.warning = True
            
            if ultra.right < LOW_THRESHOLD:            
                self.left_wheel -= 1
                self.right_wheel += 1
            else:
                self.left_wheel += 1
                self.right_wheel -= 1
            
            if self.left_wheel > 16:
                self.left_wheel = 16
            elif self.left_wheel < -16:
                self.left_wheel = -16
            
            if self.right_wheel > 16:
                self.right_wheel = 16
            elif self.right_wheel < -16:
                self.right_wheel = -16            

            if self.left_wheel > self.right_wheel + MAX_ANGULAR_VEL:
                self.left_wheel = self.right_wheel + MAX_ANGULAR_VEL
            elif self.left_wheel < self.right_wheel - MAX_ANGULAR_VEL:
                self.left_wheel = self.right_wheel - MAX_ANGULAR_VEL
            elif self.right_wheel > self.left_wheel + MAX_ANGULAR_VEL:
                self.right_wheel = self.left_wheel + MAX_ANGULAR_VEL
            elif self.right_wheel < self.left_wheel - MAX_ANGULAR_VEL:
                self.right_wheel = self.left_wheel - MAX_ANGULAR_VEL

            pose.left_wheel_velocity = self.left_wheel
            pose.right_wheel_velocity = self.right_wheel
            
            cmd_pub.publish(pose)
            return

        elif ultra.left < MID_THRESHOLD or ultra.right < MID_THRESHOLD or ultra.front < MID_THRESHOLD:
            if ultra.front < MID_THRESHOLD:
                sensor_info |= 0b0001
            if ultra.left < MID_THRESHOLD:
                sensor_info |= 0b0010
            if ultra.right < MID_THRESHOLD:
                sensor_info |= 0b0100
                    
            if sensor_info == 2: # left
                self.left_wheel += 3
                self.right_wheel -= 3
            elif sensor_info == 4: # right
                self.left_wheel -= 3
                self.right_wheel += 3
            elif sensor_info == 1: # front
                self.left_wheel -= 3
                self.right_wheel += 3
            elif sensor_info == 5:
                self.left_wheel -= 5
                self.right_wheel += 5
            elif sensor_info == 3:
                self.left_wheel += 5
                self.right_wheel -= 5

        elif ultra.left < HIGH_THRESHOLD or ultra.right < HIGH_THRESHOLD or ultra.front < HIGH_THRESHOLD:
            if ultra.front < HIGH_THRESHOLD:
                sensor_info |= 0b0001
            if ultra.left < HIGH_THRESHOLD:
                sensor_info |= 0b0010
            if ultra.right < HIGH_THRESHOLD:
                sensor_info |= 0b0100
                    
            if sensor_info == 2: # left
                self.left_wheel += 1
                self.right_wheel -= 1
            elif sensor_info == 4: # right
                self.left_wheel -= 1
                self.right_wheel += 1
            elif sensor_info == 1: # front
                self.left_wheel -= 1
                self.right_wheel += 1
            elif sensor_info == 5:
                self.left_wheel -= 3
                self.right_wheel += 3
            elif sensor_info == 3:
                self.left_wheel += 3
                self.right_wheel -= 3
        else:
            pose.status = 0

        if self.warning:
            if self.inconsistencies > 3:
                self.warning = False
            else:
                self.inconsistencies += 1
                return

        if self.left_wheel > 32:
            self.left_wheel = 32
        elif self.left_wheel < -32:
            self.left_wheel = -32
        if self.right_wheel > 32:
            self.right_wheel = 32
        elif self.right_wheel < -32:
            self.right_wheel = -32

        if self.left_wheel > self.right_wheel + MAX_ANGULAR_VEL:
            self.left_wheel = self.right_wheel + MAX_ANGULAR_VEL
        elif self.left_wheel < self.right_wheel - MAX_ANGULAR_VEL:
            self.left_wheel = self.right_wheel - MAX_ANGULAR_VEL
        elif self.right_wheel > self.left_wheel + MAX_ANGULAR_VEL:
            self.right_wheel = self.left_wheel + MAX_ANGULAR_VEL
        elif self.right_wheel < self.left_wheel - MAX_ANGULAR_VEL:
            self.right_wheel = self.left_wheel - MAX_ANGULAR_VEL

        pose.left_wheel_velocity = self.left_wheel
        pose.right_wheel_velocity = self.right_wheel
        cmd_pub.publish(pose)

if __name__ == '__main__':
    rospy.init_node('ultrasonic', anonymous=True)

    ultra = UltraNode()
    rospy.Subscriber('ultrasonic', Ultrasonic, ultra.handler)
    rospy.Subscriber('cmd_vel', Pose, ultra.update_pose)
    rospy.spin()
