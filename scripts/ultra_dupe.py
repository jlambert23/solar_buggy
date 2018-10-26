#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from solar_buggy.msg import Ultrasonic, Pose

cmd_pub = rospy.Publisher('cam_vel', Pose, queue_size=10)

sensors_clear = {
    'left': True,
    'front': True,
    'right': True,
    'back': True
}

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

    def handler(self, ultra):
        pose = Pose()
        pose.source = 'camera'
        sensor_info = 0b0
        
        if ultra.front < 15.0:
            sensor_info |= 0b0001
        if ultra.left < 15.0:
            sensor_info |= 0b0010
        if ultra.right < 15.0:
            sensor_info |= 0b0100
        
        if ultra.front < 10.00 or ultra.left < 10.0 or ultra.right < 10.00:
            self.inconsistencies = 0

            if not self.warning:
                self.left_wheel = 0
                self.right_wheel = 0
                self.warning = True
            
            if ultra.right < 10.00:            
                self.left_wheel -= 1
                self.right_wheel += 1
            else:
                self.left_wheel += 1
                self.right_wheel -= 1
            
            if self.left_wheel > 32:
                self.left_wheel = 32
            elif self.left_wheel < -32:
                self.left_wheel = -32
            
            if self.right_wheel > 32:
                self.right_wheel = 32
            elif self.right_wheel < -32:
                self.right_wheel = -32            

            pose.left_wheel_velocity = self.left_wheel
            pose.right_wheel_velocity = self.right_wheel
            
            cmd_pub.publish(pose)
            return

        if self.warning:
            if self.inconsistencies > 3:
                self.warning = False
            else:
                self.inconsistencies += 1
                return

        #if ultra.back < 30.0:
        #    sensor_info |= 0b1000
        #print sensor_info
        #print ultra
           
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
        elif sensor_info == 0: # clear
            if self.left_wheel <= self.right_wheel:
                self.left_wheel = self.right_wheel
            if self.right_wheel <= self.left_wheel:
                self.right_wheel = self.left_wheel
            self.left_wheel += 1
            self.right_wheel += 1

        if self.left_wheel > 32:
            self.left_wheel = 32
        elif self.left_wheel < -32:
            self.left_wheel = -32
        
        if self.right_wheel > 32:
            self.right_wheel = 32
        elif self.right_wheel < -32:
            self.right_wheel = -32

        pose.left_wheel_velocity = self.left_wheel
        pose.right_wheel_velocity = self.right_wheel
        cmd_pub.publish(pose)

if __name__ == '__main__':
    rospy.init_node('camera', anonymous=True)

    ultra = UltraNode()
    rospy.Subscriber('ultrasonic', Ultrasonic, ultra.handler)
    rospy.spin()
