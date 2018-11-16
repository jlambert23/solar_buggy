#!/usr/bin/env python
import rospy
from solar_buggy.msg import Pose
import numpy as np
import cv2

MAX_WHEEL_SPEED = 16
MIN_WHEEL_SPEED = -16

# (min, max)
HSV_PINK = ([121,39,98], [240,255,255])
HSV_ORANGE = ([12,80,180], [30,255,255])

height = 120
width = 160
half = width/2
third = width/3
twothird = third + third
minArea = 100

class CameraNode:

    def __init__(self):
        rospy.init_node('camera', anonymous=True)

        self.left_wheel = 0.0
        self.right_wheel = 0.0
        self.cam = cv2.VideoCapture(0)

        self.pub = rospy.Publisher('cam_vel', Pose, queue_size=10)

        if self.cam.isOpened():
            self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT,120) 
            self.cam.set(cv2.CAP_PROP_FRAME_WIDTH,160) 

            while not rospy.is_shutdown():
                self.handler()

        else:
            rospy.loginfo("Camera unable to start.")

    def update_pose(self, data):
        self.left_wheel = data.left_wheel_velocity
        self.right_wheel = data.rigth_wheel_velocity

    def handler(self):
        pose = Pose()
        pose.source = 'camera'
        pose.status = 1
        rate = rospy.Rate(30) # 30hz
        
        ##### Setting up image tracking #####
        ret,frame = self.cam.read()
        hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
        image_mask=cv2.inRange(hsv,np.array(HSV_ORANGE[0]),np.array(HSV_ORANGE[1]))   
        erode=cv2.erode(image_mask,None,iterations=3)
        moments=cv2.moments(erode,True)
        area=moments['m00']

        if moments['m00'] >= minArea:
            x=moments['m10']/moments['m00']
            y=moments['m01']/moments['m00']
            cv2.circle(frame,(int(x),int(y)),5,(0,255,0),-1)
    
            if (x < half):
                flag = 'left'
            elif (x > half):
                flag = 'right'
            else:
                flag = ''

            if (x > third and x < twothird):
                if (flag == 'right'):                   
                    self.left_wheel = 16
                    self.right_wheel = -16
                elif (flag == 'left'):
                    self.left_wheel = -16
                    self.right_wheel = 16    
            else:
                pose.status = 0
                self.pub.publish(pose)

            if self.left_wheel > MAX_WHEEL_SPEED:
                self.left_wheel = MAX_WHEEL_SPEED
            elif self.left_wheel < MIN_WHEEL_SPEED:
                self.left_wheel = MIN_WHEEL_SPEED

            if self.right_wheel > MAX_WHEEL_SPEED:
                self.right_wheel = MAX_WHEEL_SPEED
            elif self.right_wheel < MIN_WHEEL_SPEED:
                self.right_wheel = MIN_WHEEL_SPEED

            pose.left_wheel_velocity = self.left_wheel
            pose.right_wheel_velocity = self.right_wheel

            self.pub.publish(pose)

        else:
            pose.status = 0
            self.pub.publish(pose)

        rate.sleep()

    def display(self):
        try:
            cv2.imshow('eroded',erode)
            cv2.imshow('frame',frame)
            cv2.waitKey(1)
            
        except(KeyboardInterrupt):
            cv2.destroyAllWindow()
            scam.release()

if __name__ == '__main__':
    cam = CameraNode()
    rospy.Subscriber('cmd_vel', Pose, cam.update_pose)
