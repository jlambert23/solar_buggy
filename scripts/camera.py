#!/usr/bin/env python
import rospy
from solar_buggy.msg import Pose
import numpy as np
import cv2

MAX_WHEEL_SPEED = 32
MIN_WHEEL_SPEED = -32

# (min, max)
HSV_PINK = ([121,39,98], [240,255,255])
HSV_ORANGE = ([12,80,180], [30,255,255])
LAB_ORANGE = ([130,150,150], [210,200,215])

height = 120
width = 160
third = width/3
twothird = third * 2
minArea = 50

class CameraNode:

    def __init__(self):
        rospy.init_node('camera', anonymous=True)
        self.pub = rospy.Publisher('cam_vel', Pose, queue_size=10)

        self.left_wheel = 0.0
        self.right_wheel = 0.0
        self.warning = False
        self.inconsistencies = 0
        self.cam = cv2.VideoCapture(0)

        if self.cam.isOpened():
            self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT,120) 
            self.cam.set(cv2.CAP_PROP_FRAME_WIDTH,160) 
        else:
            rospy.loginfo("Camera unable to start.")
            raise SystemError

    def update_pose(self, data):
        self.left_wheel = data.left_wheel_velocity
        self.right_wheel = data.right_wheel_velocity

    def handler(self):
        pose = Pose()
        pose.source = 'camera'
        pose.status = 0
        
        ##### Setting up image tracking #####
        ret,frame = self.cam.read()
        # hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
        lab=cv2.cvtColor(frame,cv2.COLOR_BGR2LAB)
        # image_mask=cv2.inRange(hsv,np.array(HSV_ORANGE[0]),np.array(HSV_ORANGE[1]))   
        image_mask=cv2.inRange(lab,np.array(LAB_ORANGE[0]),np.array(LAB_ORANGE[1]))   
        # erode=cv2.erode(image_mask,None,iterations=1)
        moments=cv2.moments(image_mask,True)

        if moments['m00'] >= minArea:
            x=moments['m10']/moments['m00']

            if (x < third):
                flag = 'left'
            elif (x > twothird):
                flag = 'right'
            else:
                flag = 'front'
            
            if (x > third and x < twothird):
                rospy.loginfo(flag)                

                if flag == 'right':
                    self.left_wheel += 1
                    self.right_wheel -= 1
                elif flag == 'left':
                    self.left_wheel -= 1
                    self.right_wheel += 1
                else:
                    if not self.warning:
                        self.left_wheel = 0
                        self.right_wheel = 0
                        self.warning = True
                        self.inconsistencies = 0

                    self.left_wheel -= 1
                    self.right_wheel += 1

                if self.warning:
                    if self.inconsistencies > 3:
                        self.warning = False
                    else:
                        self.inconsistencies += 1
                        return

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
                pose.status = 1

        self.pub.publish(pose)
        self.display(('image_mask', image_mask))

    def display(self, *args):
        try:
            for im in args:
                cv2.imshow(im[0], im[1])

            cv2.waitKey(1)
            
        except(KeyboardInterrupt):
            cv2.destroyAllWindow()
            scam.release()

if __name__ == '__main__':
    cam = CameraNode()
    rospy.Subscriber('cmd_vel', Pose, cam.update_pose)
    rate = rospy.Rate(30) # 30hz

    while not rospy.is_shutdown():
        start_time = rospy.get_rostime()
        cam.handler()
        print(rospy.get_rostime() - start_time)
        rate.sleep()
