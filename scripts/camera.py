#!/usr/bin/env python
import rospy
from solar_buggy.msg import Pose
import numpy as np
import cv2

MAX_WHEEL_SPEED = 32
MIN_WHEEL_SPEED = -32
MAX_ANGULAR_VEL = 10.0

# (min, max)
HSV_PINK = ([121,39,98], [240,255,255])
HSV_ORANGE = ([12,80,180], [30,255,255])
LAB_ORANGE2 = ([130,140,150], [210,200,215])
LAB_ORANGE = ([113,135,135] ,[250,210,215])

height = 180
width = 240

y_third = height/3
y_two_third = y_third * 2
x_quarter = width/4
x_half = x_quarter * 2
x_three_quarter = x_quarter * 3

minArea = 90
mid_area_threshold = x_quarter * y_third * 0.1
bot_area_threshold = x_quarter * y_third * 0.17

class CameraNode:

    def __init__(self):
        rospy.init_node('camera', anonymous=True)
        self.pub = rospy.Publisher('cam_vel', Pose, queue_size=10)

        self.left_wheel = 0.0
        self.right_wheel = 0.0
        self.warning = False
        self.inconsistencies = 0
        self.cam = cv2.VideoCapture(0)

        self.last_flag = ''

        if self.cam.isOpened():
            self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT,height) 
            self.cam.set(cv2.CAP_PROP_FRAME_WIDTH,width) 
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
        # image_mask=cv2.inRange(hsv,np.array(HSV_ORANGE[0]),np.array(HSV_ORANGE[1]))   
        lab=cv2.cvtColor(frame,cv2.COLOR_BGR2LAB)
        image_mask=cv2.inRange(lab,np.array(LAB_ORANGE[0]),np.array(LAB_ORANGE[1]))   

        # erode=cv2.erode(image_mask,None,iterations=1)
        moments=cv2.moments(image_mask,True)

        # self.display(('image_mask', image_mask), ('frame', frame))

        top_left     = image_mask[0:y_third, 0:x_quarter]
        top_midleft  = image_mask[0:y_third, x_quarter+1:x_half]
        top_midright = image_mask[0:y_third, x_half+1:x_three_quarter]
        top_right    = image_mask[0:y_third, x_three_quarter+1:width]
        mid_left     = image_mask[y_third+1:y_two_third, 0:x_quarter]
        mid_midleft  = image_mask[y_third+1:y_two_third, x_quarter+1:x_half]
        mid_midright = image_mask[y_third+1:y_two_third, x_half+1:x_three_quarter]
        mid_right    = image_mask[y_third+1:y_two_third, x_three_quarter+1:width]
        bot_left     = image_mask[y_two_third+1:height, 0:x_quarter]
        bot_midleft  = image_mask[y_two_third+1:height, x_quarter+1:x_half]
        bot_midright = image_mask[y_two_third+1:height, x_half+1:x_three_quarter]
        bot_right    = image_mask[y_two_third+1:height, x_three_quarter+1:width]

        areas = [
            [np.sum(top_left) / 255, np.sum(top_midleft) / 255, np.sum(top_midright) / 255 , np.sum(top_right) / 255],
            [np.sum(mid_left) / 255, np.sum(mid_midleft) / 255, np.sum(mid_midright) / 255 , np.sum(mid_right) / 255],
            [np.sum(bot_left) / 255, np.sum(bot_midleft) / 255, np.sum(bot_midright) / 255 , np.sum(bot_right) / 255]
        ]

        flags = []
        flags.append([i > mid_area_threshold for i in areas[1]])
        flags.append([i > bot_area_threshold for i in areas[2]])

        if moments['m00'] >= minArea:
            pose.status = 1
            
            if any(flags[0]) or any(flags[1]):
                print(flags)

            if flags[1][0] or flags[1][1] or flags[0][0] or flags[0][1]:
                self.last_flag = 'left'
            if flags[1][2] or flags[1][3] or flags[0][2] or flags[0][3]:
                self.last_flag = 'right'

            if any(flags[1]):
                value = 0
                self.inconsistencies = 0

                if not self.warning:
                    self.left_wheel = 0
                    self.right_wheel = 0
                    self.warning = True
                
                # spin
                if all(flags[1]):
                    if self.last_flag == 'left':
                        self.left_wheel = 10
                        self.right_wheel = -10
                    else:
                        self.left_wheel = -10
                        self.right_wheel = 10

                    # rospy.sleep(0.6)

                # middle flags
                elif flags[1][1] and flags[1][2]:
                    # turn right
                    if flags[1][0] and not flags[1][3]:
                        self.left_wheel += 1
                        self.right_wheel -= 1
                    # turn left
                    elif flags[1][3] and not flags[1][0]:
                        self.left_wheel -= 1
                        self.right_wheel += 1
                    # spin
                    else:
                        if self.last_flag == 'left':
                            self.left_wheel = 10
                            self.right_wheel = -10
                        else:
                            self.left_wheel = -10
                            self.right_wheel = 10
                # turn left
                elif flags[1][0] or flags[1][1]:
                    self.left_wheel += 1
                    self.right_wheel -= 1
                # turn right
                elif flags[1][2] or flags[1][3]:
                    self.left_wheel -= 1
                    self.right_wheel += 1
                # spin
                else:
                    if self.last_flag == 'left':
                        self.left_wheel = 10
                        self.right_wheel = -10
                    else:
                        self.left_wheel = -10
                        self.right_wheel = 10

                    # rospy.sleep(0.6)
                
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
                
                self.pub.publish(pose)
                return

            # elif any(flags[0]):
            #     info = 0b0

            #     if flags[0][0]:
            #         info |= 0b001
            #     if flags[0][1]:
            #         info |= 0b010
            #     if flags[0][2]:
            #         info |= 0b100

            #     if info == 1: # left
            #         self.left_wheel += 1
            #     elif info == 3:
            #         self.left_wheel += 2
            #     elif info == 4: # right
            #         self.right_wheel += 1
            #     elif info == 5:
            #         self.right_wheel += 2
            #     else:
            #         pose.status = 0

            else:
                pose.status = 0

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

            if self.left_wheel > self.right_wheel + MAX_ANGULAR_VEL:
                self.left_wheel = self.right_wheel + MAX_ANGULAR_VEL
            elif self.left_wheel < self.right_wheel - MAX_ANGULAR_VEL:
                self.left_wheel = self.right_wheel - MAX_ANGULAR_VEL
            elif self.right_wheel > self.left_wheel + MAX_ANGULAR_VEL:
                self.right_wheel = self.left_wheel + MAX_ANGULAR_VEL
            elif self.right_wheel < self.left_wheel - MAX_ANGULAR_VEL:
                self.right_wheel = self.left_wheel - MAX_ANGULAR_VEL

        else:
            pose.status = 0

        pose.left_wheel_velocity = self.left_wheel
        pose.right_wheel_velocity = self.right_wheel
        self.pub.publish(pose)

    def display(self, *args):
        try:

            if self.cam.isOpened():
                self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT,height * 3) 
                self.cam.set(cv2.CAP_PROP_FRAME_WIDTH,width * 3) 

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
        cam.handler()
        rate.sleep()
