#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import numpy as np
import cv2

def camera():
    rospy.init_node('camera', anonymous=True)
    pub = rospy.Publisher('camera_cmd', String, queue_size=10)
    rate = rospy.Rate(30) # 10hz

    height = 120
    width = 160
    third = width/3
    twothird = third + third
    minArea = 1
    criticalArea = 13000
    flag = ''
    dir = ''

    ##### Camera/serial setup #####
    cam = cv2.VideoCapture(0)
    if cam.isOpened():
        cam.set(cv2.CAP_PROP_FRAME_HEIGHT,120) 
        cam.set(cv2.CAP_PROP_FRAME_WIDTH,160) 

    try:
        while not rospy.is_shutdown():
        ##### Setting up image tracking #####
            # big ball (pink): min: (121, 39, 98), max: (240, 255, 255)
            ret,frame = cam.read()
            hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
            image_mask=cv2.inRange(hsv,np.array([121,39,98]),np.array([240,255,255]))   
            erode=cv2.erode(image_mask,None,iterations=3)
            moments=cv2.moments(erode,True)
            area=moments['m00']

            if moments['m00'] >= minArea:
                x=moments['m10']/moments['m00']
                y=moments['m01']/moments['m00']
                cv2.circle(frame,(int(x),int(y)),5,(0,255,0),-1)
            
                if (x < third):
                    flag = 'left'
                elif (x > twothird):
                    flag = 'right'
                if (x > third and x < twothird):
                    if (flag == 'right'):                   
                        dir = 'veer_left'
                    else:
                        dir = 'veer_right'
                else:
                    dir = 'full_speed'

            cv2.imshow('eroded',erode)
            cv2.imshow('frame',frame)
            cv2.waitKey(1)

            pub.publish(dir)
            rate.sleep()

    except(KeyboardInterrupt):
        cv2.destroyAllWindow()
        scam.release()

if __name__ == '__main__':
    try:
        camera()
    except rospy.ROSInterruptException:
        pass

