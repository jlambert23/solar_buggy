#!/usr/bin/env python
from solar_buggy.msg import Pose
from imutils.object_detection import non_max_suppression
import numpy as np
import rospy
import cv2

# Motor parameters
max_wheel_speed = 16
min_wheel_speed = -16
p_gain = 0.01
d_gain = 0.75

# Camera parameters
display_image = False
width = 300
height = 225
error_threshold = 0.75

# HOG descriptor parameters
winStride = (4, 4)
padding = (8, 8)
scale = 1.05

# Color detector parameters
lab_orange = ([95,140,150], [210,200,215])
lab_orange2 = ([113,135,135] ,[250,210,215])


class CameraNode:

    def __init__(self):
        rospy.init_node('camera', anonymous=True)
        self.pub = rospy.Publisher('cam_vel', Pose, queue_size=10)
        self.cam = cv2.VideoCapture(-1)
        self.left_wheel = 0.0
        self.right_wheel = 0.0
        self.p_error = 0

        if self.cam.isOpened():
            self.cam.set(cv2.CAP_PROP_FRAME_WIDTH,width)
            self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT,height) 
        else:
            rospy.logerr("Camera unable to start.")
            raise SystemError
            
    # Subscribe to cmd_vel to update node's wheel velocity
    def update_pose(self, data):
        self.left_wheel = data.left_wheel_velocity
        self.right_wheel = data.right_wheel_velocity

    # Return list of paired rectangles and weights of detected objects for the current frame
    def object_detection(self, use_non_max_suppression=False):

        # Initialize HOG descripter to detect people
        print('reading camera')
        ret, frame = self.cam.read()
        hog = cv2.HOGDescriptor()
        hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

        # Detect people in image and translate rects to a numpy array
        (rects, weights) = hog.detectMultiScale(frame,
            winStride=winStride, padding=padding, scale=scale)
        rects = np.array([[x, y, x + w, y + h] for (x, y, w, h) in rects])

        # Optionally apply non-maxima suppression to remove overlapping bounding boxes
        if use_non_max_suppression:
            rects = non_max_suppression(rects, probs=None, overlapThresh=0.65)

        # Optionally display camera with drawn boxes
        if display_image:
            for (xA, yA, xB, yB) in rects:
                cv2.rectangle(frame, (xA, yA), (xB, yB), (0, 255, 0), 2)
            self.display("HOG Detector", frame)

        # Pair rectangles with weights and sort in descending weight order
        results = []
        if (any(results)):
            results = [(rects[i],weights[i][0]) for i in range(weights.size)]
            results.sort(key=lambda var: var[1], reverse=True)

        return results

    # Handle camera data and publish motor commands
    def handler(self):
        self._handler_color_detector()

    def _handler_HOG_detector(self):
        pose = Pose()
        pose.source = 'camera'
        pose.status = 0

        results = self.object_detection()
        results = []

        # If there are no results or the highest weight is 0, 
        # then publish inactive node
        if not any(results) or results[0][1] <= 0:
            self.pub.publish(pose)
            return

        # Select bounding box with the largest weight
        rect, weight = results[0]

        # Calculate centers of screen and boxes
        (xA, yA, xB, yB) = rect
        (mid_x, mid_y) = (xA + xB / 2, yA + yB / 2)
        half_cam_x = cam.get(3) / 2        
        error = mid_x - half_cam_x

        # Turn left or right depending on where detection is related to the center
        # Detection is to the left if error is negative; right if positive
        if abs(error) > error_threshold:
            self.left_wheel += p_gain * error
            self.right_wheel -= p_gain * error

        # Publish motor commands
        pose.status = 1
        pose.left_wheel_velocity = self.left_wheel
        pose.right_wheel_velocity = self.right_wheel
        self.pub.publish(pose)

    def _handler_color_detector(self):
        pose = Pose()
        pose.source = 'camera'
        pose.status = 0

        # Setup camera configurations
        ret, frame = self.cam.read()
        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        image_mask = cv2.inRange(lab, np.array(lab_orange[0]), np.array(lab_orange[1]))

        # Optionally display image (for debugging)
        if display_image:
            self.display(('frame', frame), ('image_mask', image_mask))

        # Segment the image mask x-axis into n slices
        n = 10
        x_split = split_integer(width, n)
        areas = [image_mask[0:height, 0:x_split[0]]]
        for i in range(n-1):
            area = image_mask[0:height, x_split[i] + 1:x_split[i+1]]
            areas.append(area)
        
        # Find the average of each segment of the image mask and apply a threshold
        # Then apply a weight from a normal distribution to flagged segments
        area_threshold = width / n * height * .017
        flags = [(np.sum(a)) / 255 > area_threshold for a in areas]
        weights = np.linspace(-2,2,n)
        weighted_flags = [a*b for a,b in zip(flags,weights)]
        p_error = sum(weighted_flags)
        d_error = p_error - self.p_error

        cmd = p_gain * p_error + d_gain * d_error
        print(p_error)
        
        if abs(p_error) > 0.5:
            self.left_wheel += p_error + d_gain * d_error
            self.right_wheel -= p_error + d_gain * d_error
            pose.status = 1
        else:
            pose.status = 0

        # Balance wheel speeds
        if self.left_wheel > max_wheel_speed:
            self.left_wheel = max_wheel_speed
        elif self.left_wheel < min_wheel_speed:
            self.left_wheel = min_wheel_speed
        if self.right_wheel > max_wheel_speed:
            self.right_wheel = max_wheel_speed
        elif self.right_wheel < min_wheel_speed:
            self.right_wheel = min_wheel_speed            
            
        # Publish command
        pose.left_wheel_velocity = self.left_wheel
        pose.right_wheel_velocity = self.right_wheel
        self.pub.publish(pose)

    # Display images in args. Expects the first argument to be the name
    # of the image and the second to be the image itself.
    def display(self, *args):
        try:
            if self.cam.isOpened():
                self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT,height * 2)

            for im in args:
                cv2.imshow(im[0], im[1])

            cv2.waitKey(1)

        except(KeyboardInterrupt):
            cv2.destroyAllWindow()
            self.cam.release()

def split_integer(dividend, divisor):
    xs = []
    for i in range(divisor):
        xs.append(dividend / divisor * (i+1))
    return xs

if __name__ == '__main__':
    cam = CameraNode()
    rospy.Subscriber('cmd_vel', Pose, cam.update_pose)
    rate = rospy.Rate(30)

    while not rospy.is_shutdown():
        cam.handler()
        rate.sleep()
        
    cam.cam.release()
