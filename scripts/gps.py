#!/usr/bin/env python

import rospy
import math

from std_msgs.msg import Bool
from geographic_msgs.msg import GeoPoint
from solar_buggy.msg import RelationalWayPoint, Pose, GeoPose

DISTANCE_TOLERANCE = 30 # in meters
ANGULAR_TOLERANCE = 15.0 # in degrees
BEARING_COEFF = 0.07 # Coefficient for proportional rotation control
MAX_ANGULAR_VEL = 10

def find_delta(x, y):
    return (x - y + 180) % 360 - 180

class GpsNode:

    def __init__(self):
        rospy.init_node('gps', anonymous=True)
        self.cmd_pub = rospy.Publisher('gps_vel', Pose, queue_size=10)
        self.update_wp_pub = rospy.Publisher('update_waypoint', GeoPoint, queue_size=10)

        # Convert tolerance to kilometers
        self.distance_tolerance = DISTANCE_TOLERANCE / 1000.0
        
        self.waypoint = RelationalWayPoint()
        self.coordinates = GeoPoint()
        self.origin = GeoPoint()
        self.left_wheel = 0
        self.right_wheel = 0
        self.bearing_old = 0
        self.destination_reached = False
        self.returning  = False

        rospy.Subscriber('gps', GeoPose, self.update_global_pose)
        rospy.Subscriber('waypoint', RelationalWayPoint, self.update_waypoint)
        rospy.Subscriber('cmd_vel', Pose, self.update_pose)
        rospy.Subscriber('return', Bool, self.update_returning)

    def update_pose(self, data):
        self.left_wheel = data.left_wheel_velocity
        self.right_wheel = data.right_wheel_velocity

    def update_global_pose(self, data):
        self.coordinates = data.coordinates
        self.bearing = data.bearing

        if self.origin.longitude == 0 and self.origin.latitude == 0:
            self.origin = self.coordinates

    def update_waypoint(self, data):
        self.waypoint = data
        # self.waypoint.distance = round(data.distance, 8)
        # self.move2goal()
        self.go_to_waypoint_and_back()

    def update_returning(self, is_returning):
        self.returning = is_returning

    def go_to_waypoint_and_back(self):
        if not self.destination_reached:
            self.move2goal()
        
        elif not self.returning:
            rate = rospy.Rate(10)
            self.update_wp_pub.publish(self.origin)
            self.destination_reached = False
            rospy.loginfo("Returning...")
            rate.sleep()
            
        else:
            rospy.loginfo("Circuit complete!")

    def move2goal(self):
        pose = Pose()
        pose.source = 'GPS'

        rate = rospy.Rate(1)

        if self.coordinates.latitude == 0 and self.coordinates.longitude == 0:
            rospy.loginfo('Waiting for fix...')
            rate.sleep()
            return

        if self.waypoint.distance >= self.distance_tolerance:
            bearing_delta = find_delta(self.bearing, self.waypoint.bearing)

            print("distance: " + str(self.waypoint.distance) + "\nbearing: " + str(bearing_delta))

            # 'Normalize' bearing
            if abs(self.bearing_old - bearing_delta) > 15:
                p_controller = abs(self.bearing_old) * BEARING_COEFF
            else:
                p_controller = abs(bearing_delta) * BEARING_COEFF
            self.bearing_old = bearing_delta

            # print(p_controller)

            #clock-wise; turn right
            if bearing_delta < 0 - ANGULAR_TOLERANCE:
                self.left_wheel += p_controller
                self.right_wheel -= p_controller * BEARING_COEFF

            # counter clock-wise; turn left
            elif bearing_delta > ANGULAR_TOLERANCE:
                self.left_wheel -= p_controller * BEARING_COEFF
                self.right_wheel += p_controller

            # Go straight(ish)
            else:
                max_speed = max(self.left_wheel, self.right_wheel)
                self.left_wheel = 1 + max_speed
                self.right_wheel = 1 + max_speed

            ####
            if self.left_wheel > self.right_wheel + MAX_ANGULAR_VEL:
                self.left_wheel = self.right_wheel + MAX_ANGULAR_VEL
            elif self.left_wheel < self.right_wheel - MAX_ANGULAR_VEL:
                self.left_wheel = self.right_wheel - MAX_ANGULAR_VEL
            elif self.right_wheel > self.left_wheel + MAX_ANGULAR_VEL:
                self.right_wheel = self.left_wheel + MAX_ANGULAR_VEL
            elif self.right_wheel < self.left_wheel - MAX_ANGULAR_VEL:
                self.right_wheel = self.left_wheel - MAX_ANGULAR_VEL

            if self.left_wheel > 63:
                self.left_wheel = 63
            elif self.left_wheel < 0:
                self.left_wheel = 0
            if self.right_wheel > 63:
                self.right_wheel = 63
            elif self.right_wheel < 0:
                self.right_wheel = 0
        
            pose.left_wheel_velocity = self.left_wheel
            pose.right_wheel_velocity = self.right_wheel    
            pose.status = 1

        else:
            pose.status = 0
            rospy.loginfo("Destination reached!")
            self.destination_reached = True
        
        self.cmd_pub.publish(pose)

if __name__ == '__main__':
    gps = GpsNode()
    rospy.spin()
