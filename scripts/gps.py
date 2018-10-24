#!/usr/bin/env python

import rospy
import math

from std_msgs.msg import String
from geographic_msgs.msg import GeoPoint
from solar_buggy.msg import RelationalWayPoint, Pose

DISTANCE_TOLERANCE = 20.0 # in feet
ANGULAR_TOLERANCE = 15.0 # in degrees

def find_delta(x, y):
    return (x - y + 180) % 360 - 180

class GpsNode:

    def __init__(self):
        rospy.init_node('gps', anonymous=True)

        # Convert tolerance to miles and store in object.
        self.distance_tolerance = round(DISTANCE_TOLERANCE / 5280.0, 8)    
        
        self.waypoint = RelationalWayPoint()
        self.coordinates = GeoPoint()

        self.cmd_pub = rospy.Publisher('gps_cmd', String, queue_size=10)
        self.geopoint_sub = rospy.Subscriber('gps', Pose, self.update_pose)
        self.waypoint_sub = rospy.Subscriber('waypoint', RelationalWayPoint, self.update_waypoint)

    def update_pose(self, data):
        self.coordinates = data.coordinates
        self.bearing = data.bearing

    def update_waypoint(self, data):
        if self.waypoint is not None:
            self.waypoint_old = self.waypoint

        self.waypoint = data
        self.waypoint.distance = round(data.distance, 8)

        self.move2goal()

    def move2goal(self):
        if self.coordinates.latitude == 0 and self.coordinates.longitude == 0:
            rospy.loginfo('Waiting for fix...')
            return

        if self.waypoint.distance >= self.distance_tolerance:
            bearing_delta = find_delta(self.bearing, self.waypoint.bearing)

            if bearing_delta < 0 - ANGULAR_TOLERANCE:
                #clock-wise; turn right
                self.cmd_pub.publish('right')

            elif bearing_delta > ANGULAR_TOLERANCE:
                # counter clock-wise; turn left
                self.cmd_pub.publish('left')

            else:
                self.cmd_pub.publish('half_speed')

        else:
            self.cmd_pub.publish('destination_reached')
        

if __name__ == '__main__':
    gps = GpsNode()
    rospy.spin()
