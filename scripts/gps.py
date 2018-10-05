#!/usr/bin/env python

import rospy
import math

from std_msgs.msg import String
from geographic_msgs.msg import GeoPoint
from solar_buggy.msg import RelationalWayPoint

# Tolerance for vehicle to satisfy waypoint in feet.
DISTANCE_TOLERANCE = 5.0

class GpsNode:

    def __init__(self):
        rospy.init_node('gps', anonymous=True)

        # Convert tolerance to miles and store in object.
        self.distance_tolerance = round(DISTANCE_TOLERANCE / 5280.0, 8)    
        
        self.waypoint = RelationalWayPoint()
        self.location = GeoPoint()

        self.cmd_pub = rospy.Publisher('gps_cmd', String, queue_size=10)
        self.geopoint_sub = rospy.Subscriber('gps', GeoPoint, self.update_location)
        self.waypoint_sub = rospy.Subscriber('waypoint', RelationalWayPoint, self.update_waypoint)

    def update_location(self, data):
        self.location = data

    def update_waypoint(self, data):
        if self.waypoint is not None:
            self.waypoint_old = self.waypoint

        self.waypoint = data
        self.waypoint.distance = round(data.distance, 8)

        self.move2goal()

    def move2goal(self):
        if self.location is None or (self.location.latitude == 0 and self.location.longitude == 0):
            rospy.loginfo('Waiting for fix...')
            return

        if self.waypoint.distance >= self.distance_tolerance:
            if self.waypoint.bearing < self.waypoint_old.bearing:
                self.cmd_pub.publish('left')
            elif self.waypoint.bearing > self.waypoint_old.bearing:
                self.cmd_pub.publish('right')
            else:
                if self.waypoint.distance > self.waypoint_old.distance:
                    self.cmd_pub.publish('turn_around')
                else:
                    self.cmd_pub.publish('full_speed')
        else:
            self.cmd_pub.publish('destination_reached')
        

if __name__ == '__main__':
    gps = GpsNode()
    rospy.spin()
