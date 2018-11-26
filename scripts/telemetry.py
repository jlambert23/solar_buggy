#!/usr/bin/env python

import json, rospy
from solar_buggy.msg import GeoPose, RelationalWayPoint
from geographic_msgs.msg import GeoPoint
from solar_buggy.srv import UpdateWaypoint

coord_path = '/home/odroid/catkin_buggy2/src/solar_buggy/server/settings/initial_coordinates.json'
telem_path = '/home/odroid/catkin_buggy2/src/solar_buggy/server/settings/telemetry.json'

class Telemetry:
    def __init__(self):
        rospy.init_node('test', anonymous=True)
        rospy.Subscriber('gps', GeoPose, self.update_gps)
        rospy.Subscriber('waypoint', RelationalWayPoint, self.update_waypoint)
        self.update_waypoint = rospy.ServiceProxy('update_waypoint', UpdateWaypoint)

        self.statistics = {
            'GPS': {
                'Longitude': 0,
                'Latitude': 0,
                'Bearing': 0
            },
            'Waypoint': {
                'Distance': 0,
                'Bearing': 0
            }
        }

    def update_gps(self, data):
        self.statistics['GPS']['Longitude'] = data.coordinates.longitude
        self.statistics['GPS']['Latitude'] = data.coordinates.latitude
        self.statistics['GPS']['Bearing'] = data.bearing
        self.update_statistics()

    def update_waypoint(self, data):
        self.statistics['Waypoint']['Distance'] = data.distance
        self.statistics['Waypoint']['Bearing'] = data.bearing

    def update_statistics(self):
        file = open(telem_path, 'w')
        file.write(json.dumps(self.statistics))
        file.close()

    def check_coordinates(self):
        file = open(coord_path)

        try:
            coord_obj = json.loads(file.read())
            file.close()

            if coord_obj['longitude'] == 0 and coord_obj['latitude'] == 0:
                return

            coordinates = GeoPoint()
            coordinates.longitude = coord_obj['longitude']
            coordinates.latitude = coord_obj['latitude']

            print('Waypoint submitted. Updating waypoint...')
            is_updated = False
            
            while not is_updated:
                rospy.wait_for_service('update_waypoint')
                is_updated = self.update_waypoint(coordinates)
                rospy.sleep(5)
            
            coord_obj['longitude'] = 0
            coord_obj['latitude'] = 0

            file = open(coord_path, 'w')
            file.write(json.dumps(coord_obj))
            file.close()

        except ValueError:
            rospy.logerr('Unable to read json in coordinate file.')

if __name__ == '__main__':
    ts = Telemetry()

    while not rospy.is_shutdown():
        ts.check_coordinates()
        # ts.update_statistics()
        rospy.sleep(5)
