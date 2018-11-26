#!/usr/bin/env python

import serial, json, rospy
from solar_buggy.msg import Ultrasonic, RelationalWayPoint, GeoPose
from solar_buggy.srv import UpdateWaypoint
from geographic_msgs.msg import GeoPoint
from std_msgs.msg import Bool

port = '/dev/ttyACM0'

class ReadSerial:
    def __init__(self):
        rospy.init_node('read_serial', anonymous=True)
        self.gps_pub = rospy.Publisher('gps', GeoPose, queue_size=10)
        self.wp_pub = rospy.Publisher('waypoint', RelationalWayPoint, queue_size=10)
        self.ultra_pub = rospy.Publisher('ultrasonic', Ultrasonic, queue_size=10)
        self.serU = serial.Serial(port, 9600, timeout=3)
        self.reading = True
        self.writing = False

        rospy.Service('update_waypoint', UpdateWaypoint, self.update_waypoint)

    def read_serial(self):
        while not rospy.is_shutdown():
            if self.serU.in_waiting == 0:
                continue

            if self.reading:
                self.read_serial_csv()
            else:
                self.writing = True

    def read_serial_csv(self):
        # ultrasonic_0, ultrasonic_1, ultrasonic_2, ultrasonic_3, latitude, longitude, heading, distance, bearing    
        data = self.serU.readline().split(',')
        
        try:
            self.ultra_pub.publish(
                back = float(data[0]),
                right = float(data[1]),
                front = float(data[2]),
                left = float(data[3])
            )

            coordinates = GeoPoint()
            coordinates.latitude = float(data[4])
            coordinates.longitude = float(data[5])

            self.gps_pub.publish(
                coordinates = coordinates,
                bearing = float(data[6])
            )

            self.wp_pub.publish(
                distance = float(data[7]),
                bearing = float(data[8])
            )
        
        except IndexError:
            return
        except ValueError:
            return

    def read_serial_json(self):
        data = json.loads(serU.readline())

        try:
            coordinates = GeoPoint()
            coordinates.latitude = data['gps']['latitude']
            coordinates.longitude = data['gps']['longitude']

            # Publishing gps data.
            self.gps_pub.publish(
                coordinates = coordinates,
                bearing = data['gps']['heading']
            )

            # Publishing waypoint data.
            self.wp_pub.publish(
                distance = data['gps']['waypoint']['distance'],
                bearing = data['gps']['waypoint']['bearing']
            )

            # Publishing ultrasonic data.
            self.ultra_pub.publish(
                back = data['ultrasonic']['0'],
                right = data['ultrasonic']['1'],
                front = data['ultrasonic']['2'],
                left = data['ultrasonic']['3'],
            )

        except KeyError:
            return
        except ValueError:
            return

    def transmit_new_waypoint(self, latitude, longitude):
        rate = rospy.Rate(100)
        confirm = False
        
        while not self.writing:
            rate.sleep()

        while not confirm:
            self.serU.flush()

            while self.serU.readline():
                self.serU.write("#w") # Send waypoint command
                print("Waiting for serial to clear...")

            output = str(latitude) + "," + str(longitude)
            print("Sending lat/long: " + output)
            self.serU.write(output + '\n') # no space

            ret = self.serU.readline().strip('\r\n')
            confirm = ret == output.strip('\n')
            print("Received: " + ret)

            if not confirm:
                print("Incorrect output... trying again.")
                rate.sleep()
            else:
                print("Waypoint successfully entered... Resuming output.")
                self.serU.write("#o") # Send output command
                rate.sleep()

        self.writing = False

    def update_waypoint(self, geo_point):
        print('sending: ' + str(geo_point.coordinates))
        self.reading = False
        self.transmit_new_waypoint(geo_point.coordinates.latitude, geo_point.coordinates.longitude)
        self.reading = True
        return True

if __name__ == '__main__':
    rs = ReadSerial()
    # rs.update_waypoint(28.5845915, -81.1997349)
    rs.read_serial()
    rs.serU.close()
    