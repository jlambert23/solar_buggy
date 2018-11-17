#!/usr/bin/env python

import serial, json, rospy
from solar_buggy.msg import Ultrasonic, RelationalWayPoint, GeoPose
from geographic_msgs.msg import GeoPoint
from std_msgs.msg import String

port = '/dev/ttyACM0'

class ReadSerial:
    def __init__(self):
        rospy.init_node('read_serial', anonymous=True) 
        self.gps_pub = rospy.Publisher('gps', GeoPose, queue_size=10)
        self.wp_pub = rospy.Publisher('waypoint', RelationalWayPoint, queue_size=10)
        self.ultra_pub = rospy.Publisher('ultrasonic', Ultrasonic, queue_size=10)
        self.serU = serial.Serial(port, 9600, timeout=3)

    def read_serial(self):
        while not rospy.is_shutdown():
            if self.serU.in_waiting == 0:
                continue

            self.read_serial_csv()

    def read_serial_csv(self):
        # ultrasonic_0, ultrasonic_1, ultrasonic_2, ultrasonic_3, longitude, latitude, heading, distance, bearing    
        data = self.serU.readline().split(',')
        
        try:
            self.ultra_pub.publish(
                back = float(data[0]),
                right = float(data[1]),
                front = float(data[2]),
                left = float(data[3])
            )

            coordinates = GeoPoint()
            coordinates.longitude = float(data[4])
            coordinates.latitude = float(data[5])

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

    def update_waypoint(self, longitude, latitude):
        rate = rospy.Rate(10)
        confirm = False

        while not confirm:
            self.serU.flush()

            while self.serU.readline():
                self.serU.write("#w") # Send waypoint command
                print("Waiting for serial to clear...")

            output = str(longitude) + "," + str(latitude)
            print("Sending long/lat: " + output)
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

if __name__ == '__main__':
    rs = ReadSerial()
    # rs.update_waypoint(28.5845915, -81.1997349)
    rs.read_serial()

    rs.serU.close()
    