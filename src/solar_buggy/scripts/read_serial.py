#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def publish_sensor_data(data):
    port = data.data[0:4].strip('\r\n')
    distance = data.data[5:].strip('\r\n')
        
    pub = rospy.Publisher('ir_' + port, String, queue_size=10)
    pub.publish(distance)


# Currently using pubsub for simulated serial data. Will replace as arduino uno is implemented.
def read_serial():
    rospy.Subscriber('ir', String, publish_sensor_data)
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('read_serial', anonymous=True)
    read_serial()