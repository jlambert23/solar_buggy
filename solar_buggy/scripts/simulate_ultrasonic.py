#!/usr/bin/env python

# Actual ultrasonic data will be coming through a serial port, 
# so this is a poor representation of how the data will be sent.
# However, for the sake of brevity and testing this script will 
# use a pub/sub representation that will simply send a digit until terminated.

import rospy
from std_msgs.msg import String
from get_config import config

l_sensor_data  = config['ports']['left']        + '\n50.1'
fl_sensor_data = config['ports']['front_left']  + '\n69.33'
fr_sensor_data = config['ports']['front_right'] + '\n3.7'
r_sensor_data  = config['ports']['right']       + '\n76.32'

def publish_digit():
    pub = rospy.Publisher('ultrasonic', String, queue_size=10)
    rospy.init_node('ultrasonic', anonymous=True)

    # Rate is 2 x second.
    rate = rospy.Rate(2)

    while not rospy.is_shutdown():

        # rospy.loginfo(l_sensor_data)
        pub.publish(l_sensor_data)

        # rospy.loginfo(fl_sensor_data)
        pub.publish(fl_sensor_data)
        
        # rospy.loginfo(fr_sensor_data)
        pub.publish(fr_sensor_data)

        # rospy.loginfo(r_sensor_data)
        pub.publish(r_sensor_data)

        rate.sleep()

if __name__ == '__main__':
    try:
        publish_digit()
    except rospy.ROSInterruptExcetion:
        pass
