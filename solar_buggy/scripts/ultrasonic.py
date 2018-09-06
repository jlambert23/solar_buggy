#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from solar_buggy.msg import Ultrasonic
from get_config import config

sensors_clear = {
    'left': True,
    'front_left': True,
    'front_right': True,
    'right': True
}

# TODO: Test how this will change with the different threaded calls in the handle_* functions.
def publish_to_controller():
    pub = rospy.Publisher('sensors', Ultrasonic, queue_size=20)
    pub.publish(*sensors_clear.values())
    # rospy.loginfo(sensors_clear.values())


def handle_left(data):
    distance = int(data.data) * 10
    print(distance)

    if distance >= 30 and not sensors_clear['left']:
        sensors_clear['left'] = True
        publish_to_controller()

    elif distance < 30 and sensors_clear['left']:
        sensors_clear['left'] = False
        publish_to_controller()


def handle_front_left(data):
    distance = int(data.data) * 10
    print(distance)

    if distance >= 30 and not sensors_clear['front_left']:
        sensors_clear['front_left'] = True
        publish_to_controller()

    elif distance < 30 and sensors_clear['front_left']:
        sensors_clear['front_left'] = False
        publish_to_controller()


def handle_front_right(data):
    distance = int(data.data) * 10
    print(distance)

    if distance >= 30 and not sensors_clear['front_right']:
        sensors_clear['front_right'] = True
        publish_to_controller()

    elif distance < 30 and sensors_clear['front_right']:
        sensors_clear['front_right'] = False
        publish_to_controller()


def handle_right(data):
    distance = int(data.data) * 10
    print(distance)

    if distance >= 30 and not sensors_clear['front_right']:
        sensors_clear['front_right'] = True
        publish_to_controller()

    elif distance < 30 and sensors_clear['front_right']:
        sensors_clear['front_right'] = False
        publish_to_controller()


if __name__ == '__main__':

    try:
        rospy.init_node('ultrasonic', anonymous=True)
        rospy.Subscriber('ultrasonic_' + config['ports']['left'], String, handle_left)
        rospy.Subscriber('ultrasonic_' + config['ports']['front_left'], String, handle_front_left)
        rospy.Subscriber('ultrasonic_' + config['ports']['front_right'], String, handle_front_right)
        rospy.Subscriber('ultrasonic_' + config['ports']['right'], String, handle_right)

        rospy.spin()

    except Exception as ex:
        print(str(ex))
