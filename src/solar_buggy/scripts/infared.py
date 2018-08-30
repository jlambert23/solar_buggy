#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Int8
from get_config import config

sensors = {
    'left': False,
    'front_left': False,
    'front_right': False,
    'right': False
}

def handle_left(data):
    distance = float(data.data)
    rospy.loginfo('left: ' + data.data)
    pub = rospy.Publisher('left_sensor', Int8, queue_size=10)
    
    if distance >= config['proximity']['mid'] and not sensors['left']:
        sensors['left'] = True
        pub.publish(True)
        # leftSensor = False

    elif distance <= config['proximity']['low'] and sensors['left']:
        sensors['left'] = False
        pub.publish(False)
    #     leftSensor = False
    #     lookingFromLeft = True

def handle_front_left(data):
    distance = float(data.data)
    rospy.loginfo('front left: ' + data.data)
    pub = rospy.Publisher('front_left_sensor', Int8, queue_size=10)

    if distance > config['proximity']['high']:
        signalgo = True
    #     if corner.isSet():
    #       corner.clear()

    elif distance >= config['proximity']['mid'] and sensors['front_left']:
        sensors['front_left'] = False
        pub.publish(False)
    #     signalGo = True
    #     leftFrontSensor = False
    #     if corner.isSet():
    #         corner.clear()

    elif distance <= config['proximity']['low'] and not sensors['front_left']:
        sensors['front_left'] = True
        pub.publish(True)
    #     signalGo = False
    #     leftFrontSensor = True

def handle_front_right(data):
    distance = float(data.data)
    rospy.loginfo('front right: ' + data.data)
    pub = rospy.Publisher('front_right_sensor', Int8, queue_size=10)

    if distance > config['proximity']['high'] and not sensors['front_right']:
        sensors['front_right'] = True
        pub.publish(True)
    #     signalGo = True
    #     rightFrontSensor = False
    #     if corner.isSet():
    #         corner.clear()

    elif distance >= config['proximity']['mid'] and sensors['front_right']:
        sensors['front_right'] = False
        pub.publish(False)
    #     signalGo = True
    #     rightFrontSensor = False
    #     if corner.isSet():
    #         corner.clear()

    elif distance <= config['proximity']['low'] and sensors['front_right']:
        sensors['front_right'] = False
        pub.publish(False)
    #     signalGo = False
    #     rightFrontSensor = True

def handle_right(data):
    distance = float(data.data)
    rospy.loginfo('right: ' + data.data)
    pub = rospy.Publisher('right_sensor', Int8, queue_size=10)

    if distance >= config['proximity']['mid'] and sensors['right']:
        sensors['right'] = False
        pub.publish(False)
    #     rightSensor = False

    elif distance <= config['proximity']['low'] and not sensors['right']:
        sensors['right'] = True
        pub.publish(True)
    #     rightSensor = True
    #     lookingFromRight = True

if __name__ == '__main__':

    try:
        rospy.init_node('infared', anonymous=True)
        rospy.Subscriber('ir_' + config['ports']['left'], String, handle_left)
        rospy.Subscriber('ir_' + config['ports']['front_left'], String, handle_front_left)
        rospy.Subscriber('ir_' + config['ports']['front_right'], String, handle_front_right)
        rospy.Subscriber('ir_' + config['ports']['right'], String, handle_right)

        rospy.spin()

    except Exception as ex:
        print(str(ex))