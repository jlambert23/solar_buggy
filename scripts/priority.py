#!/usr/bin/env python

import rospy
from solar_buggy.msg import Pose
from solar_buggy.srv import Controller
from std_msgs.msg import String
import heapq

priority = {'ultrasonic': 1, 'camera': 2, 'gps': 3}

''' We're using a MinHeap y'all! '''
class PriorityStack:

    def __init__(self):
        rospy.init_node('priority', anonymous=True)
        
        self.cmd_publisher = rospy.Publisher('cmd_vel', Pose, queue_size=10)

        # Ordered by highest priority
        self.stack = [
            ['ultrasonic', 0],
            ['camera', 0],
            ['gps', 0]
        ]        

    def _publish_command(self, pose):
        self.cmd_publisher.publish(pose)
            
    def ultra_handler(self, pose):
        
        # iterate through the stack
        for i in self.stack:

            # if any sensor with a higher priority has status not 0, return
            if i[0] != 'ultrasonic' and i[1] != 0:
                return

            # store sensor status and publish command if status is not 0
            elif i[0] == 'ultrasonic':
                i[1] = int(pose.status)

                if i[1] != 0:
                    self._publish_command(pose)

                return

    def cam_handler(self, pose):
            
        # iterate through the stack
        for i in self.stack:

            # if any sensor with a higher priority has status not 0, return
            if i[0] != 'camera' and i[1] != 0:
                return

            # store sensor status and publish command if status is not 0
            elif i[0] == 'camera':
                i[1] = int(pose.status)

                if i[1] != 0:
                    self._publish_command(pose)

                return

    def gps_handler(self, pose):
        
        # iterate through the stack
        for i in self.stack:

            # if any sensor with a higher priority has status not 0, return
            if i[0] != 'gps' and i[1] != 0:
                return

            # store sensor status and publish command if status is not 0
            elif i[0] == 'gps':
                i[1] = int(pose.status)

                if i[1] != 0:
                    self._publish_command(pose)

                return


if __name__ == '__main__':
    ps = PriorityStack()

    rospy.Subscriber('ultra_vel', Pose, ps.ultra_handler)
    rospy.Subscriber('cam_vel', Pose, ps.cam_handler)
    rospy.Subscriber('gps_vel', Pose, ps.gps_handler)

    rospy.spin()