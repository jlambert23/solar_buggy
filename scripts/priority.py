#!/usr/bin/env python

import rospy
from solar_buggy.msg import Pose
from solar_buggy.srv import Controller
from std_msgs.msg import String
import heapq

priority = {'ultra_cmd': 1, 'cam_cmd': 2, 'gps_cmd': 3}

''' We're using a MinHeap y'all! '''
class PriorityStack:

    def __init__(self):
        rospy.init_node('priority', anonymous=True)

        self.control_pub = rospy.Publisher('cmd_vel', Pose, queue_size=10)
        self.heap = []
    
    def _send_command(self):
        _, cmd = heapq.heappop(self.heap)
        self.control_pub.publish(cmd)

    def ultra_handler(self, pose):
        
        # Remove and replace command in heap
        self.heap = [i for i in self.heap if x[0] == priority['ultra_cmd']]
        heapq.heappush(self.heap, (priority['ultra_cmd'], pose))

        self._send_command()

    def cam_handler(self, pose):
        
        # Remove and replace command in heap
        self.heap = [i for i in self.heap if x[0] == priority['cam_cmd']]
        heapq.heappush(self.heap, (priority['cam_cmd'], pose))

        self._send_command()

    def gps_handler(self, pose):
        
        # Remove and replace command in heap
        self.heap = [i for i in self.heap if x[0] == priority['gps_cmd']]
        heapq.heappush(self.heap, (priority['gps_cmd'], pose))
        
        self._send_command()

if __name__ == '__main__':
    ps = PriorityStack()

    rospy.Subscriber('ultra_vel', Pose, ps.ultra_handler)
    rospy.Subscriber('cam_vel', Pose, ps.cam_handler)
    rospy.Subscriber('gps_vel', Pose, ps.gps_handler)

    rospy.spin()