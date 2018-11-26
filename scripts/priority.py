#!/usr/bin/env python

import rospy
from solar_buggy.msg import Pose
from std_msgs.msg import String

USE_GPS = True

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
        if not USE_GPS:
            self.cmd_publisher.publish(pose)
            return

        # Don't send a command if gps is not giving commands; i.e., if we don't have a waypoint or a fix
        if (self.stack[2][1] != 0):
            self.cmd_publisher.publish(pose)
        else:
            pose = Pose()
            pose.source = "Priority"
            pose.left_wheel_velocity = 0
            pose.right_wheel_velocity = 0
            self.cmd_publisher.publish(pose)
            
    def ultra_handler(self, pose):
        self.stack[0][1] = int(pose.status)

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
        self.stack[1][1] = int(pose.status)

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

                elif not USE_GPS:
                    pose.left_wheel_velocity = 24
                    pose.right_wheel_velocity = 24
                    self._publish_command(pose)

                return

    def gps_handler(self, pose):
        self.stack[2][1] = int(pose.status)

        # iterate through the stack
        for i in self.stack:

            # if any sensor with a higher priority has status not 0, return
            if i[0] != 'gps' and i[1] != 0:
                return

            # store sensor status and publish command if status is not 0
            elif i[0] == 'gps':
                if i[1] != 0:
                    self._publish_command(pose)

                # destination reached
                else:
                    pose.left_wheel_velocity = 0
                    pose.right_wheel_velocity = 0
                    self._publish_command(pose)

                return

if __name__ == '__main__':
    ps = PriorityStack()

    rospy.Subscriber('ultra_vel', Pose, ps.ultra_handler)
    rospy.Subscriber('cam_vel', Pose, ps.cam_handler)
    rospy.Subscriber('gps_vel', Pose, ps.gps_handler)

    rospy.spin()