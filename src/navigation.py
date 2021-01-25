#!/usr/bin/env python

import os
import time
import rospy
from geometry_msgs.msg import PoseStamped


class Navigation:
    def __init__(self):
        self.__saved_dir = os.getcwd() + '/../target.txt'
        self.__odom = {}
        self.getDestination()
        self.pub_goal = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=1)
        self.goal = PoseStamped()
        self.navi()

    # Get destination from text file
    def getDestination(self):
        file = open(self.__saved_dir, 'r')
        data = file.read()
        file.close()

        self.__odom = data.split("%")
        self.__odom = [v for v in self.__odom if v]
        print("""-----------------Target Odometry-----------------
position x : %s, y : %s, z : %s
orientation x : %s, y : %s, z : %s, w : %s
        """ % (self.__odom[0], self.__odom[1], self.__odom[2], self.__odom[3],
              self.__odom[4], self.__odom[5], self.__odom[6]))

    # Publish goal position via ros topic
    def navi(self):
        self.goal = PoseStamped()

        self.goal.header.frame_id = "map"
        self.goal.header.stamp = rospy.Time.now()

        self.goal.pose.position.x = float(self.__odom[0])
        self.goal.pose.position.y = float(self.__odom[1])
        self.goal.pose.position.z = 0.0

        self.goal.pose.orientation.x = 0.0
        self.goal.pose.orientation.y = 0.0
        self.goal.pose.orientation.z = float(self.__odom[5])
        self.goal.pose.orientation.w = float(self.__odom[6])

        time.sleep(1)
        self.pub_goal.publish(self.goal)
        rospy.loginfo("goal published")
        rospy.signal_shutdown("goal published")


if __name__ == "__main__":
    rospy.init_node('target_pos_pub', anonymous=True)
    navi = Navigation()
    rospy.spin()
