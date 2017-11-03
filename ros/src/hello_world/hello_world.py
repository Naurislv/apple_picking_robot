#!/usr/bin/env python

"""Simple ROS Catkin workspace package (node) example."""

# Standard imports
import time

# Dependency imports
import rospy

class HelloWorld(object):
    """Traffic Light detection and classification. Results publishing to ROS nodes."""

    def __init__(self):
        rospy.init_node('hello_world')
        rospy.loginfo('Init hello_world')
        print('Hello Apple Picking Team. Congratulations for running ROS project for first time.')

        self.loop()

    def loop(self):
        """
            Loop function to publish waypoints for car to follow
        :return:
        """
        rate = rospy.Rate(10) # 10Hz

        i = 0
        #while not rospy.is_shutdown():
        #    i += 1
        #    rospy.loginfo('Loop iter: %d', i)
        #    rate.sleep()

if __name__ == '__main__':
    try:
        HelloWorld()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start hello_world node.')
