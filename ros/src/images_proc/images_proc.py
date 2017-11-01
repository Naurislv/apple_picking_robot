#!/usr/bin/env python
import time
import rospy

class ImagesProc(object):
	def __init__(self):
		rospy.init_node('image_proc')
		rospy.loginfo('starting image stream')
		imageStream = ' testing'

	def loop(self):
	       	rate = rospy.Rate(20) 
        	i = 0
        	while not rospy.is_shutdown():
            		i += 1
            		rospy.loginfo('image iter: %d', i)
            		rate.sleep()

if __name__ == '__main__':
    try:
        ImagesProc()
    except rospy.ROSInterruptException:
        rospy.logerr('Failed image processing node')
