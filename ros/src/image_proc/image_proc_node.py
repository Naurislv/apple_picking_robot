#!/usr/bin/env python

# Standard imports
import time

# Dependecy imports
import rospy
from std_msgs.msg  import Int32

import numpy as np
import cv2

# Local imports
from webcamera_driver import WebCamera

class ImageProc(object):
    """Porcessing images from input pipeline and publish it to node."""

    def __init__(self):
        rospy.init_node('image_proc')

        self.im_stream = WebCamera()

        if self.im_stream is not None:
            rospy.loginfo('Found image stream with')
        else:
            rospy.loginfo('Cannot read image')

        self.im_pub = rospy.Publisher('imagedata', Int32, queue_size=3)
        self.loop()

    def loop(self):
        """Create ROS loop to publish image in predefined rate"""

        # TODO: Create similarrly rospy.set_param somewhere
        rate = rospy.Rate(rospy.get_param('/image_refresh_rate', 5))
        while not rospy.is_shutdown():
            im = self.im_stream.grab_frame(width=400)

            self.im_pub.publish(im)
            rate.sleep()

if __name__ == '__main__':
    try:
        rospy.loginfo('Starting image processing node')
        ImageProc()
    except rospy.ROSInterruptException:
        rospy.logerr('Failed image processing node')
