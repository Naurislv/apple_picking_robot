#!/usr/bin/env python
import time
import rospy
import numpy as np
import cv2
from std_msgs.msg  import Int32

#TEST_IMAGE_NAME = '/home/robond/Downloads/apple_picking_robot/ros/src/images_proc/incoming/apple1.jpeg'
TEST_IMAGE_NAME = 'incoming/apple1.jpeg'
#rospy.init_node('imagePublisher'

class ImagesProc(object):
    height =0
    width = 0
    def __init__(self):
        rospy.init_node('image_proc')
        rospy.loginfo('starting image stream')
        imageStream = cv2.imread(TEST_IMAGE_NAME)
        if imageStream is not None:
            self.height, self.width = imageStream.shape[:2]
	    rospy.loginfo(' found image with height ; %d , width: %d ',self.height, self.width)
        else:
            rospy.loginfo(' -------------- Image not read in')

        self.image_pub = rospy.Publisher('imagedata',Int32, queue_size=3 )
        self.image_pub.publish(self.height)
        self.image_pub.publish(self.width)
        self.loop()

    def loop(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            self.image_pub.publish(self.height)
 	    rate.sleep()

if __name__ == '__main__':
    try:
        ImagesProc()
    except rospy.ROSInterruptException:
        rospy.logerr('Failed image processing node')
