#!/usr/bin/env python

"""Read image frame from integrated webcamera.
"""

# Standard import
import time

# Dependency imports
import rospy

import imutils
import cv2

class WebCamera(object):
    """Main class."""

    def __init__(self, camera_id=None, warm_up_time=2.5):
        # initialize the camera and grab a reference to the raw camera capture
        # video_capture = cv2.VideoCapture('/dev/video0')
        if camera_id is not None:
            self.video_capture = cv2.VideoCapture(camera_id)
        else:
            self.video_capture = cv2.VideoCapture(0)

        # allow the camera to warmup, then initialize the average frame, last
        # uploaded timestamp, and frame motion counter
        rospy.loginfo('Wait for webcamera to warm up...')
        time.sleep(warm_up_time)

    def grab_frame(self, width=None):
        """Run programs main loop."""
        # grab the raw NumPy array representing the image and initialize
        # the timestamp and occupied/unoccupied text
        _, frame = self.video_capture.read()

        # resize the frame, convert it to grayscale, and blur it
        if width is not None:
            frame = imutils.resize(frame, width=width)

        return frame
