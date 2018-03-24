#! /usr/bin/env python

# Standard imports
import os
import time

# Local imports
from object_localization import Localizator

# Dependecy imports
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class SceneUnderstanding(object):
    """Class rensponsible for understanding are from input sensors such as camera."""

    def __init__(self):
        rospy.loginfo('Starting object scene understanding node')
        rospy.init_node('secene_understanding', anonymous=True)

        # Define user configurable parameters
        frozen_graph_path = rospy.get_param(
            '/frozen_graph_path', './tf_model/frozen_inference_graph.pb')
        label_map_path = rospy.get_param('/label_map_path', './tf_model/mscoco_label_map.pbtxt')
        self.class_prob = rospy.get_param('/class_prob', 0.8)

        if not os.path.exists(frozen_graph_path):
            down_link = ("https://github.com/tensorflow/models/blob/master/research/"
                         "object_detection/g3doc/detection_model_zoo.md")

            rospy.logerr('Frozen graph does not exist: %s', frozen_graph_path)
            rospy.logerr("You can download it here: %s and place it in tf_model with "
                         "frozen_inference_graph.pb name", down_link)

            rospy.signal_shutdown("Frozen graph does not exist!")

        self.bridge = CvBridge() # Convert ROS msg to numpy array
        self.detector = Localizator(
            frozen_graph_path,
            label_map_path,
        )

        self.im_pub = rospy.Publisher('imgdecor', Image, queue_size=1)

        rospy.Subscriber('imagedata', Image, callback=self.image_callback)
        rospy.spin()

    def image_callback(self, msg):
        """Callback wich initializes when imagedata publish it's data to our created Subscriber."""

        image_array = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        rospy.logdebug('Recieved image data is: %s %s', image_array.shape, image_array.dtype)

        dec_st = time.time()
        self.decorate_image(image_array)
        dec_et = time.time()
        rospy.loginfo('Decoration time: %.3f', dec_et - dec_st)

    def decorate_image(self, image):
        """Localize objects in image and decorate image with those boxes, class names
        and probabilties."""

        image_array = self.detector.run_with_boxes(image, self.class_prob)
        rospy.logdebug('Decorated image: %s', image_array.shape)

        image_message = self.bridge.cv2_to_imgmsg(image_array, encoding="rgb8")
        self.im_pub.publish(image_message)

if __name__ == '__main__':
    try:
        SceneUnderstanding()
    except rospy.ROSInterruptException:
        rospy.logerr('Failed scene understanding node')
