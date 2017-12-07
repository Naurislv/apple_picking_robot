#! /usr/bin/env python

# Local imports
from object_localization import Localizator

# Dependecy imports
import rospy
from std_msgs.msg
import Int32


class SceneUnderstanding(object):

    def __init__(self):
        rospy.loginfo('Starting object scene understanding node')
        rospy.init_node('secene_understanding', anonymous=True)

        frozen_graph_path = rospy.get_param('/frozen_graph_path', './tf_model/frozen_inference_graph.pb')
        label_map_path = rospy.get_param('/label_map_path', './tf_model/mscoco_label_map.pbtxt')

        if not os.path.exists(frozen_graph_path):
            down_link = ("https://github.com/tensorflow/models/blob/master/research/"
                         "object_detection/g3doc/detection_model_zoo.md")

            rospy.warning('Frozen graph does not exist: %s', frozen_graph_path)
            rospy.warning("You can download it here: %s and place it in tf_model with "
                          "frozen_inference_graph.pb name", down_link)
            
            rospy.signal_shutdown("Frozen graph does not exist!")

        self.detector = Localizator(
            frozen_graph_path,
            label_map_path,
        )

        rospy.Subscriber('imagedata', Int32, callback=self.image_callback)

    def image_callback(self, msg):
        """Callback wich initializes when imagedata publish it's data to our created Subscriber."""

        rospy.loginfo('Recieved image data is: %s',  msg.data.shape)

        self.decorate_image(msg.data)
    
    def decorate_image(self, image)
        image_array, nobjects = self.detector.run_with_boxes(image)
        rospy.loginfo(('Decorated image: %s', image_array.shape)
        

if __name__ == '__main__':
    try:
        SceneUnderstanding()
    except rospy.ROSInterruptException:
        rospy.logerr('Failed image processing node')
