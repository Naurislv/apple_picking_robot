"""Detect human hands in natural images."""

# Standard imports
import time
import sys

sys.path.append('./utils/')

# pylint: disable=C0413

# Dependecy imports
import tensorflow as tf
import numpy as np

# Local imports
from utils import label_map_util
from utils import visualization_utils as vis_util

# pylint: disable=E1101

class Localizator(object):
    """Tensorflow based hand detection using SSD and TF model API"""

    def __init__(self, frozen_graph_path, label_map_path):
        """Load tensorflow model for pb."""

        self._load_model(frozen_graph_path)

        label_map = label_map_util.load_labelmap(label_map_path)

        categories = label_map_util.convert_label_map_to_categories(
            label_map, max_num_classes=1000, use_display_name=True)
        self.category_index = label_map_util.create_category_index(categories)
        print(self.category_index)

    def run(self, image_array, class_prob):
        """Run hand detection."""

        image_np_expanded = np.expand_dims(image_array, axis=0)

        ret_tensors = [self.detection_boxes, self.detection_scores,
                       self.detection_classes, self.num_detections]

        # Actual detection.
        (boxes, scores, classes, _) = self.sess.run(
            ret_tensors, feed_dict={self.image_tensor: image_np_expanded})

        boxes = np.squeeze(boxes)
        classes = np.squeeze(classes).astype(np.int32)
        scores = np.squeeze(scores)

        classes = classes[scores > class_prob]
        boxes = boxes[scores > class_prob]

        return boxes, classes, scores

    def run_with_boxes(self, image_array, class_prob):
        """Detect object and add them as rectangles on image."""
        boxes, classes, scores = self.run(image_array, class_prob)

        vis_util.visualize_boxes_and_labels_on_image_array(
            image_array,
            boxes,
            classes,
            scores,
            self.category_index,
            min_score_thresh=class_prob,
            use_normalized_coordinates=True,
            line_thickness=2
        )

        nobjects = classes[scores > class_prob]

        return image_array, nobjects

    def _load_model(self, frozen_model_path):
        detection_graph = tf.Graph()

        with detection_graph.as_default(): # pylint: disable=E1129

            od_graph_def = tf.GraphDef()

            with tf.gfile.GFile(frozen_model_path, 'rb') as fid:

                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

        self.sess = tf.Session(graph=detection_graph)

        # Definite input and output Tensors for detection_graph
        self.image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')

        # Each box represents a part of the image where a particular object was detected.
        self.detection_boxes = detection_graph.get_tensor_by_name('detection_boxes:0')

        # Each score represent how level of confidence for each of the objects.
        # Score is shown on the result image, together with the class label.
        self.detection_scores = detection_graph.get_tensor_by_name('detection_scores:0')
        self.detection_classes = detection_graph.get_tensor_by_name('detection_classes:0')
        self.num_detections = detection_graph.get_tensor_by_name('num_detections:0')
