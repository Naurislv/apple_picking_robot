"""Credits to https://github.com/swirlingsand

Bosch example:
https://github.com/swirlingsand/deeper-traffic-lights/blob/master/data_conversion_bosch.py

How to prepare inputs:
https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/preparing_inputs.md

In case to run this code follow instructions:
https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/running_locally.md
"""

import tensorflow as tf
import yaml
import os
from object_detection.utils import dataset_util
import json
import glob
import scipy.io
import numpy as np
import cv2

_FLAGS = tf.app.flags
_FLAGS.DEFINE_string('input_path', '', 'Path to .yaml logs')
_FLAGS.DEFINE_string('output_path', '', 'Path to output TFRecord')
FLAGS = _FLAGS.FLAGS

LABEL_DICT =  {
    "Apple" : 1
}

def create_tf_example(example, _file, counter):
    """Create tensorflow example."""

    # Bosch
    height = example['height'] # Image height
    width = example['width'] # Image width

    filename = example['path'] # Filename of the image. Empty if image is not from file
    filename = filename.encode()

    with tf.gfile.GFile(example['path'], 'rb') as fid:
        encoded_image = fid.read()

    image_format = 'png'.encode()

    xmins = [] # List of normalized left x coordinates in bounding box (1 per box)
    xmaxs = [] # List of normalized right x coordinates in bounding box
                # (1 per box)
    ymins = [] # List of normalized top y coordinates in bounding box (1 per box)
    ymaxs = [] # List of normalized bottom y coordinates in bounding box
                # (1 per box)
    classes_text = [] # List of string class name of bounding box (1 per box)
    classes = [] # List of integer class id of bounding box (1 per box)

    for box in example['boxes']:
        #if box['occluded'] is False:
        #print("adding box")
        xmins.append(float(box['x_min'] / width))
        xmaxs.append(float(box['x_max'] / width))
        ymins.append(float(box['y_min'] / height))
        ymaxs.append(float(box['y_max'] / height))
        classes_text.append(box['label'].encode())
        classes.append(int(LABEL_DICT[box['label']]))

        entry = "item {{\n  id: {}\n  name: '{}'\n}}\n\n".format(counter, box['label'])
        _file.write(entry)

        counter += 1

    tf_example = tf.train.Example(features=tf.train.Features(feature={
        'image/height': dataset_util.int64_feature(height),
        'image/width': dataset_util.int64_feature(width),
        'image/filename': dataset_util.bytes_feature(filename),
        'image/source_id': dataset_util.bytes_feature(filename),
        'image/encoded': dataset_util.bytes_feature(encoded_image),
        'image/format': dataset_util.bytes_feature(image_format),
        'image/object/bbox/xmin': dataset_util.float_list_feature(xmins),
        'image/object/bbox/xmax': dataset_util.float_list_feature(xmaxs),
        'image/object/bbox/ymin': dataset_util.float_list_feature(ymins),
        'image/object/bbox/ymax': dataset_util.float_list_feature(ymaxs),
        'image/object/class/text': dataset_util.bytes_list_feature(classes_text),
        'image/object/class/label': dataset_util.int64_list_feature(classes),
    }))

    return tf_example, counter

def read_annotations():
    # dir_path = '/home/nauris/Dropbox/coding/Valopaat'

    dataset_path = glob.glob('./data/hand_dataset/test_dataset/images/*.jpg')
    dataset_path += glob.glob('./data/hand_dataset/training_dataset/images/*.jpg')
    dataset_path += glob.glob('./data/hand_dataset/validation_dataset/images/*.jpg')

    print('Number of files found', len(dataset_path))
    for path in dataset_path:
        dir_path = '/'.join(os.path.dirname(path).split('/')[:-1])

        annot_file = dir_path + '/annotations/' + ''.join(os.path.basename(path).split('.')[:-1])
        im_file = dir_path + '/images/' + ''.join(os.path.basename(path).split('.')[:-1]) + '.jpg'
        annot = scipy.io.loadmat(annot_file + '.mat')

        example = {}
        bboxs = []

        for hand in annot['boxes'][0]:
            try:
                _ = hand[0][0][4][0] # Hand label: Left or right
            except IndexError:
                continue
            bbox = np.array([[hand[0][0][0][0][1], hand[0][0][0][0][0]],
                             [hand[0][0][1][0][1], hand[0][0][1][0][0]],
                             [hand[0][0][2][0][1], hand[0][0][2][0][0]],
                             [hand[0][0][3][0][1], hand[0][0][3][0][0]]], np.int32)

            box = {}
            box['x_min'] = np.min(bbox[:, 0])
            box['x_max'] = np.max(bbox[:, 0])
            box['y_min'] = np.min(bbox[:, 1])
            box['y_max'] = np.max(bbox[:, 1])
            box['label'] = 'Hand'

            bboxs.append(box)

        example['boxes'] = bboxs
        example['path'] = im_file

        img = cv2.imread(example['path'])
        example['height'] = img.shape[0]
        example['width'] = img.shape[1]

        yield example

def main(_):
    print(FLAGS.output_path)
    writer = tf.python_io.TFRecordWriter(FLAGS.output_path)

    counter = 1
    label_path = os.path.join(os.path.dirname(FLAGS.input_path), 'labels.pbtxt')

    iterator = read_annotations()

    with open(label_path, mode='a', encoding='utf-8') as _file:
        for i, example in enumerate(iterator):
            tf_example, counter = create_tf_example(example, _file, counter)
            writer.write(tf_example.SerializeToString())

            if i % 10 == 0:
                print("Done", i)

    writer.close()

if __name__ == '__main__':
    tf.app.run()
