"""Sequence Feature Generation model.

MaxoutCNN Neural Network.

Based on: https://github.com/wagonhelm/Deep-Policy-Gradient/blob/master/PolicyGradDoom.ipynb
"""

# Standard imports
import logging

# Dependency imports
import rospy

import tensorflow as tf


def build(input_shape, dtype, classes):
    """Maxout CNN Network.

    Input :
        extract_features: Boolean. If True then return features from 4th
        convolution layer with size output_size
        *output_size: must be greater than batch_size. Will padd with 0 to
        get this size.

    """

    convs = [16, 32]
    kerns = [8, 8]
    strides = [4, 4]
    f_c = 256
    pads = 'valid'
    activ = tf.nn.elu

    ret = {}
    # Input data.

    inputs = tf.placeholder(shape=[None] + input_shape, name='Input', dtype=dtype)
    ret['inputs'] = inputs
    rospy.loginfo('inputs %s', ret['inputs'].get_shape().as_list())

    net = tf.layers.conv2d(
        inputs=ret['inputs'],
        filters=convs[0],
        kernel_size=kerns[0],
        strides=strides[0],
        padding=pads,
        activation=activ,
        name='conv1'
    )
    rospy.loginfo('net %s', net.get_shape().as_list())

    net = tf.layers.conv2d(
        inputs=net,
        filters=convs[1],
        kernel_size=kerns[1],
        strides=strides[1],
        padding=pads,
        activation=activ,
        name='conv2'
    )
    rospy.loginfo('net %s', net.get_shape().as_list())

    flat = tf.layers.flatten(net)
    rospy.loginfo('flat %s', flat.get_shape().as_list())

    net = tf.layers.dense(
        inputs=flat,
        units=f_c,
        activation=activ,
        name='fc'
    )
    rospy.loginfo('net %s', net.get_shape().as_list())

    logits = tf.layers.dense(
        inputs=net,
        units=classes,
        name='logits'
    )
    rospy.loginfo('logits %s', logits.get_shape().as_list())

    value = tf.layers.dense(
        inputs=net,
        units=1,
        name='value'
    )
    rospy.loginfo('value %s', value.get_shape().as_list())

    ret['logits'] = logits
    ret['logits_softmax'] = tf.nn.softmax(logits)
    ret['logits_logsoftmax'] = tf.nn.log_softmax(logits)
    ret['value'] = value

    return ret
