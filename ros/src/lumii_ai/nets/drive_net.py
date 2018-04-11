"""Sequence Feature Generation model.

MaxoutCNN Neural Network.
"""

# Standard imports
import logging

# Dependency imports
import rospy

import tensorflow as tf
import tensorflow.contrib.slim as slim  # pylint: disable=E0611


def build(input_shape, dtype, classes):
    """Maxout CNN Network.

    Input :
        extract_features: Boolean. If True then return features from 4th
        convolution layer with size output_size
        *output_size: must be greater than batch_size. Will padd with 0 to
        get this size.

    """

    ret = {}

    # Input data.

    inputs = tf.placeholder(shape=[None] + input_shape, name='Input', dtype=dtype)
    ret['inputs'] = inputs

    rospy.loginfo('inputs %s', inputs.get_shape().as_list())

    net = slim.conv2d(
        inputs=inputs,
        num_outputs=24,
        kernel_size=[5, 5],
        stride=[2, 2],
        padding='VALID',
        activation_fn=tf.nn.elu,
        weights_initializer=tf.truncated_normal_initializer(mean=0, stddev=0.01)
    )
    rospy.loginfo('net %s', net.get_shape().as_list())

    net = slim.conv2d(
        inputs=net,
        num_outputs=36,
        kernel_size=[5, 5],
        stride=[2, 2],
        padding='VALID',
        activation_fn=tf.nn.elu,
        weights_initializer=tf.truncated_normal_initializer(mean=0, stddev=0.01)
    )
    rospy.loginfo('net %s', net.get_shape().as_list())

    net = slim.conv2d(
        inputs=net,
        num_outputs=48,
        kernel_size=[5, 5],
        stride=[2, 2],
        padding='VALID',
        activation_fn=tf.nn.elu,
        weights_initializer=tf.truncated_normal_initializer(mean=0, stddev=0.01)
    )
    rospy.loginfo('net %s', net.get_shape().as_list())

    net = slim.conv2d(
        inputs=net,
        num_outputs=64,
        kernel_size=[3, 3],
        stride=[1, 1],
        padding='VALID',
        activation_fn=tf.nn.elu,
        weights_initializer=tf.truncated_normal_initializer(mean=0, stddev=0.01)
    )
    rospy.loginfo('net %s', net.get_shape().as_list())

    net = slim.conv2d(
        inputs=net,
        num_outputs=64,
        kernel_size=[3, 3],
        stride=[1, 1],
        padding='VALID',
        activation_fn=tf.nn.elu,
        weights_initializer=tf.truncated_normal_initializer(mean=0, stddev=0.01)
    )
    rospy.loginfo('net %s', net.get_shape().as_list())

    net = slim.fully_connected(
        slim.flatten(net),
        100,
        activation_fn=tf.nn.elu,
        weights_initializer=tf.truncated_normal_initializer(mean=0, stddev=0.01)
    )
    rospy.loginfo('net %s', net.get_shape().as_list())

    net = slim.fully_connected(
        net,
        50,
        activation_fn=tf.nn.elu,
        weights_initializer=tf.truncated_normal_initializer(mean=0, stddev=0.01)
    )
    rospy.loginfo('net %s', net.get_shape().as_list())

    net = slim.fully_connected(
        net,
        10,
        activation_fn=tf.nn.elu,
        weights_initializer=tf.truncated_normal_initializer(mean=0, stddev=0.01)
    )
    rospy.loginfo('net %s', net.get_shape().as_list())

    # Output layers for policy and value estimations
    logits = slim.fully_connected(
        net,
        classes,
        activation_fn=None,
        weights_initializer=tf.truncated_normal_initializer(mean=0, stddev=0.01)
    )
    rospy.loginfo('logits %s', net.get_shape().as_list())

    ret['logits'] = logits
    rospy.loginfo('logits %s', logits.get_shape().as_list())

    ret['logits_softmax'] = tf.nn.softmax(logits)
    rospy.loginfo('logits softmax %s', ret['logits_softmax'].get_shape().as_list())

    return ret
