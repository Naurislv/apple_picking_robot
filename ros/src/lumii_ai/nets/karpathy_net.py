"""Sequence Feature Generation model.

MaxoutCNN Neural Network.
"""

import logging
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

    logging.debug('inputs %s', inputs.get_shape().as_list())

    net = slim.fully_connected(
        slim.flatten(inputs),
        50,
        # activation_fn=_tf_selu,
        activation_fn=tf.nn.relu,
        weights_initializer=tf.contrib.layers.xavier_initializer(uniform=True, seed=1)
    )

    # Output layers for policy and value estimations
    logits = slim.fully_connected(
        net,
        classes,
        activation_fn=None,
        weights_initializer=tf.contrib.layers.xavier_initializer(uniform=True, seed=1)
    )

    ret['logits'] = logits
    logging.debug('logits %s', logits.get_shape().as_list())

    ret['logits_softmax'] = tf.nn.softmax(logits)

    return ret
