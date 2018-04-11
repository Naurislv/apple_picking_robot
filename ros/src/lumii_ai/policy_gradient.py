"""Sample Tensorflow network for Policy Gradient.

Policy Gradient in TF:
    https://gist.github.com/shanest/535acf4c62ee2a71da498281c2dfc4f4
"""

# Standard imports
import logging

# Other imports
import rospy

import tensorflow as tf
import numpy as np
# from MaxoutNet import maxout_cnn as policy_net
# from nets import maxout_net as policy_net
# from nets import drive_net as policy_net
# from nets import karpathy_net as policy_net
from nets import small_conv_net as policy_net

TF_CONFIG = tf.ConfigProto()
TF_CONFIG.allow_soft_placement = True
TF_CONFIG.gpu_options.allocator_type = 'BFC'  # pylint: disable=E1101
# TF_CONFIG.gpu_options.per_process_gpu_memory_fraction = 0.8
TF_CONFIG.gpu_options.allow_growth = True  # pylint: disable=E1101
# TF_CONFIG.log_device_placement = True

class Policy(object):
    """Three Dense layers"""

    def __init__(self, state_shape, data_type, n_actions):
        """
        ARGS:
            state_shape: list of input shape for convolutional network e.g. [65, 65, 3]
            scope: variable scope, with this scope you will be able to access those variables

        """
        self.rmsprop_decay = 0.99
        self.learning_rate = 1e-3
        self.state_shape = state_shape

        self._sess = tf.Session(config=TF_CONFIG)

        self.net = None
        self.actions = None
        self.disc_reward = None
        self.sample = None
        self.loss = None
        self.train = None

        self.data_type = data_type
        self.n_actions = n_actions

    def define(self, scope):
        """Define network graph in a scope for Policy Gradient."""

        with tf.variable_scope(scope):  # pylint: disable=E1129
            rospy.loginfo('Building Network in %s scope', scope)

            self.net = policy_net.build(self.state_shape, self.data_type['tf'], self.n_actions)

            self.actions = tf.placeholder(dtype=tf.int32, shape=[None,], name='actions')
            self.disc_reward = tf.placeholder(dtype=self.data_type['tf'], shape=[None,], name='d_r')

            # Initialize Session
            sess = tf.Session()
            init = tf.global_variables_initializer()
            sess.run(init)

            # Samples an action from multinomial distribution
            # We need to use logits_softmax here istead of just logits because
            # all values equal and below will be zero and never will be sampled which
            # is not true for softmax
            self.sample = tf.multinomial(self.net['logits'], 1)

            # Define Losses
            pg_loss = tf.reduce_mean(
                (self.disc_reward - self.net['value']) *
                tf.nn.sparse_softmax_cross_entropy_with_logits(
                    logits=self.net['logits'], labels=self.actions))

            value_scale = 0.5
            value_loss = value_scale * tf.reduce_mean(
                tf.square(self.disc_reward - self.net['value']))

            entropy_scale = 0.00
            entropy_loss = -entropy_scale * tf.reduce_sum(
                self.net['logits_softmax'] * tf.exp(self.net['logits_softmax']))
            self.loss = pg_loss + value_loss - entropy_loss

            # surrogate loss
            # self.loss = - tf.reduce_sum(self.disc_reward *  # pylint: disable=E1130
            #                             self.actions *
            #                             tf.log(self.net['logits_softmax'] + 0.00001))

            # Add Optimizer
            alpha = 1e-4
            gradient_clip = 40

            optimizer = tf.train.AdamOptimizer(alpha)
            grads = tf.gradients(self.loss, tf.trainable_variables())
            grads, _ = tf.clip_by_global_norm(grads, gradient_clip) # gradient clipping
            grads_and_vars = list(zip(grads, tf.trainable_variables()))
            self.train = optimizer.apply_gradients(grads_and_vars)

            # update
            # optimizer = tf.train.RMSPropOptimizer(
            #     learning_rate=self.learning_rate,
            #     decay=self.rmsprop_decay
            # )

            # Discussions - how to reduce memory consumpsion in TF:
            # https://groups.google.com/a/tensorflow.org/forum/#!topic/discuss/q9bT3Ql2bYk
            # https://stackoverflow.com/questions/36194394/how-i-reduce-memory-consumption-in-a-loop-in-tensorflow
            # self.train = optimizer.minimize(self.loss)

    def init_weights(self):
        """Initialize TF variables."""

        logging.info('Initializing network Random variables')
        # Add an op to initialize the variables.
        init_op = tf.global_variables_initializer()
        self._sess.run(init_op)

    def fit(self, obs, actions, disc_reward):
        """Train neural network."""

        batch_feed = {self.net['inputs']: obs,
                      self.actions: actions,
                      self.disc_reward: disc_reward}

        _, loss = self._sess.run([self.train, self.loss], feed_dict=batch_feed)
        rospy.loginfo("Loss sum: %s", loss)

    def sample_action(self, observation):
        """Predict action from observation.

        Return index of action.
        """
        observation = np.expand_dims(observation, axis=0)

        return self._sess.run(self.sample, feed_dict={self.net['inputs']: observation})

    def save(self, path):
        """Save Tesnroflow checkpoint."""
        saver = tf.train.Saver()

        saver.save(self._sess, path)
        rospy.loginfo('Checkpoint saved %s', path)

    def load(self, path):
        """Save Tesnroflow checkpoint."""
        saver = tf.train.Saver()

        saver.restore(self._sess, path)
        rospy.loginfo('Variables loaded from checkpoint file: %s', path)
