#! /usr/bin/env python
"""
API implementation for training robot in Gazebo.
Functionality is based on OpenAI gym so code could be resued.

Note that this API will only work if placed in ROS directory together with Gazebo virtual environment.

Learn more about OpenAI Gym: https://gym.openai.com/
More about what is LUMII: http://www.lumii.lv/
"""

# Standard imports
import logging
import time

# Dependency imports
import rospy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class LumiiGym(object):
    """LUMII Gym API. Based on OpenAI Gym API."""

    def __init__(self):

        self.action_space = Discrete(['up', 'down', 'left', 'right'])
        self.observation_space = Box('/camera1/image_raw')

    def reset(self):
        """Reset VM to beginning."""

        return self.observation_space.sample()

    def step(self, action):
        """Make one step in VM."""

        return self.observation_space.sample(), float(1), False, None

class Discrete(object):
    """Action space class."""

    def __init__(self, spaces):
        self.spaces = spaces

        self.n = len(self.spaces) # Number of discrite actions

    def sample(self):
        """Sample action space sample."""
        return 0

    def __repr__(self):

        return 'Discrete(%d)' % len(self.spaces)

class Box(object):
    """Observation space type for image."""

    def __init__(self, img_ros_topic):
        self.image_array = None
        self.im_shape = None

        self.nb_im_received = 0
        self.nb_im_sent = 0

        self._bridge = CvBridge() # Convert ROS msg to numpy array

        rospy.Subscriber(img_ros_topic, Image, callback=self._image_callback)

        # print('Spin start')
        # rospy.spin()
        # print('Spin end')

    def sample(self):
        """Sample observation space sample."""

        # Dont return image until it's not received rom subscriber
        while self.nb_im_sent == self.nb_im_received:
            time.sleep(0.001)

        self.nb_im_sent = self.nb_im_received

        return self.image_array

    def _image_callback(self, msg):
        """Callback wich initializes when imagedata publish it's data to our created Subscriber."""

        self.image_array = self._bridge.imgmsg_to_cv2(msg, "bgr8")
        self.im_shape = self.image_array.shape

        self.nb_im_received += 1

    def __repr__(self):

        return 'Box%s' % self.im_shape


def make(env_name):
    """Prepare environment for training."""

    logging.info('Making environment: %s', env_name)

    return LumiiGym()
