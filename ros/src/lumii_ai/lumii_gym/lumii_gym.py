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
from random import randrange
import sys

# Dependency imports
import rospy

from gazebo_msgs.msg import ContactsState
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

sys.path.append('../robot_control/')

# Local imports
from robot_control import RobotControl # pylint: disable=E0401,C0413

class LumiiGym(RobotControl):
    """LUMII Gym API. Based on OpenAI Gym API."""

    def __init__(self):

        RobotControl.__init__(self) # Initialize RobotControl

        self.action_space = Discrete(['i', 'j', 'l', 'p'])
        self.observation_space = Box('iRobot/camera/image_raw')

        rospy.Subscriber('/irobot_bumper', ContactsState, callback=self._bumper_callback)

        # Count steps, so we can know when game is Done (dont play forever)
        self.nb_steps = 300
        self.is_bumper_triggered = False
        self.step_counter = self.nb_steps

        # Step counter
        self._step_counter = self.nb_steps

    def reset(self):
        """Reset VM to beginning."""

        self.env_reset()
        self.step_counter = self.nb_steps
        self.is_bumper_triggered = False

        return self.observation_space.sample()

    def step(self, action_idx):
        """Make one step in VM.

        action: index of valid action space
        """

        env_binding = self.action_space.env_binding(action_idx)

        step_feedback = self.env_step(env_binding)
        # step_feedback = self.env_step(None)
        # time.sleep(0.5)
        reward, done = self._calc_reward(step_feedback)

        return self.observation_space.sample(), reward, done, 0

    def _bumper_callback(self, msg):
        """Callback for bumper sensor."""

        if len(msg.states) > 2:
            states = []
            for state in msg.states:
                states.append(state.collision1_name)

            # 'cricket_ball_1::link::collision'
            if not any("cricket_ball" in state for state in states):
                self.is_bumper_triggered = True

    def _calc_reward(self, step_feedback):
        """Calculate single step reward."""

        done_reason = None
        reward = 0
        done = False

        self.step_counter -= 1
        if self.step_counter <= 0:
            done_reason = 'step_count'
            done = True
        elif self.is_bumper_triggered and self.nb_steps - self.step_counter > 2:
            done_reason = 'bumper_triggered'
            done = True
        else:
            self.is_bumper_triggered = False

        if step_feedback['tried_pickup'] and step_feedback['done_pickup']:
            reward += 10

        # reward += step_feedback['dist_towrds_apple']
        # reward = reward + 10 * step_feedback['dist_traveled']
        # reward = reward + 200 * step_feedback['dist_traveled_from_o']
        # reward -= 1

        if done:
            rospy.logwarn('Environment is done. Reason: %s', done_reason)

        return float(reward), done

class Discrete(object):
    """Action space class."""

    def __init__(self, spaces):
        self.spaces = spaces

        self.n = len(self.spaces) # Number of discrite actions

    def sample(self):
        """Sample action space sample."""
        return randrange(0, self.n)

    def env_binding(self, idx):
        """Translate action index to action for environment."""
        return self.spaces[idx]

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
