#! /usr/bin/env python

"""Train robot to spot apple and pick it up.

Deep Reinforcement Learning using LUMII Gym as environment.

Insipred from :
    http://karpathy.github.io/2016/05/31/rl/
    https://gist.github.com/karpathy/a4166c7fe253700972fcbc77e4ea32c5

"""

# Standard imports
import time
import os
import sys

# Dependency imports
import rospy

import imageio
import tensorflow as tf
import numpy as np
import cv2

sys.path.append('../../robot_control/')

# Local imports
import lumii_gym as gym # pylint: disable=E0401,C0413
from policy_gradient import Policy # pylint: disable=E0401,C0413

# Create tensorflow flags for input arguments
_FLAGS = tf.app.flags
FLAGS = _FLAGS.FLAGS

_FLAGS.DEFINE_boolean('video_small', False, "Whether to record and save video or not. Boolean.")
_FLAGS.DEFINE_boolean('video_big', False, "Whether to record and save video or not. Gym built "
                                          "in command. Also create report .json files. Boolean.")
_FLAGS.DEFINE_boolean('gpu', False, "Use GPU.")
_FLAGS.DEFINE_string('name', 'test_0',
                     'Name of run, will be used to save and load checkpoint and statistic files.')
_FLAGS.DEFINE_integer('nb_episodes', 100000000, 'number of episodes to run')
_FLAGS.DEFINE_integer('checkpoint_steps', 300, 'After how many episodes to save checkpoint')
_FLAGS.DEFINE_integer('batch_size', 4, 'After how many episodes to save checkpoint')

class GymEnv(object):
    """OpenAI Gym Virtual Environment - setup."""

    def __init__(self, env_name, policy):
        self.data_type = {'tf': tf.float32, 'np': np.float32}
        self.no_ep_load = 0  # number of episodes load from statistics to be displayed

        # To store data and make plots later
        self.episdod_reward = []  # episode reward sum
        self.no_frames = []  # number of frames in all episode

        # Create gym environment for CarRacing-v0
        self.env = gym.make(env_name)
        rospy.loginfo("%s initialized", env_name)

        if FLAGS.video_small:
            self.writer = imageio.get_writer('{}.mp4'.format(env_name), mode='I')

        self.n_actions = self.env.action_space.n
        rospy.loginfo('Action Space %s total of %d actions',
                      self.env.action_space, self.n_actions)
        rospy.loginfo([func for func in dir(self.env.action_space) if '__' not in func])
        rospy.loginfo('Observation Space %s', self.env.observation_space)
        rospy.loginfo([func for func in dir(self.env.observation_space) if '__' not in func])

        for _ in range(10):
            rospy.loginfo('AS sample: %s', self.env.action_space.sample())

        # Observation sapce shape after preprocessing
        # os_sample_shape = self.prepro(self.env.observation_space.sample(), None)[0].shape
        os_sample_shape = self.prepro(self.env.observation_space.sample()).shape
        rospy.loginfo("Observation sample shape after preprocess: %s", os_sample_shape)

        if FLAGS.gpu:
            device = '/gpu:0'
        else:
            # Tensorflow somehow allocated GPU memory even though used /cpu:o
            # This masks GPU device from tensorflow.
            # https://stackoverflow.com/questions/44500733/tensorflow-allocating-
            # gpu-memory-when-using-tf-device-cpu0
            os.environ["CUDA_VISIBLE_DEVICES"] = "-1"
            device = '/cpu:0'

        rospy.loginfo("Setting up %s TF device for training", device)

        with tf.device(device):
            self.policy = policy(list(os_sample_shape),
                                 self.data_type,
                                 self.n_actions)
            self.policy.define('PolicyGradient')

    def run(self, run_name):
        """Start loop."""

        if run_name == '':
            run_name = str(int(round(time.time() * 1000)))

        if not os.path.exists('outputs/' + run_name):
            rospy.loginfo('Creating outputs/' + run_name)
            os.makedirs('outputs/' + run_name)

        chk_file = 'outputs/' + run_name + '/models'
        statistics_file = 'outputs/' + run_name + '/statistics.npy'

        # Try to load tensorflow model
        try:
            self.policy.load(chk_file)
        except tf.errors.NotFoundError:
            rospy.logwarn("Checkpoint Not Found: %s", chk_file)
            self.policy.init_weights()

        try: # Try to load statistics
            statistics = np.load(statistics_file)
            self.no_ep_load = len(statistics[0])
            rospy.loginfo("Statistics loaded %s", statistics.shape) # pylint: disable=E1101
        except EnvironmentError:
            rospy.logwarn("Statistics Not Found: %s", statistics_file)

        self._run(chk_file, statistics_file)

    # def prepro(self, img, prev_img):
    def prepro(self, img):
        """ prepro 77x102x3 uint8 frame into 6400 (80x80) 1D float vector """
        # Resize image using interpolation
        img = cv2.resize(img, None, fx=0.16, fy=0.16, interpolation=cv2.INTER_AREA)
        img = img.astype(self.data_type['np'])
        # Standardize image so it would be in range -0.5 .. 0.5
        img = (img - 127.5) / 255

        # Insert motion in frame by subtracting previous frame from current
        # if prev_img is not None:
            # absdiff does not give different colors if differences
            # so neural network can't distinguish betwwen previous and current frame
            # policy_input = cv2.absdiff(img, prev_img)

        #     policy_input = img - prev_img
        # else:
        #     policy_input = np.zeros_like(img)

        # prev_img = img

        # print(policy_input.shape, prev_img.shape)
        # cv2.imwrite('save_im/{}.png'.format(str(int(round(time.time() * 1000)))),
        #             np.concatenate((prev_img, policy_input), axis=1) * 255 + 127.5)

        # return policy_input, prev_img
        return img

    def discount_rewards(self, reward_his, gamma=.99, normal=True):
        """Returns discounted rewards
        Args:
            reward_his (1-D array): a list of `reward` at each time step
            gamma (float): Will discount the future value by this rate.
            You can place None value in reward_hist it will be considered as
            environment reset and discounted reward will be adjusted acordingly.
        Returns:
            discounted_r (1-D array): same shape as input `R`
                but the values are discounted
        Examples:
            >>> R = [1, 1, 1]
            >>> discount_rewards(R, .99) # before normalization
            [1 + 0.99 + 0.99**2, 1 + 0.99, 1]
        """

        assert isinstance(reward_his, np.ndarray), 'reward_his must be numpy ndarry type'

        none_idx = list(np.argwhere(reward_his == None).flatten()) # pylint: disable=C0121
        discounted_r = np.zeros_like(np.delete(reward_his, none_idx, None))
        running_add = 0.0

        for i in reversed(range(0, len(reward_his))):

            # reset the sum, since this was a game boundary
            if reward_his[i] is None:
                running_add = 0
                continue

            running_add = running_add * gamma + reward_his[i]
            discounted_r[i] = running_add

        # Normalize
        if normal:
            mean = np.mean(discounted_r)
            std = np.std(discounted_r) + 0.00000001
            discounted_r = (discounted_r - mean) / (std)

        print(discounted_r)

        return discounted_r

    def _save_checkpoint(self, chk_file, statistics_file):
        """Save model and statistics checkpoint."""

        statistics = np.array([np.array([]), np.array([])])

        # Save results
        dir_path = os.path.join(chk_file, str(time.time()).replace('.', ''))
        os.makedirs(dir_path)
        self.policy.save(dir_path + '/model')

        epr = np.concatenate((statistics[0], np.array(self.episdod_reward)), axis=0)
        nfr = np.concatenate((statistics[1], np.array(self.no_frames)), axis=0)

        statistics = np.array([epr, nfr])

        np.save(statistics_file, statistics)
        rospy.logwarn("Statistics saved %s %s", statistics_file, statistics.shape)

    def _run(self, chk_file, statistics_file):
        """Run virtual environment loop."""

        # 1 episode is multiple games (until game env responde with done == True)
        episode_number = 1
        n_frames = 1  # frames per episode
        prev_img = None  # Previous image

        reward_sum = 0

        # Used for training after episode
        reward_his = []  # save rewards
        action_his = []  # action history
        obs_his = []  # save eposiodes

        observation = self.env.reset()

        start_time = time.time()
        train_time = start_time

        while episode_number <= FLAGS.nb_episodes:

            # preprocess the observation, set input to network to be difference image
            # policy_input, prev_img = self.prepro(observation, prev_img)
            policy_input = self.prepro(observation)

            action = self.policy.sample_action(policy_input)[0][0]

            # label = np.zeros((self.n_actions), dtype=self.data_type['np'])
            # label[action] = 1

            # step the environment and get new measurements
            observation, reward, done, _ = self.env.step(action)
            reward_sum += reward

            obs_his.append(policy_input)
            action_his.append(action)
            # record reward (has to be done after we call step() to get reward
            # for previous action)
            reward_his.append(reward)

            if n_frames % 20 == 0:
                end_time = time.time()

                fps = 20 / (end_time - start_time)
                rospy.loginfo("%s.[%s]. T[%.2fs] FPS: %.2f, Reward Sum: %s (%.1f)",
                              episode_number, self.no_ep_load, end_time - train_time,
                              fps, reward_sum, reward)
                start_time = time.time()

            if FLAGS.video_small:
                self.writer.append_data(observation)

            n_frames += 1

            if done:  # When Game env say it's done - end of episode.
                rospy.loginfo('')
                rospy.loginfo("Episode done! Reward sum: %.2f , Frames: %d",
                              reward_sum, n_frames)
                rospy.loginfo('')

                if episode_number % FLAGS.batch_size == 0:
                    # action_counter = [np.where(aprob == 1)[0][0] for aprob in action_his]
                    action_space = {i: 0 for i in range(self.n_actions)}
                    # for act in action_counter:
                    for act in action_his:
                        action_space[act] += 1

                    rospy.loginfo("Update weights from %d frames with average score: %s",
                                  len(reward_his), sum(reward_his) / FLAGS.batch_size)
                    rospy.loginfo("Used action space: %s", action_space)

                    # compute the discounted reward backwards through time
                    reward_his = self.discount_rewards(np.array(reward_his))
                    self.policy.fit(np.array(obs_his), np.stack(action_his), np.stack(reward_his))

                    # Reset history
                    reward_his = []  # save rewards
                    action_his = []  # action history
                    obs_his = []  # save eposiodes

                # Just for plotting
                self.episdod_reward.append(reward_sum)
                self.no_frames.append(n_frames)

                if episode_number % FLAGS.checkpoint_steps == 0:
                    self._save_checkpoint(chk_file, statistics_file)

                observation = self.env.reset()  # reset env
                reward_sum = 0
                n_frames = 1
                prev_img = None
                episode_number += 1
                reward_his.append(None) # None will indicate that environment have is done

                start_time = time.time()


if __name__ == '__main__':
    rospy.loginfo('Starting bot training node')
    rospy.init_node('bot training', anonymous=True)

    ENV = GymEnv('dbaby', Policy)
    ENV.run(run_name=FLAGS.name)
