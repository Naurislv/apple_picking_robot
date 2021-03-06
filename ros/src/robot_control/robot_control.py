#! /usr/bin/env python

"""
API implementation for controlling Gazebo apple picking robot.
"""

# Standard imports
import math
import sys
import select
import termios
import tty
import time

# Dependency imports
import roslib
import rospy
import random

from std_srvs.srv import Empty
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.srv import DeleteModel
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState

roslib.load_manifest('robot_control')

class Block(object):
    """Define block object."""

    def __init__(self, name, relative_entity_name):
        self.name = name
        self.relative_entity_name = relative_entity_name

class RobotControl(object):
    """Control robot in gazebo environment."""

    def __init__(self):
        self.help() # Print Robot Control infomration
        self.turtlebot = Block('iRobot', '')

        # Minimum distance to apple, where robot can pick up
        self.apple_distance = 0.3
        # Distance to travel in single step
        self.x_distance = 0.2
        self.a_distance = 0.25

        # Robot control ROS publisher
        self.cmd_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        # Settings for keyboard control (get_key)
        self.settings = termios.tcgetattr(sys.stdin)

        self.move_bindings = {
            'i': (1, 0, 0, 0),
            'o': (1, 0, 0, -1),
            'j': (0, 0, 0, 1),
            'l': (0, 0, 0, -1),
            'u': (1, 0, 0, 1),
            ',': (-1, 0, 0, 0),
            '.': (-1, 0, 0, 1),
            'm': (-1, 0, 0, -1),
            'O': (1, -1, 0, 0),
            'I': (1, 0, 0, 0),
            'J': (0, 1, 0, 0),
            'L': (0, -1, 0, 0),
            'U': (1, 1, 0, 0),
            '<': (-1, 0, 0, 0),
            '>': (-1, -1, 0, 0),
            'M': (-1, 1, 0, 0),
            't': (0, 0, 1, 0),
            'b': (0, 0, -1, 0)
        }

        self.speed_bindings = {
            'q': (1.1, 1.1),
            'z': (.9, .9),
            'w': (1.1, 1),
            'x': (.9, 1),
            'e': (1, 1.1),
            'c': (1, .9)
        }

        self.pickup_bindings = {
            'p': (1)
        }

        # Wait for environment to setup
        time.sleep(2)

        self._reset_vals()

    def get_key(self):
        """Get pressed key from keyboard using sys package."""

        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

        return key

    def keyboard_loop(self):
        """Start keyboard loop."""
        try:
            while True:

                key = self.get_key()
                rospy.logdebug('Pressed key: %s', key)

                if key == '\x03':
                    break
                elif key == 'p':
                    self.try_to_pick_up_apple()
                elif key == 'r':
                    self.env_reset()
                else:
                    self.env_step(key)

        except Exception as err: # pylint: disable=W0703,I0011
            rospy.logerr(err)

    def turtlebot_coords(self):
        """Get current turtlebot coordinates"""

        model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        block_name = str(self.turtlebot.name)
        turtlebot_coordinates = model_coordinates(block_name, self.turtlebot.relative_entity_name)

        return turtlebot_coordinates

    def pose_distance(self, pos_0, pos_1):
        """Calculate distance from pose parameters."""

        x_diff = pos_0.pose.position.x - pos_1.pose.position.x
        y_diff = pos_0.pose.position.y - pos_1.pose.position.y
        z_diff = pos_0.pose.position.z - pos_1.pose.position.z

        distance = math.sqrt(
            math.pow(x_diff, 2) +
            math.pow(y_diff, 2) +
            math.pow(z_diff, 2)
        )

        return distance

    def closest_apple(self):
        """Calculate distance to closes apple."""

        distance_list = []
        distance_min = 10

        model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        turtlebot_coordinates = self.turtlebot_coords()

        for i in range(0, 9):

            apple = Block('cricket_ball_' + str(i), 'link')
            block_name = str(apple.name)
            apple_coordinates = model_coordinates(block_name, apple.relative_entity_name)

            distance = self.pose_distance(turtlebot_coordinates, apple_coordinates)
            distance_list.append(distance)

        distance_min = min(distance_list)
        number_min = distance_list.index(min(distance_list))

        return distance_min, number_min

    def try_to_pick_up_apple(self):
        """Try to pickup apple."""

        is_apple_picked = False # idicate wether picked apple or not
        try:
            distance_min, number_min = self.closest_apple()
            rospy.logdebug("Trying to pick up apple - distance %.2f, "
                           "minimum to succeed %.2f", distance_min, self.apple_distance)

            if distance_min <= self.apple_distance and self.apple_idx_list:
                # Instead of deleting apples from world we change it's position
                # by setting them far away from environment
                new_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

                model_state = ModelState()
                apple_name = 'cricket_ball_' + str(number_min)
                rospy.loginfo('Picked %s', apple_name)
                model_state.model_name = apple_name

                twist = Twist()
                twist = self._set_twist(twist)
                model_state.twist = twist

                pose = Pose()
                pose.position.x = 99
                pose.position.y = -99
                pose.position.z = 0.0
                pose.orientation.x = 0.0
                pose.orientation.y = 0.0
                pose.orientation.z = 0.0
                pose.orientation.w = 0.0

                model_state.pose = pose
                model_state.reference_frame = 'world'

                new_model_state(model_state)

                # We need to keep up a list of pick-up apples because we dont actually delete
                # instead change position but we dont want to try pick them up again
                self.apple_idx_list.remove(number_min)
                is_apple_picked = True
            else:
                rospy.logdebug('Come closer to the apple. Min distance is %.2f',
                               self.apple_distance)
        except rospy.ServiceException as err:
            rospy.logerr("Get Model State service call failed: %s", err)

        return is_apple_picked

    def random_apples(self):
        """Put apples in random places."""

        new_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        i = 0

        model_state = ModelState()

        for i in range(0, 9):
            apple_name = 'cricket_ball_' + str(i)
            rospy.loginfo('Random %s', apple_name)
            model_state.model_name = apple_name

            twist = Twist()
            twist = self._set_twist(twist)
            model_state.twist = twist

            pose = Pose()
            pose.position.x = random.uniform(-1.8, 1.8)
            pose.position.y = random.uniform(-1.8, 1.8)
            pose.position.z = 0.0
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = 0.0
            pose.orientation.w = 0.0

            model_state.pose = pose
            model_state.reference_frame = 'world'

            new_model_state(model_state)

    def env_reset(self):
        """Restart world."""

        twist = Twist()
        self.cmd_publisher.publish(twist)

        # TODO: Perhaps will improve spawning issue when some command didn't complete.
        rospy.sleep(1)

        rospy.wait_for_service('/gazebo/reset_world')
        reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        reset_world()

        self.random_apples()

        rospy.loginfo('World reset')

        self._reset_vals()

    def env_step(self, action):
        """Make a step in world"""
        rospy.logdebug("Envstep runnning: '%s'", action)

        l_x = 0
        theta = 0
        step_result = {}
        step_result['tried_pickup'] = False
        step_result['done_pickup'] = False

        # Used when we don't want the RL algo to control robot but we control
        # with keyboard instead. We'd like to see training process anyway.
        if action is None:
            return step_result

        distance_before, _ = self.closest_apple()
        coords_before = self.turtlebot_coords()

        if action in self.move_bindings.keys():

            l_x = self.move_bindings[action][0] # analog to speed
            theta = self.move_bindings[action][3]

            twist = Twist()
            twist = self._set_twist(twist, l_x=l_x, a_z=theta)
            self.cmd_publisher.publish(twist)
            rospy.sleep(0.2)

            twist = self._set_twist(twist)
            self.cmd_publisher.publish(twist)

            # x_distance = 0
            # angular_distance = 0
            # l_x_abs = abs(l_x)
            # theta_abs = abs(theta)

            # if l_x_abs > 0 or theta_abs > 0:
            #     # Setting the current time for distance calculus
            #     start_time = rospy.Time.now().to_sec()

            #     publish_rate = rospy.Rate(30)
            #     # Loop to move the turtle in an specified distance
            #     while x_distance < self.x_distance and angular_distance < self.a_distance:
            #         # Publish the velocity
            #         self.cmd_publisher.publish(twist)
            #         # Takes actual time to velocity calculus
            #         end_time = rospy.Time.now().to_sec()

            #         time_delta = end_time - start_time

            #         # Calculates distancePoseStamped
            #         x_distance = l_x_abs * time_delta
            #         angular_distance = theta_abs * time_delta

            #         publish_rate.sleep()

            # After the loop reset twist values
            # twist = self._set_twist(twist)
            # self.cmd_publisher.publish(twist)

            # TODO: Something is wrong with Gazebo VE, currently dont see another solution.
            # So we just wait until cmd_publisher has published!
            # time.sleep(0.075)

        elif action == 'p':
            is_apple_picked = self.try_to_pick_up_apple()

            step_result['tried_pickup'] = True
            step_result['done_pickup'] = is_apple_picked

        distance_after, _ = self.closest_apple()
        coords_after = self.turtlebot_coords()

        step_result['dist_towrds_apple'] = distance_before - distance_after
        step_result['dist_traveled'] = self.pose_distance(coords_before, coords_after)
        step_result['coords_after'] = coords_after

        dist_from_o_after = self.pose_distance(self.origin_coords, coords_after)

        if dist_from_o_after - self.best_from_o > 0:
            dist_from_o_after_set = dist_from_o_after - self.best_from_o
            self.best_from_o = dist_from_o_after # pylint: disable=W0201
        else:
            dist_from_o_after_set = 0

        step_result['dist_traveled_from_o'] = dist_from_o_after_set

        return step_result

    def help(self):
        """Prints information about controlling robot."""
        msg = """
        Reading from the keyboard  and Publishing to Twist!
        ---------------------------
        Moving around:
        u    i    o
        j    k    l
        m    ,    .

        For Holonomic mode (strafing), hold down the shift key:
        ---------------------------
        U    I    O
        J    K    L
        M    <    >

        t : up (+z)
        b : down (-z)

        anything else : stop

        p: pick up apple

        q/z : increase/decrease max speeds by 10%
        w/x : increase/decrease only linear speed by 10%
        e/c : increase/decrease only angular speed by 10%

        CTRL-C to quit
        """
        rospy.loginfo(msg)

    def _set_twist(self, twist, l_x=0, l_y=0, l_z=0, a_x=0, a_y=0, a_z=0):
        """Reset twist values."""

        twist.linear.x = l_x
        twist.linear.y = l_y
        twist.linear.z = l_z
        twist.angular.x = a_x
        twist.angular.y = a_y
        twist.angular.z = a_z

        return twist

    def _reset_vals(self):
        """Method which reset values necesarry to do so after for each environment reset."""

        # All apple indexes in envornment.
        self.apple_idx_list = range(0, 9)
        # So we know where robot spaw and measure distance it traveled from there
        self.origin_coords = self.turtlebot_coords()
        self.best_from_o = 0

if __name__ == '__main__':
    rospy.loginfo('Starting robot control node')
    rospy.init_node('robot_control')

    ROBO_CONTROL = RobotControl()
    ROBO_CONTROL.env_reset()
    ROBO_CONTROL.keyboard_loop()
