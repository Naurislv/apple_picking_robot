#! /usr/bin/env python

# Standard imports
import math
import sys
import select
import termios
import tty

# Dependency imports
import roslib
import rospy

from std_srvs.srv import Empty
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.srv import DeleteModel
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose

roslib.load_manifest('robot_control')

class Block:
    """Define block object."""

    def __init__(self, name, relative_entity_name):
        self.name = name
        self.relative_entity_name = relative_entity_name

class RobotControl(object):
    """Control robot in gazebo environment."""

    def __init__(self):
        self.help() # Print Robot Control infomration
        self.turtlebot = Block('turtlebot3_burger', '')

        # Robot control ROS publisher
        self.cmd_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        # Minimum distance to apple, where robot can pick up
        self.distance_lim = 0.35
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
                    self.env_action(key)

        except Exception as err: #pylint: disable=W0703
            rospy.logerr(err)

    def closest_apple(self):
        """Calculate distance to closes apple."""

        model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        block_name = str(self.turtlebot.name)
        turtlebot_coordinates = model_coordinates(block_name, self.turtlebot.relative_entity_name)

        distance_list = []
        distance_min = 10

        for i in range(0, 9):
            apple = Block('cricket_ball_' + str(i), 'link')
            block_name = str(apple.name)
            apple_coordinates = model_coordinates(block_name, apple.relative_entity_name)

            x_diff = turtlebot_coordinates.pose.position.x - apple_coordinates.pose.position.x
            y_diff = turtlebot_coordinates.pose.position.y - apple_coordinates.pose.position.y
            z_diff = turtlebot_coordinates.pose.position.z - apple_coordinates.pose.position.z
            distance = math.sqrt(
                math.pow(x_diff, 2) +
                math.pow(y_diff, 2) +
                math.pow(z_diff, 2)
            )

            distance_list.append(distance)

        distance_min = min(distance_list)
        number_min = distance_list.index(min(distance_list))

        return distance_min, number_min

    def try_to_pick_up_apple(self):
        """Try to pickup apple."""

        is_apple_picked = False # idicate wether picked apple or not
        try:
            distance_min, number_min = self.closest_apple()
            rospy.loginfo("Trying to pick up apple - distance %.2f, "
                          "minimum to succeed %.2f", distance_min, self.distance_lim)

            if distance_min <= self.distance_lim:
                # delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
                rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

                model_state = ModelState()
                apple_name = 'cricket_ball_' + str(number_min)
                rospy.loginfo('Picked %s', apple_name)
                model_state.model_name = apple_name

                twist = Twist()
                twist = self._reset_twist(twist)
                model_state.twist = twist

                pose = Pose()
                pose.position.x = 0.2
                pose.position.y = -2.4
                pose.position.z = 0.0
                pose.orientation.x = 0.0
                pose.orientation.y = 0.0
                pose.orientation.z = 0.0
                pose.orientation.w = 0.0

                model_state.pose = pose
                model_state.reference_frame = 'world'

                # delete_apple = delete_model(apple_name)
                # new_apple_state = new_model_state(model_state)

                is_apple_picked = True
            else:
                rospy.loginfo('Come closer to the apple. Min distance is %.2f', self.distance_lim)
        except rospy.ServiceException as err:
            rospy.loginfo("Get Model State service call failed:  {0}".format(err))

        return is_apple_picked

    def env_reset(self):
        """Restart world."""
        twist = Twist()
        self.cmd_publisher.publish(twist)

        rospy.wait_for_service('/gazebo/reset_world')
        reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        reset_world()
        rospy.loginfo('World reset')

    def env_step(self, action):
        """Make a step in world"""
        rospy.loginfo("Envstep runnning: '%s'", action)

        l_x = 0
        theta = 0
        step_result = {}
        step_result['tried_pickup'] = False
        step_result['done_pickup'] = False

        distance_before, _ = self.closest_apple()
        step_result['distance_before'] = distance_before

        if action in self.move_bindings.keys():
            l_x = self.move_bindings[action][0]
            theta = self.move_bindings[action][3]
        elif action == 'p':
            is_apple_picked = self.try_to_pick_up_apple()

            step_result['tried_pickup'] = True
            step_result['done_pickup'] = is_apple_picked

        twist = Twist()
        twist = twist = self._reset_twist(twist, l_x=l_x, a_z=theta)

        self.cmd_publisher.publish(twist)
        step_result['distance_after'] = self.closest_apple()

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

    def _reset_twist(self, twist, l_x=0, l_y=0, l_z=0, a_x=0, a_y=0, a_z=0):
        """Reset twist values."""

        twist.linear.x = l_x
        twist.linear.y = l_y
        twist.linear.z = l_z
        twist.angular.x = a_x
        twist.angular.y = a_y
        twist.angular.z = a_z

        return twist

if __name__ == '__main__':
    rospy.init_node('robot_control')

    ROBO_CONTROL = RobotControl()
    ROBO_CONTROL.keyboard_loop()
