#! /usr/bin/env python
import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy
import math

from geometry_msgs.msg import Twist
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.srv import DeleteModel
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose

import sys, select, termios, tty

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

moveBindings = {
		'i':(1,0,0,0),
		'o':(1,0,0,-1),
		'j':(0,0,0,1),
		'l':(0,0,0,-1),
		'u':(1,0,0,1),
		',':(-1,0,0,0),
		'.':(-1,0,0,1),
		'm':(-1,0,0,-1),
		'O':(1,-1,0,0),
		'I':(1,0,0,0),
		'J':(0,1,0,0),
		'L':(0,-1,0,0),
		'U':(1,1,0,0),
		'<':(-1,0,0,0),
		'>':(-1,-1,0,0),
		'M':(-1,1,0,0),
		't':(0,0,1,0),
		'b':(0,0,-1,0),
	       }

speedBindings={
		'q':(1.1,1.1),
		'z':(.9,.9),
		'w':(1.1,1),
		'x':(.9,1),
		'e':(1,1.1),
		'c':(1,.9),
	      }

pickupBindings={
		'p':(1)
	      }

def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key


def vels(speed,turn):
	return "currently:\tspeed %s\tturn %s " % (speed,turn)

class Block:
    def __init__(self, name, relative_entity_name):
        self._name = name
        self._relative_entity_name = relative_entity_name

class Tutorial:
    i=0
    turtlebot = Block('turtlebot3_burger','')

    def closeest_apple_calculation(self):
        try:
            model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
	    blockName = str(self.turtlebot._name)
	    turtlebot_coordinates = model_coordinates(blockName, self.turtlebot._relative_entity_name)
	    print(blockName)
            print(turtlebot_coordinates.pose.position)
	    distanceList = []
	    distanceMin = 10
	    distanceLim = 0.25 #We should define the minimum distance to apple, where robot can pick up 
	    for i in range (0,9):
		apple = Block('cricket_ball_'+str(i), 'link')
                blockName = str(apple._name)
	    	apple_coordinates = model_coordinates(blockName, apple._relative_entity_name)
		distance = math.sqrt(math.pow(turtlebot_coordinates.pose.position.x - 			       apple_coordinates.pose.position.x,2)+math.pow(turtlebot_coordinates.pose.position.y - apple_coordinates.pose.position.y,2)+math.pow(turtlebot_coordinates.pose.position.z - apple_coordinates.pose.position.z,2))
		print '\n'		
		distanceList.append(distance)
	    print (distanceList)
	    distanceMin = min(distanceList)
	    numberMin = distanceList.index(min(distanceList))
	    print '\n'
	    print ('Minimal distance = '+str(distanceMin)) 
	    print '\n'
	    if distanceMin <= distanceLim: 
		    #delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
		    new_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
		    model_state = ModelState()
		    apple_name = 'cricket_ball_'+str(numberMin)
		    print (apple_name)
		    model_state.model_name = apple_name
		    twist = Twist()
		    twist.linear.x = 0
		    twist.linear.y = 0
		    twist.linear.z = 0
		    twist.angular.x = 0 
		    twist.angular.y = 0 
		    twist.angular.z = 0
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
		    #delete_apple = delete_model(apple_name)
		    new_apple_state = new_model_state(model_state)
	    else:
		    print ('You should come closer to the apple. Minimal distance = 0.25')
        except rospy.ServiceException as e:
            rospy.loginfo("Get Model State service call failed:  {0}".format(e))


if __name__=="__main__":
    	settings = termios.tcgetattr(sys.stdin)
	
	pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
	rospy.init_node('teleop_twist_keyboard')

	speed = rospy.get_param("~speed", 0.5)
	turn = rospy.get_param("~turn", 1.0)
	x = 0
	y = 0
	z = 0
	th = 0
	status = 0
	model_state = ''

	try:
		print msg
		print vels(speed,turn)
		while(1):
			key = getKey()
			if key in moveBindings.keys():
				x = moveBindings[key][0]
				y = moveBindings[key][1]
				z = moveBindings[key][2]
				th = moveBindings[key][3]
			elif key in speedBindings.keys():
				speed = speed * speedBindings[key][0]
				turn = turn * speedBindings[key][1]

				print vels(speed,turn)
				if (status == 14):
					print msg
				status = (status + 1) % 15
			else:
				x = 0
				y = 0
				z = 0
				th = 0
				if key in pickupBindings.keys():
				   	print 'pick up apple'
				   	tuto = Tutorial()
					tuto.closeest_apple_calculation()
				if (key == '\x03'):
					break

			twist = Twist()
			twist.linear.x = x*speed; twist.linear.y = y*speed; twist.linear.z = z*speed;
			twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th*turn
			pub.publish(twist)
			rospy.loginfo("Twist  {}\n".format(twist))

	except:
		print 'Something wrong'

	finally:
		twist = Twist()
		twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
		twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
		pub.publish(twist)

    		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


