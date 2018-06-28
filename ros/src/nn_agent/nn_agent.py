#!/usr/bin/env python

#version 2.1 - removing cumulative score
#version 2.2  - increase spot area from 5deg to 10deg, sight from 20 to 30deg
#version 3.0  - speed control (in constant time) in neural network 
#version 3.1  - backward movement, fix in turning speed. Increased step size from 4 frames to 10 frames (@30fps) - was not enought time to accelerate for higher speed.
import roslib; roslib.load_manifest('nn_agent')
import rospy
import torch
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose

from std_srvs.srv import Empty
from std_msgs.msg import UInt8
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.srv import DeleteModel
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState



from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import sys, select, termios, tty, math, time

from guntis4 import run_episodic_learning

#robot angle detection
import tensorflow as tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
import random

#bumper 
#from irobotcreate2.msg import Bumper
#gazebo_msgs/ContactsState , rostopic type /irobot_bumper
from gazebo_msgs.msg import ContactsState


# -------------------
# temporary code from teleop_twist_keyboard

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

#moveBindings = {
#        'i':(0.5,0,0,0),
#        'j':(0,0,0,1),
#        'l':(0,0,0,-1),
#        ',':(-0.5,0,0,0)
#          }
 

#http://wiki.ros.org/turtlesim/Tutorials/Rotating%20Left%20and%20Right 
#  angular velocity.
PI = 3.1415926535897 
#  {x , y, z, theta (angular DEG/sec speed) , duration}

#http://wiki.ros.org/turtlesim/Tutorials/Rotating%20Left%20and%20Right
#   speed = input("Input your speed (degrees/sec):")
#  15     angle = input("Type your distance (degrees):")
#  16     clockwise = input("Clockwise?: ") #True or false
#  18     #Converting from angles to radians
#  19     angular_speed = speed*2*PI/360 = SPEED* 0,017453293 ~ 0.0175
#  20     relative_angle = angle*2*PI/360
# Checking if our movement is CW or CCW
#  30     if clockwise:
#  31         vel_msg.angular.z = -abs(angular_speed)
#  32     else:
#  33         vel_msg.angular.z = abs(angular_speed)
# PI = 3.1415926535897

moveBindings = {
#j
        'L1':(0,0,0,10),
        'L2':(0,0,0,20),
        'L3':(0,0,0,30),
        'L4':(0,0,0,40),
        'L5':(0,0,0,50),
        'L6':(0,0,0,60),        
        'L7':(0,0,0,70),        
        'L8':(0,0,0,80),        
        'L9':(0,0,0,90),        
#l
        'R1':(0,0,0,-10),
        'R2':(0,0,0,-20),
        'R3':(0,0,0,-30),
        'R4':(0,0,0,-40),
        'R5':(0,0,0,-50),
        'R6':(0,0,0,-60),        
        'R7':(0,0,0,-70),        
        'R8':(0,0,0,-80),        
        'R9':(0,0,0,-90),        
        
#i
        'F1':(0.20,0,0,0),
        'F2':(0.40,0,0,0),
        'F3':(0.60,0,0,0),
        'F4':(0.80,0,0,0),
        'F5':(1.00,0,0,0),
        'F6':(1.20,0,0,0),        
        'F7':(1.40,0,0,0),        
        'F8':(1.60,0,0,0),        
        'F9':(1.80,0,0,0),
        
        'B' :(-0.50,0,0,0)        
       }

 

class Block:
    def __init__(self, name, relative_entity_name):
        self._name = name
        self._relative_entity_name = relative_entity_name

class NNAgent(object):
    def __init__(self):
        rospy.init_node('nn_agent')
        rospy.loginfo('Init nn_agent')
        print('Hello world from nn_agent!')

        self.steps_to_stop = 0
        self.observe = True
        self.cmd_publisher = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
        self.bot_publisher = rospy.Publisher('bot', UInt8, queue_size = 1)
				#gunars commented out   
        #self.turtlebot = Block('turtlebot3_burger','')
        self.turtlebot = Block('iRobot','')
        self.bridge = CvBridge() # Convert ROS msg to numpy array
        self.image_array = None
        self.new_image = False
        # rospy.Subscriber('/camera1/image_raw', Image, callback=self.image_callback)
        rospy.Subscriber('/iRobot/camera/image_raw', Image, callback=self.image_callback)
        # rospy.Subscriber('camera1', Image, callback=self.image_callback)
        # print('spinning')
        # rospy.spin()
        # print('spinned')
        self.spotreward = 0.0
        # if in spot and moved closer, gets microreward
        self.gotcloserreward = 0.0
        self.prevdistance = 0.0
        # ANTI-SHAKING penalty , i j i | j i j   ANTIpatern 
        self.prev1action=' ' #previous 1 move
        self.prev2action=' ' #previous 2 move
        self.appleinspot = -1 #number of apple in pickup spot (distance 0.6..0.40 and head angle diff not more that 5deg ) 
        self.target=0 #start with 1th apple, other are deleted
        self.done=False #flag for indication
       

        #BUMPER HIT DETECTION! 
        rospy.Subscriber('/irobot_bumper', ContactsState, callback=self.bumper_callback)
        self.bumped=0;
        
        #remove unnecesarry 1..9 for now 
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        for d in range (1,9):
            apple_name = 'cricket_ball_'+str(d)
            delete_apple = delete_model(apple_name)

        #remove_movable_boxes
        delete_box = delete_model('unit_box_clone');
        delete_box = delete_model('unit_box_clone_0');
        delete_box = delete_model('unit_box_clone_1');
        delete_box = delete_model('unit_box_clone_2');
        

    def get_observation(self):
        while not self.new_image:
            time.sleep(0.1) 
        self.new_image = False
        # print('Received image data is: %s %s' % (self.image_array.shape, self.image_array.dtype))
        return self.image_array

    def image_callback(self, msg):
        if self.observe:
            self.observe = False
            self.image_array = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.new_image = True

        if self.steps_to_stop:
            self.steps_to_stop -= 1
            if not self.steps_to_stop:
                # print('Stopping')
                twist = Twist()
                twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0;
                twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0;            
                self.cmd_publisher.publish(twist)  
                self.observe = True          

    def bumper_callback(self, msg):
         if not  msg.states:
             self.bumped=0;
             #print('BUMPER MESSAGE: empty - not in touch ')
         else:
             #print('BUMPER MESSAGE: NOT empty - HIT IN OBJECT!!! ')
             #print(msg)
             
             if msg.states[0].collision2_name=='cricket_ball_0::link::collision' or msg.states[0].collision1_name=='cricket_ball_0::link::collision':
                 #print('  __ UPS__ Ignoring bumping in apple ')
                 self.appleinspot = self.target; #if bumping, then enough close (little cheat)
             else:
                 #RESET world triggers ground plane collision, IGNORE IT
                 if msg.states[0].collision2_name=='ground_plane::link::collision' or msg.states[0].collision1_name=='ground_plane::link::collision':
                     #print('  __  Ignoring bumping in GROUND ')
                     self.bumped=0
                 else:
                     print(' =>[ Collision '+ msg.states[0].collision2_name +' into '+ msg.states[0].collision1_name );
                     self.bumped=self.bumped+1;
             

    def envreset(self):
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0;
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0;            
        self.cmd_publisher.publish(twist)                    
        rospy.wait_for_service('/gazebo/reset_world')
        reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        reset_world()
        print('World reset')
        self.observe = True
        self.spotreward = 0.0
        self.gotcloserreward = 0.0
        self.prevdistance = 0.0
        self.prev1action=' ' #previous 1 move
        self.prev2action=' ' #previous 2 move
        self.target = 0
        self.done=False #flag for indication


        #put needed apple0 in spot in 90 deg CCW 
        new_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        model_state = ModelState()
        apple_name = 'cricket_ball_'+str(self.target)
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
        # x=-2 , y=0 (left side)
        # x=-2 , y=-1 (right side)
        # x=-2.5 y=-1 (back quadrant 2 135deg)
        # x=-2.5  y=0 (back quadrant 3, 215deg)
        # -- default, in left corner behind 2 towers but visible at start
        #  x=-0.5  y=+1.5
        
        #static start point with fluctuation
        #pose.position.x = -0.5 + (np.random.random_sample()/2)
        #pose.position.y = 1.5 + (np.random.random_sample()/2)

        #two blocks straight forward
        #pose.position.x = -0.3 + (np.random.random_sample())
        #pose.position.y = -1 + (np.random.random_sample())

        #one block straight forward
        pose.position.x = -1.1 + (np.random.random_sample()/2)
        pose.position.y = -1 + (np.random.random_sample()/2)


        #just in front for apple collision exception
        #pose.position.x = -1.7 
        #pose.position.y = -0.5 


        #random point
        #pose.position.x=np.random.random_sample()*2; #scale of grid -3 to +3 
        #pose.position.y=np.random.random_sample()*2; #scale of grid -3 to +3 
        #print('APPLE put in random location x,y = ( %.2f , %.2f )' % (pose.position.x, pose.position.y))
       
        pose.position.z = 0.0
        pose.orientation.x = 0.0 
        pose.orientation.y = 0.0 
        pose.orientation.z = 0.0 
        pose.orientation.w = 0.0
        model_state.pose = pose
        model_state.reference_frame = 'world'
        new_apple_state = new_model_state(model_state)

        


        return self.get_observation()

    def closest_apple(self):
        model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        blockName = str(self.turtlebot._name)
        turtlebot_coordinates = model_coordinates(blockName, self.turtlebot._relative_entity_name)
        global roll, pitch, yaw

        #added by gunars
        TWOPI = 6.2831853071795865
        RAD2DEG = 57.2957795130823209        
        PI =3.141592654
        #since v2.1
        #INSPOT=5 #+- 10degree pickup spot , microreward
        #INSIGHT=20 #+- 40degree visibility spot , microreward , 70fow was too narrow
        i = self.target; #targeting first apple , 1 _0 not 
        
        #since v2.2
        INSPOT=10 #+- 10degree pickup spot , microreward
        INSIGHT=30 #+- 40degree visibility spot , microreward , 70fow was too narrow

        

        self.appleinspot=-1; #each time detect apple in spot (targeted)
				
        OFFSET=1000; #coordinate system shift for calculation, cannot handle negative 

        distanceList = []
        distanceMin = 10

        orientation_q = turtlebot_coordinates.pose.orientation        
        npQ4 =np.zeros(4)
        npQ4[0]=turtlebot_coordinates.pose.orientation.x
        npQ4[1]=turtlebot_coordinates.pose.orientation.y
        npQ4[2]=turtlebot_coordinates.pose.orientation.z
        npQ4[3]=turtlebot_coordinates.pose.orientation.w
        
        #(roll, pitch, yaw) = euler_from_quaternion (orientation_q)  #requires numpy array
        (roll, pitch, yaw) = euler_from_quaternion (npQ4)  #requires numpy array  
        az=yaw; # 0.0 = faceforward 0deg , left90 =+pi/2 , right90= -pi/2 , backward180 = +/- pi 
        fz=yaw*-180/PI; #invert rotation from CCW to CW -pi to +pi
        if fz < 0.0: #make to 360
            fz=fz+360; 

        #print('YAW %s yaw=%.4f  DEG=%.4f ' % (blockName, yaw, fz))
            

        if  i < 10:
        #for i in range (1,2):
            decreased_distance = 0;
            apple = Block('cricket_ball_'+str(i), 'link')
            blockName = str(apple._name)
            apple_coordinates = model_coordinates(blockName, apple._relative_entity_name)
            distance = math.sqrt(math.pow(turtlebot_coordinates.pose.position.x - apple_coordinates.pose.position.x,2)+math.pow(turtlebot_coordinates.pose.position.y - apple_coordinates.pose.position.y,2)+math.pow(turtlebot_coordinates.pose.position.z - apple_coordinates.pose.position.z,2))

            if self.prevdistance == 0.0:
						    self.prevdistance =distance;
            gotcloser =self.prevdistance-distance;
            if (gotcloser) > 0.05: #filter-off small accedential move reward by turning robot
						    self.prevdistance = distance;
						    decreased_distance = 1; #flag for award moving closer to target (if in radar or spot)

				    #self.gotcloserreward = 0.0
				    #self.prevdistance = 0.0

            #problems with negative numbers , need quadrant transformation
            argx=apple_coordinates.pose.position.x-turtlebot_coordinates.pose.position.x;
            if turtlebot_coordinates.pose.position.x < apple_coordinates.pose.position.x:
              argx=apple_coordinates.pose.position.x - turtlebot_coordinates.pose.position.x;
            #  print('X arg inv')

            argy=turtlebot_coordinates.pose.position.y -  apple_coordinates.pose.position.y;
            if turtlebot_coordinates.pose.position.y > apple_coordinates.pose.position.y:
               argy=turtlebot_coordinates.pose.position.y -  apple_coordinates.pose.position.y;
            #   print('Y arg inv')
            
            theta = math.atan2(argy,argx);

            #if argx<0.0 and argy<0.0:
            #  print('Quadrant 2 compensation +90deg');
            #  theta=theta+PI/2; 
            
            #shortest way detection fix 270 aple , bot 0 =CCW 
            cw_dist = (theta*180/PI) - (yaw*-180/PI);  
            #print('APPLE x,y = ( %.2f , %.2f ) BOT x,y = ( %.2f , %.2f ) , dist %.2f , ARGx=%.4f, ARGy=%.4f , CWDist raw= %.0f ' % (apple_coordinates.pose.position.x, apple_coordinates.pose.position.y,turtlebot_coordinates.pose.position.x, turtlebot_coordinates.pose.position.y,distance,argx, argy, cw_dist  ))


            diff=cw_dist;

            if diff<-180.0:  #from -270 to -180 
                diff=360+diff;

            if diff>180.0:  #from +180 +270 to  
                diff=360-diff;
            
            #invert
            if diff<0.0:
                diff=-diff;            
           
            #--DEBUGING angle 
            #print(' APPLE0 theta=%.4f  bot_yaw=%.4f  dist CW =%.4f diff=%.4f   dist=%.2f' % (theta,yaw, cw_dist, diff, distance))

            #print('THETA %s BOTZa=%.4f z=%.4f , Apple_angle(p)=%.4f DIFF=%.4f || theta= %.4f  theta2a=%.4f  thetaXY %.4f ' % (blockName, fz,az, p, diff, theta, theta2a, thetaXYa))
            #print('APPLE x,y = ( %.2f , %.2f ) BOT x,y = ( %.2f , %.2f ) , dist %.2f ' % (apple_coordinates.pose.position.x, apple_coordinates.pose.position.y, turtlebot_coordinates.pose.position.x, turtlebot_coordinates.pose.position.y,distance ))
            
            #try:
            #   input("STEP CALC. enter to continue")
            #except SyntaxError:
            #   pass


            if diff <= INSIGHT:
                print('  #### %s in radar SIGHT (%.0f) diff= %.2f ' % (blockName,INSIGHT,diff))
                self.spotreward+=0.10;
                #try:
                #   input("Press enter to continue")
                #except SyntaxError:
                #   pass

                if decreased_distance == 1 :
                    self.gotcloserreward+=2.20;
                    print(' -->> <<-- %s GOT MOVE sight award+ %.2f , got closer: %.4f ' % (blockName,self.gotcloserreward, gotcloser))
                    #verify that locking really works
                    #sys.exit()
                    #try:
                    #   input("Press enter to continue")
                    #except SyntaxError:
                    #   pass


            if diff <= INSPOT:
                print('  -==(*)==-  %s radar SPOT (%.0f) diff= %.2f ' % (blockName,INSPOT,diff))
                self.spotreward+=0.45;
                self.appleinspot=i;
                #verify that locking really works
                #sys.exit()
                #try:
                #   input("Press enter to continue")
                #except SyntaxError:
                #   pass
                
                
 
                if decreased_distance == 1:
                    self.gotcloserreward+=5.48;
                    print('  >>> * <<< %s GOT MOVE SPOT  sight award+ %.2f , got closer: %.4f ' % (blockName,self.gotcloserreward, gotcloser))
                    #try:
                    #   input("Press enter to continue")
                    #except SyntaxError:
                    #   pass

            distanceList.append(distance)
        distanceMin = min(distanceList)
        numberMin = distanceList.index(min(distanceList))
        return distanceMin, 1 # numberMin

    def try_to_pick_up_apple(self):
        reward = 0
        try:
            distanceLim = 0.80 #We should define the minimum distance to apple, where robot can pick up , org=.50 , easening to 0.80 
            distanceMin, numberMin = self.closest_apple()
            #print('Trying to pick up apple - distance %.2f, minimum to succeed %.2f APPLE IN SPOT=%.0f' % (distanceMin, distanceLim, self.appleinspot))
            if (self.appleinspot!=self.target):
                #print('PICKUP FAIL. - Target %.0f not in spot or wrong apple (SPOT=%.0f)' % (self.target, self.appleinspot))
                #self.spotreward=self.spotreward-1.5; #clean reward -1.5 makes affraid to pickup
                self.spotreward=self.spotreward-0.10; #remove minimal in sight award
                
            else:
              if distanceMin <= distanceLim : 
                  #delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
                  new_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
                  model_state = ModelState()
                  apple_name = 'cricket_ball_0'#+str(numberMin)
                  print('  +==========================================+ ' )
                  print('  |               PICKED  APPLE              | ' )
                  print('  |                   + 1000                 | ' )
                  print('  +==========================================+ ' )                  
                  self.done=True
                  
                  #try:
                  #  input("Press enter to continue")
                  #except SyntaxError:
                  #  pass

                  
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
                  
                  #random location (not default x=0.2 y=-2.4)
                  #pose.position.x = 0.2
                  #pose.position.y = -2.4 
                  pose.position.x=np.random.random_sample()*2; #scale of grid -3 to +3 
                  pose.position.y=np.random.random_sample()*2; #scale of grid -3 to +3
                  print('  >>>>  APPLE MOVED in random location x,y = ( %.2f , %.2f )' % (pose.position.x, pose.position.y))
                  pose.position.z = 1.0
                  pose.orientation.x = 0.0 
                  pose.orientation.y = 0.0 
                  pose.orientation.z = 0.0 
                  pose.orientation.w = 0.0
                  model_state.pose = pose
                  model_state.reference_frame = 'world'
                  #delete_apple = delete_model(apple_name)
                  new_apple_state = new_model_state(model_state)
                  reward = 1000
                  #self.target=self.target+1; #next apple 

                  
                  
              else:
                  # reward = -1
                  #print ('You should come closer to the apple. Minimal distance = 0.50')
                  self.gotcloserreward=self.gotcloserreward+2/distanceMin; #attempt to pickup in spot , the closer the better score
                  print(' _^^^^_ PICKUP (not in range). - Distance %.2f , got spot pickup bonus +%.2f ' % (distanceMin, 2/distanceMin))
                  
        except rospy.ServiceException as e:
            rospy.loginfo("Get Model State service call failed:  {0}".format(e))
        return reward

    def keyboard_loop(self):
        try:
            while(1):
                x = 0
                th = 0
                key = getKey()
                
                
                if (key == '\x03'):
                    break
                elif key == 'p':
                    self.try_to_pick_up_apple()
                elif key == 'r':
                    self.envreset()
                elif key in moveBindings.keys():
                    x = moveBindings[key][0]
                    th = moveBindings[key][3]
                bot_action = botBindings[key]
                print(key, bot_action)
                twist = Twist()
                twist.linear.x = x;
                twist.linear.y = 0;
                twist.linear.z = 0;
                twist.angular.x = 0;
                twist.angular.y = 0;
                twist.angular.z = th*0.0175; #DEGREE TO RADIANS
                print(twist)
                #self.steps_to_stop = 4

                self.steps_to_stop = 10
                self.cmd_publisher.publish(twist)
                self.bot_publisher.publish(UInt8(bot_action))

        except Exception as e:
            print(e)

    def envstep(self, action):
    
        #temporary , rotatle left (j) for radar math tuning --gunars
        #action='j'
        bot_action=action[0]
        bot_speed=' '

    
        #distance_before, _ = self.closest_apple()
        #bot_action = ord(action)
        #NEW mapper FOR ACTIONS
        if len(action)==2:
          bot_action=action[0]
          bot_speed = action[1]          
        
        
        #print("Envstep runnning: '%s' / [%s] speed [%s]" % (action, bot_action,bot_speed ))
        #checking for SHAKING anti-patern  j l j or l j l 


        #if  action == 'j' and self.prev1action=='l' and self.prev2action=='j' :
        #    self.spotreward-=0.16;	
        #    print(" __SLAP__  jlj shake penalty -0.16 ! spotrew = %.2f " % self.spotreward)
                
        #if action=='l' and self.prev1action=='j' and self.prev2action=='l':
        #    self.spotreward-=0.16;	
        #    print(" __SLAP__  ljl shake penalty -0.16 ! spotrew = %.2f " % self.spotreward)


        if action=='p' and self.prev1action=='p' :
            self.spotreward-=0.50;	
            print(" __SLAP PP __   penalty -0.50 ! spotrew = %.2f " % self.spotreward)


        self.prev2action=self.prev1action
        self.prev1action=action;

        x = 0
        th = 0
        reward = 0
        if action in moveBindings.keys():
            x = moveBindings[action][0]
            th = moveBindings[action][3]
            
            #print("MOVEMENT : '%s' / x = [%f] angle th =[%f]" % (action, x,th ))
        elif action == 'p':
            reward = self.try_to_pick_up_apple()

        twist = Twist()
        twist.linear.x = x;
        twist.linear.y = 0;
        twist.linear.z = 0;
        twist.angular.x = 0;
        twist.angular.y = 0;
        twist.angular.z = th*0.0175; #DEGREE TO RADIANS
        #self.steps_to_stop = 4
        self.steps_to_stop = 10 #@30FPS = 1/3 SEC
        self.cmd_publisher.publish(twist)
        if bot_speed!=' ':
          self.bot_publisher.publish(UInt8(ord(bot_speed))) 
        
        self.bot_publisher.publish(UInt8(ord(bot_action)))
        observation = self.get_observation()        
        distance_after, _ = self.closest_apple() 
        
        # reward = reward + distance_before - distance_after
        summ = reward+self.spotreward+self.gotcloserreward
        

        
        #handle BUMP activity , limit 3 bumps
        # -200 was too mach, if makes affraid to move forward
        if self.bumped>3:
          print(" __ [ BUMP ] __  penalty -20 ! ")
          summ=-20;
          self.envreset()
          self.bumped=0
          self.done=True
          
        
        
        print('Key: [%s] . Apple1 away %.2f. Base reward %.2f + spot %.2f + attack %.2f = %.2f' % (action, distance_after, reward, self.spotreward, self.gotcloserreward, summ))

        #since v2.1 -remove accumulating score during play
        self.spotreward = 0.0
        self.gotcloserreward = 0.0
        return (observation, summ, self.done, 0)

if __name__ == '__main__':
    try:
        settings = termios.tcgetattr(sys.stdin)
        agent = NNAgent()
        # agent.keyboard_loop()
        run_episodic_learning(agent.envreset, agent.envstep)
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start nn_agent node.')
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
