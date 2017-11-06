#! /usr/bin/env python
import rospy
from std_msgs.msg  import Int32

def callback(data):
    rospy.loginfo(' recieved data is: %d',  data.data)

def listener():
    rospy.init_node('images_listener', anonymous=True)
    rospy.Subscriber('imagedata',Int32, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
