#!/usr/bin/env python
#The line above tells Linux that this file is a Python script,
#and that the OS should use the Python interpreter in /usr/bin/env
#to run it. Don't forget to use "chmod +x [filename]" to make
#this script executable.

#Import the rospy package. For an import to work, it must be specified
#in both the package manifest AND the Python file in which it is used.
import rospy
import sys

#Import the String message type from the /msg directory of
#the std_msgs package.
from std_msgs.msg import String
from geometry_msgs.msg import Point

#Define the method which contains the main functionality of the node.
def bouncer():

  #Create an instance of the rospy.Publisher object which we can 
  #use to publish messages to a topic. This publisher publishes 
  #messages of type std_msgs/String to the topic /chatter_talk
  pub = rospy.Publisher('kuka/control13_command', Point, queue_size=10)

  # Create a timer object that will sleep long enough to result in
  # a 10Hz publishing rate
  r = rospy.Rate(10) # 10hz

  # Loop until the node is killed with Ctrl-C
  while not rospy.is_shutdown():
    # Construct a string that we want to publish
    # (In Python, the "%" operator functions similarly
    #  to sprintf in C or MATLAB)
    pt_msg = Point()

    #key = raw_input('Press A to bring plate down, S to bring up: ')
    #time = rospy.get_time()

    #if key == 'a':
    pt_msg.x = 0
    pt_msg.y = 0.1
    pt_msg.z = -0.1
    pub.publish(pt_msg)
    rospy.sleep(0.1)

    #elif key == 's':
    pt_msg.x = 0
    pt_msg.y = 0
    pt_msg.z = 0
    pub.publish(pt_msg)
    rospy.sleep(0.2)
    #pub_string = "hello world %s" % (rospy.get_time())
    
    # Publish our string to the 'chatter_talk' topic
    
    # Use our rate object to sleep until it is time to publish again
    r.sleep()
      
# This is Python's sytax for a main() method, which is run by default
# when exectued in the shell
if __name__ == '__main__':
  # Check if the node has received a signal to shut down
  # If not, run the talker method

  #Run this program as a new node in the ROS computation graph 
  #called /talker.
  rospy.init_node('bouncer', anonymous=True)
  
  try:
    bouncer()
  except rospy.ROSInterruptException: pass
