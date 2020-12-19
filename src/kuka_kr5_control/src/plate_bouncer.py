#!/usr/bin/env python

import rospy
import sys

from std_msgs.msg import String
from geometry_msgs.msg import Point

def bouncer():
  pub = rospy.Publisher('kuka/control13_command', Point, queue_size=10)
  r = rospy.Rate(10) # 10hz

  while not rospy.is_shutdown():
    pt_msg = Point()
    
    #Command plate to move down for 0.1 seconds
    pt_msg.x = 0
    pt_msg.y = 0.1
    pt_msg.z = -0.1
    pub.publish(pt_msg)
    rospy.sleep(0.1)

    #Command plate to move up (to default zero position) for 0.2 seconds
    pt_msg.x = 0
    pt_msg.y = 0
    pt_msg.z = 0
    pub.publish(pt_msg)
    rospy.sleep(0.2)
    
    r.sleep()

if __name__ == '__main__':
  rospy.init_node('bouncer', anonymous=True)
  
  try:
    bouncer()
  except rospy.ROSInterruptException: pass
