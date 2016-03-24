#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def talker():
  pub = rospy.Publisher('chatter', String, queue_size=10)
  rospy.init_node('talker', anonymous=True)
  rate = rospy.Rate(10) # 10 Hz
  # Check that node not shutdown (Ctrl-C etc.)
  while not rospy.is_shutdown():
    # Init string
    hello_str = "hello world %s" % rospy.get_time()
    # Prints message to screen, writes message to the node's 
    # log file, and writes message to rosout
    rospy.loginfo(hello_str)
    # Publishes string to chatter topic
    pub.publish(hello_str) 
    # Sleeps just long enough to maintain desired rate
    rate.sleep()

# Checks rospy.ROSInterruptException exception which may be thrown by
# rospy.sleep() and rospy.Rate.sleep() when Ctrl-C is pressed or when
# the Node is otherwise shutdown
if __name__ == '__main__':
  try:
    talker()
  except rospy.ROSInterruptException:
    pass


