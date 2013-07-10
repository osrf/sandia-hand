#!/usr/bin/env python
#
# Software License Agreement (Apache License)
#
# Copyright 2013 Open Source Robotics Foundation
# Author: Morgan Quigley
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import roslib; roslib.load_manifest('sandia_hand_teleop')
import rospy, sys
from sandia_hand_msgs.msg import SimpleGrasp

g_grasp_pub = None
key_map = { '1': [ 'cylindrical', 0.00],
            '2': [ 'cylindrical', 0.25] }

def print_usage():
  print "Each row corresponds to a canonical grasp:"
  print "  top row of letters    = cylindrical (power)"
  print "  second row of letters = spherical"
  print "  third row of letters  = prismatic"
  print ""
  print "Each column corresponds to an amount to open/close the grasp:"
  print "  leftmost column = fully open"
  print "  second column   = 25%  closed"
  print "  third column    = 50%  closed"
  print "  fourth column   = 75%  closed"
  print "  fifth column    = 100% closed"
  print ""
  print "The left side of the keyboard controls the left (or only) hand."
  print "Right side of the keyboard, starting at 'Y', controls the right hand."
  print ""
  print "Example 1: a fully open cylindrical grasp for the left/only hand "
  print "is commnanded by pressing 'q'. For the right hand, press 'y'."
  print ""
  print "Example 2: fully closed spherical grasp for the left/only hand "
  print "is commanded by pressing 'g'. For the right hand, press ';'."

def keypress(c):
  if c in 
  


if __name__ == '__main__':
  if len(rospy.myargv()) != 2:
    print "usage: simple_grasp_keyboard_teleop.py SIDE"
    print "  where side = {left, right, only}"
    sys.exit(1)
  side = rospy.myargv()[1]
  if side == "left":
    side = "left_hand"
  elif side == "right":
    side = "right_hand"
  elif side == "only":
    pass
  else:
    print "side must be in {left, right, only}"
    sys.exit(1)
  if (side == 'only'):
    topic_name = "simple_grasp"
  else:
    topic_name = "%s/simple_grasp" % side 
  rospy.init_node("simple_grasp_keyboard_teleop_%s" % side)
  g_grasp_pub = rospy.Publisher(topic_name, SimpleGrasp)
  rate = rospy.Rate(1.0) # todo: something better
  while not rospy.is_shutdown():
    rate.sleep()
