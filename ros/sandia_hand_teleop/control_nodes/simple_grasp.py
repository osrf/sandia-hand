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
import rospy
from sandia_hand_msgs.srv import SimpleGraspSrv, SimpleGraspSrvResponse
from sandia_hand_msgs.msg import SimpleGrasp
from osrf_msgs.msg import JointCommands

g_jc_pub = None
g_jc = JointCommands()

def grasp_srv(req):
  return grasp_cb(req.grasp)

def grasp_cb(msg):
  global g_jc_pub, g_jc
  print "request: grasp [%s] amount [%f]" % (msg.name, msg.closed_amount)
  # save some typing
  gn = msg.name
  x = msg.closed_amount
  if x < 0:
    x = 0
  elif x > 1:
    x = 1
  origin = [0] * 12
  g0 = [0] * 12
  if (gn == "cylindrical"):
    g0 = [0,1.5,1.7, 0,1.5,1.7, 0,1.5,1.7, -0.2,.8,1.2]
  elif (gn == "spherical"):
    origin = [-0.7,0,0, 0.1,0,0, 0.7,0,0, 0,0,0]
    g0 = [0,1.4,1.4, 0,1.4,1.4, 0,1.4,1.4, 0,0.7,0.7]
  elif (gn == "prismatic"):
    origin = [0,1.4,0, 0,1.4,0, 0,1.4,0, -0.1,0.8,-0.8]
    g0 = [0,0,1.4, 0,0,1.4, 0,0,1.4, 0,0,1.4]
  else:
    return None # bogus
  g_jc.position = [0] * 12
  for i in xrange(0, 12):
    g_jc.position[i] = origin[i] + g0[i] * x
  print "joint state: %s" % (str(g_jc.position))
  g_jc_pub.publish(g_jc)

  return SimpleGraspSrvResponse()

if __name__ == '__main__':
  rospy.init_node('simple_grasp')
  g_jc.name = ["f0_j0", "f0_j1", "f0_j2",
               "f1_j0", "f1_j1", "f1_j2",
               "f2_j0", "f2_j1", "f2_j2",
               "f3_j0", "f3_j1", "f3_j2"]
  g_jc.position = [0] * 12
  g_jc_pub = rospy.Publisher('joint_commands', JointCommands) # same namespace
  g_jc_srv = rospy.Service('simple_grasp', SimpleGraspSrv, grasp_srv)
  g_jc_sub = rospy.Subscriber('simple_grasp', SimpleGrasp, grasp_cb)
  rospy.spin()
