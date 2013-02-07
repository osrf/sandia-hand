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
from sandia_hand_msgs.srv import SimpleGraspSrv
from sandia_hand_msgs.msg import SimpleGrasp

if __name__ == '__main__':
  if len(sys.argv) != 4:
    print "usage: simple_grasp.py SIDE GRASP AMOUNT"
    print "  example:  simple_grasp.py left cylindrical 0.5"
    sys.exit(1)
  side = sys.argv[1]
  grasp = sys.argv[2]
  amount = float(sys.argv[3])
  if side == "left":
    side = "l_hand"
  elif side == "right":
    side = "r_hand"
  else:
    print "side must be 'left' or 'right'"
    sys.exit(1)
  srv_name = "/sandia_hand/%s/simple_grasp" % side 
  rospy.wait_for_service(srv_name)
  try:
    sgs = rospy.ServiceProxy(srv_name, SimpleGraspSrv)
    sgs(SimpleGrasp(grasp, amount))
  except rospy.ServiceException, e:
    print "service call failed: %s" % e
