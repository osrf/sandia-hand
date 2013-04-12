#!/usr/bin/env python
import roslib; roslib.load_manifest('sandia_hand_driver')
import rospy, sys
from sandia_hand_msgs.msg import RawFingerStatus, RawMoboStatus, RawPalmStatus

# todo: at some point, consider porting this to c++ for speed,
# or even stuffing it into the driver node. easy to hack on it here though

class HandStateEstimator:
  def __init__(self, argv):
    rospy.init_node('sandia_hand_state_estimator')
    self.finger_subs = []
    self.finger_subs.append(rospy.Subscriber('raw_finger_status_0', 
                                           RawFingerStatus, self.finger_0_cb))
    self.finger_subs.append(rospy.Subscriber('raw_finger_status_1', 
                                           RawFingerStatus, self.finger_1_cb))
    self.finger_subs.append(rospy.Subscriber('raw_finger_status_2', 
                                           RawFingerStatus, self.finger_2_cb))
    self.finger_subs.append(rospy.Subscriber('raw_finger_status_3', 
                                           RawFingerStatus, self.finger_3_cb))
    self.palm_sub = rospy.Subscriber('raw_palm_status', 
                                     RawPalmStatus, self.palm_cb) 
    self.mobo_sub = rospy.Subscriber('raw_mobo_status',
                                     RawMoboStatus, self.mobo_cb)

  # todo: figure out how closures work in python. this is gross. want to 
  # work on something downstream at the moment, though, so I'm leaving it...
  def finger_0_cb(self, msg):
    self.finger_cb(0, msg)
  def finger_1_cb(self, msg):
    self.finger_cb(1, msg)
  def finger_2_cb(self, msg):
    self.finger_cb(2, msg)
  def finger_3_cb(self, msg):
    self.finger_cb(3, msg)

  def finger_cb(self, finger_idx, msg):
    #if finger_idx == 0:
    #  print "finger 0 rx"
    dpa = [float(x)/1023.0 for x in msg.dp_accel]
    ppa = [float(x)/1023.0 for x in msg.pp_accel]
    mma = [float(x)/1023.0 for x in msg.mm_accel]
    #print [dpa, ppa, mma]
    if finger_idx == 0:
      print msg.mm_accel, msg.dp_accel, msg.pp_accel

  def mobo_cb(self, msg):
    pass

  def palm_cb(self, msg):
    pass

############################################################################
if __name__ == '__main__':
  hse = HandStateEstimator(rospy.myargv())
  rospy.spin()
