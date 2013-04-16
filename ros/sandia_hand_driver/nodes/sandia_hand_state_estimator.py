#!/usr/bin/env python
import roslib; roslib.load_manifest('sandia_hand_driver')
import rospy, sys, math
from sandia_hand_msgs.msg import RawFingerStatus, RawMoboStatus, \
                                 RawPalmStatus, CalFingerStatus

# todo: at some point, consider porting this to c++ for speed,
# or even stuffing it into the driver node. easy to hack on it here though

class HandStateEstimator:
  def __init__(self, argv):
    rospy.init_node('sandia_hand_state_estimator')
    self.finger_subs = []
    self.finger_pubs = []
    self.cfs = []
    for i in xrange(0,4):
      self.finger_pubs.append(rospy.Publisher('cal_finger_status_' + str(i),
                                              CalFingerStatus))
      self.cfs.append(CalFingerStatus())
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
    #print "finger %d rx" % finger_idx
    mma = [float(x)/1023.0 for x in msg.mm_accel]
    ppa = [float(x)/1023.0 for x in msg.pp_accel]
    dpa = [float(x)/1023.0 for x in msg.dp_accel]
    #print [dpa, ppa, mma]
    #print msg.mm_accel, msg.pp_accel, msg.dp_accel
    j2 = math.atan2(ppa[2], ppa[0]) - math.atan2(dpa[2], dpa[0])
    self.cfs[finger_idx].joints_inertial[2] = j2
    H2R = 3.14159 * 2.0 / 36.0  # hall state to radians conversion
    R0_INV = 1.0 / 231.0
    R1_INV = 1.0 / 196.7
    R2_INV = 1.0 / 170.0
    self.cfs[finger_idx].joints_hall[0] = H2R * R0_INV *   msg.hall_pos[0]
    self.cfs[finger_idx].joints_hall[1] = H2R * R1_INV * ( msg.hall_pos[1] + \
                                                           msg.hall_pos[0] )
    self.cfs[finger_idx].joints_hall[2] = H2R * R2_INV * ( msg.hall_pos[2] - \
                                                           msg.hall_pos[1] - \
                                                           2*msg.hall_pos[0])
    self.finger_pubs[finger_idx].publish(self.cfs[finger_idx])
    #if finger_idx == 0:
    #  print "  %.3f" % (j2*180./3.1415)

  def mobo_cb(self, msg):
    pass

  def palm_cb(self, msg):
    pass

############################################################################
if __name__ == '__main__':
  hse = HandStateEstimator(rospy.myargv())
  rospy.spin()
