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

  def simplify_angle(self, x):
    if x < -3.1415:
      return x + 2 * 3.1415 
    elif x > 3.1415:
      return x - 2 * 3.1415
    else:
      return x

  def normalize_accel(self, a):
    mag = math.sqrt(a[0]*a[0] + a[1]*a[1] + a[2]*a[2])
    if mag == 0:
      mag = 1 # we're hosed
    return [float(x) / mag for x in a]

  def finger_cb(self, finger_idx, msg):
    ########################################################################
    # calcluate joint angles based on accelerometers
    mma = self.normalize_accel(msg.mm_accel)
    #if (finger_idx == 3):
    #  print [msg.mm_accel, mma]
    ppa = self.normalize_accel(msg.pp_accel)
    dpa = self.normalize_accel(msg.dp_accel)
    self.cfs[finger_idx].joints_inertial_variance = [1e6, 1e6, 1e6] # no bueno
    #print [dpa, ppa, mma]
    #print msg.mm_accel, msg.pp_accel, msg.dp_accel
    # first, estimate the easy one: the distal joint.
    j2 = math.atan2(ppa[2], ppa[0]) - math.atan2(dpa[2], dpa[0])
    self.cfs[finger_idx].joints_inertial_variance[2] = 0.1 # todo
    self.cfs[finger_idx].joints_inertial[2] = j2
    # next, estimate the proximal joint
    x0 = mma[0]
    y0 = mma[1]
    z0 = mma[2]
    x2 = ppa[0]
    y2 = ppa[1]
    det_j0 = x0*x0 + y0*y0 - y2*y2
    j0_sol = []
    if det_j0 > 0:
      j0_sol = [math.atan2(y2,  math.sqrt(det_j0)) - math.atan2(y0,-x0), \
                math.atan2(y2, -math.sqrt(det_j0)) - math.atan2(y0,-x0)]
    j0_valid = []
    for j0 in j0_sol:
      x = self.simplify_angle(j0)
      if finger_idx < 3:
        if x > -1.6 and x < 1.6:
          j0_valid.append(x)
      elif finger_idx == 3: # thumb has more severe constraints (fortunately)
        if x > -0.1 and x < 1.6:
          j0_valid.append(x)
    if len(j0_valid) == 1:
      j0 = j0_valid[0]
      self.cfs[finger_idx].joints_inertial[0] = j0
      self.cfs[finger_idx].joints_inertial_variance[0] = 0.1 # finite
      # finally, estimate the middle joint
      gamma = (x0*math.cos(j0) + y0*math.sin(j0))
      det_j1 = gamma**2 - z0*z0 - x2*x2
      if det_j1 > 0:
        j1_sol = [math.atan2(x2,  math.sqrt(det_j1)) - math.atan2(gamma,-z0), \
                  math.atan2(x2, -math.sqrt(det_j1)) - math.atan2(gamma,-z0)]
        j1_valid = []
        for j1 in j1_sol:
          x = self.simplify_angle(j1)
          if x > -1.6 and x < 1.6:
            j1_valid.append(x)
        if finger_idx == 1:
          print j1_valid
      else:
        if finger_idx == 1:
          print "j1 estimate invalid"
        #if len(j1_valid) == 1:

    #if finger_idx == 3:
    #  print "j0: %.3f   %.3f" % (j0[0], j0[1])
    #  print j0_cand
      #print "  %.3f  %.3f" % (self.simplify_angle(j0[0]), self.simplify_angle(j0[1]))
      #print "    det: %.3f" % det
    ########################################################################
    # calculate joint angles based on hall sensor offsets
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
