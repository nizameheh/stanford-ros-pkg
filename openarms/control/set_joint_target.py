#!/usr/bin/env python
import roslib; roslib.load_manifest('openarms')
import sys
import rospy
from openarms.srv import *
from sensor_msgs.msg import *

if __name__ == "__main__":
  if len(sys.argv) == 9:
    js = sensor_msgs.msg.JointState()
    js.name = ['shoulder1','shoulder2','shoulder3',
               'elbow','wrist1','wrist2','wrist3','gripper']
    js.position = [0] * 8
    for x in xrange(0,8):
      js.position[x] = float(sys.argv[x+1])
    print js
    rospy.wait_for_service('set_joint_target')
    try:
      sjt = rospy.ServiceProxy('set_joint_target', SetJointTarget)
      sjt(js)
    except rospy.ServiceException, e:
      print "service call failed: %s" % e
    print "bai"
  else:
    print "usage: set_joint_target J0 J1 J2 J3 J4 J5 J6 GRIPPER"
    sys.exit(1)
