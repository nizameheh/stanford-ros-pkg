#!/usr/bin/env python

import roslib; roslib.load_manifest('spacepoint')
import rospy
import tf
from geometry_msgs.msg import Quaternion

print "hello world"

def quat_cb(q):
  b = tf.TransformBroadcaster()
  b.sendTransform((0,0,0),(q.x, q.y, q.z, q.w), rospy.Time.now(), 'spacepoint2','world')

if __name__ == '__main__':
  rospy.init_node('quat2tf')
  rospy.Subscriber('spacepoint_quat', Quaternion, quat_cb)
  rospy.spin()

