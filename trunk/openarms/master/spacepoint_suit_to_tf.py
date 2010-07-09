#!/usr/bin/env python

import roslib; roslib.load_manifest('spacepoint')
import rospy
import tf
from geometry_msgs.msg import Quaternion

print "hello world"

def quat1_cb(q):
  b = tf.TransformBroadcaster()
  b.sendTransform((0,0,0),(q.x, q.y, q.z, q.w), rospy.Time.now(), 'spacepoint1','world')
def quat2_cb(q):
  b = tf.TransformBroadcaster()
  b.sendTransform((0,0,0),(q.x, q.y, q.z, q.w), rospy.Time.now(), 'spacepoint2','world')
def quat3_cb(q):
  b = tf.TransformBroadcaster()
  b.sendTransform((0,0,0),(q.x, q.y, q.z, q.w), rospy.Time.now(), 'spacepoint3','world')

if __name__ == '__main__':
  rospy.init_node('quat2tf')
  rospy.Subscriber('spacepoint1_quat', Quaternion, quat1_cb)
  rospy.Subscriber('spacepoint2_quat', Quaternion, quat2_cb)
  rospy.Subscriber('spacepoint3_quat', Quaternion, quat3_cb)
  rospy.spin()

