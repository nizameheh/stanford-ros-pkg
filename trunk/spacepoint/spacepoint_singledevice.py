#!/usr/bin/env python

import roslib; roslib.load_manifest('spacepoint')
import rospy, sys
import spacepoint
from geometry_msgs.msg import Quaternion

print "hello world"

if __name__ == '__main__':
  rospy.init_node('spacepoint')
  quat_pub = rospy.Publisher('spacepoint_quat', Quaternion)
  fusion = spacepoint.SpacePoint()
  q = Quaternion()
  while not rospy.is_shutdown():
    #print repr(fusion)
    fusion.update()
    q.x = fusion.quat[0]
    q.y = fusion.quat[1]
    q.z = fusion.quat[2]
    q.w = fusion.quat[3]
    quat_pub.publish(q)
