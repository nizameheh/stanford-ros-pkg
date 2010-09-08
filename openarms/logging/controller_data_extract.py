#!/usr/bin/env python
import roslib; roslib.load_manifest('openarms')
import sys
import rosbag

def strr(x):
  return str(round(x,4))

#print len(sys.argv)
if len(sys.argv) != 2:
  print "give ur bagfile as the second parameter"
  sys.exit(1)
bag = rosbag.Bag(sys.argv[1])
js = tj = None
for topic, msg, t in bag.read_messages(topics=['joint_states','target_joints']):
  if topic == 'joint_states':
    js = msg
  elif topic == 'target_joints':
    tj = msg
  if not js or not tj:
    continue
  
  if topic == 'joint_states':
    print " ".join([str(t)] + map(strr, js.position) + map(strr, tj.position))
