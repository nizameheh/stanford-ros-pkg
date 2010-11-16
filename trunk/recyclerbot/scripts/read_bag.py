#!/usr/bin/env python
import roslib; roslib.load_manifest('recyclerbot')
import rospy
import rosbag
import sys

bag = rosbag.Bag('/home/jiahui/test_data/bottles_and_can_data.bag')
outputf = open('/home/jiahui/stanford-ros-pkg/recyclerbot/include/recyclerbot/bottles_and_can_data.h', 'w')

i = 0
for topic, msg, t in bag.read_messages(topics=['/tf','/wide_stereo/points']):
	i += 1
	if (topic == '/tf'):
		trans = msg.transforms[0]
		
	if (topic == '/wide_stereo/points'):
		n = len(msg.points)
		
		print str('point number: '+str(n))
		print msg.channels[1].name
		
		outputf.write('int pointCloudNum = '+str(n)+';\n')
		outputf.write('double pointCloud[' + str(n) + '][3]={')
		for i in range(n-1):
			p = msg.points[i]
			transformPoint('')
			
			outputf.write('{'+str(p.x)+', '+str(p.y)+', '+str(p.z)+'},\n')
			
		p = msg.points[n-1]
		outputf.write('{'+str(p.x)+', '+str(p.y)+', '+str(p.z)+'}};\n')
	break
			
bag.close()
outputf.close()

