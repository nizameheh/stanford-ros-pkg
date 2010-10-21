#!/usr/bin/env python
import roslib; roslib.load_manifest('recyclerbot')
import rospy
import rosbag
import sys

bag = rosbag.Bag('/home/jiahui/test_data/bottles_and_can_data.bag')
outputf = open('/home/jiahui/stanford-ros-pkg/recyclerbot/include/recyclerbot/bottles_and_can_data.h', 'w')

i = 0
for topic, msg, t in bag.read_messages(topics=['/wide_stereo/points']):
	i += 1
	if (i == 5) :
		n = len(msg.points)
		
		print str('point number: '+str(n))
		print msg.channels[1].name
		
		outputf.write('int pointCloudNum = '+str(n)+';\n')
		outputf.write('double pointCloud[' + str(n) + '][3]={')
		for i in range(n-1):
			p = msg.points[i]
			outputf.write('{'+str(p.x)+', '+str(p.y)+', '+str(p.z)+'},\n')
			
		p = msg.points[n-1]
		outputf.write('{'+str(p.x)+', '+str(p.y)+', '+str(p.z)+'}};\n')
			
bag.close()
outputf.close()

