#!/usr/bin/env python
import roslib; roslib.load_manifest('openarms')
import rospy
from sensor_msgs.msg import JointState
import sys, select, tty, termios

print_next_js = False
spacebar_counter = 1

def joint_callback(js):
  global print_next_js, spacebar_counter
  if print_next_js:
    print_next_js = False
    print "state_%d %s"%(spacebar_counter , ' '.join(str(round(x,3)) for x in js.position))
    spacebar_counter += 1

old_termios = termios.tcgetattr(sys.stdin)
rospy.init_node('print_joint_states', anonymous=True)
rospy.Subscriber("joint_states", JointState, joint_callback)
try:
  tty.setcbreak(sys.stdin.fileno())
  while not rospy.core.is_shutdown():
    rospy.sleep(0.05)
    i,o,e = select.select([sys.stdin],[],[],0)
    for s in i:
      if s == sys.stdin:
        c = sys.stdin.read(1)
        if c == ' ':
          print_next_js = True
          break
except KeyboardInterrupt:
  rospy.core.signal_shutdown('keyboard_interrupt')
finally:
  termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_termios)
print "bai"
