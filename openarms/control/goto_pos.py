#!/usr/bin/env python

import roslib; roslib.load_manifest('openarms')
import rospy, sys
import math
from openarms.msg import StepperTarget
from joy.msg import Joy
from sensor_msgs.msg import JointState

g_vel = [0,0,0,0]
g_pos = None
g_enable = False
g_prev_buttons = None
g_pos_tgt = [0,0,0,0]

def joy_update(msg):
  global g_vel, g_enable, g_prev_buttons
  g_enable = bool(msg.buttons[0])
  #if g_prev_buttons and len(msg.buttons) >= 1: // button press detect
  g_prev_buttons = msg.buttons

def joint_state_update(msg):
  global g_pos
  g_pos = msg.position

def clamp(x, lower, upper):
  if x > upper:
    x = upper
  elif x < lower:
    x = lower
  return x

def joint_target_update(msg):
  global g_pos_tgt
  g_pos_tgt[0] = clamp(msg.position[0], -0.7, 0.7)

rospy.init_node('goto_pos')
joy_sub = rospy.Subscriber('joy_0', Joy, joy_update)
pos_sub = rospy.Subscriber('joint_states', JointState, joint_state_update)
tgt_sub = rospy.Subscriber('joint_targets', JointState, joint_target_update)
step_pub = rospy.Publisher('stepper_targets', StepperTarget)

r = rospy.Rate(100)
cmd_step = StepperTarget()
#tgt = [float(sys.argv[1])]
while not rospy.is_shutdown():
  cmd_step.mode = [2,2,2,2] # velocity
  cmd_step.vel = [0,0,0,0]
  cmd_step.pos = [0,0,0,0]
  if g_pos:
    err = [g_pos_tgt[0] - g_pos[0]]
    if g_enable:
      cmd_step.vel[0] = clamp(int(abs(int(err[0] * 1000))), 0, 150)
      if err[0] < 0:
        cmd_step.mode[0] += 128
    else:
      cmd_step.vel = [0,0,0,0]
    print "%10f %10f %10f %10f" % (g_pos[0], g_pos_tgt[0], err[0], cmd_step.vel[0])
  step_pub.publish(cmd_step)

  #print g_enable
  r.sleep()

