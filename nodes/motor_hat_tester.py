#!/usr/bin/env python

'''
ROS node for testing the motor hat node
'''

import rospy
from hydro_system.msg import MotorHatCmd


def command_publisher():
  global speed
  global direction
  global increase
  global command_pub

  if speed == 0:
    rospy.loginfo('increasing speed')
    increase = True
    if direction == 'forward':
      direction = 'backward'
    else:
      direction = 'forward'
  elif speed == 255:
    rospy.loginfo('decreasing speed')
    increase = False

  if increase:
    speed += 1
  else: 
    speed -= 1

  msg = MotorHatCmd()
  msg.commands = [direction, 'release', 'release', 'release']
  msg.speeds = [speed, 0, 0, 0]

  command_pub.publish(msg)

  rospy.sleep(0.05)



if __name__ == '__main__':

  rospy.init_node('motor_tester', anonymous=True)
  command_pub = rospy.Publisher('/motor_hat_cmd', MotorHatCmd, queue_size=1)

  direction = 'forward'
  increase = True
  speed = 0

  while not rospy.is_shutdown():
    try:
      command_publisher()
    except rospy.ROSInterruptException:
      pass
