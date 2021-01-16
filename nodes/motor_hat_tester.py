#!/usr/bin/env python

'''
ROS node for testing the motor hat node
'''

import rospy
from hydro_system.msg import MotorHatCmd


def command_publisher():
  global motor
  global command_pub

  if motor == 0:
    motor = 1
  else:
    motor = 0

  msg = MotorHatCmd()
  msg.commands = ['release'] * 4
  msg.commands[motor] = 'forward'
  msg.speeds = [0] * 4
  msg.speeds[motor] = 100

  command_pub.publish(msg)

  rospy.sleep(2.0)



if __name__ == '__main__':

  rospy.init_node('motor_tester', anonymous=True)
  command_pub = rospy.Publisher('/motor_hat_cmd', MotorHatCmd, queue_size=1)

  motor = 1

  while not rospy.is_shutdown():
    try:
      command_publisher()
    except rospy.ROSInterruptException:
      pass
